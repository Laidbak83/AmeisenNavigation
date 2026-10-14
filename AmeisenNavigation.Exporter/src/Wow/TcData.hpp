#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../Utils/CityMap.hpp"
#include "../Utils/FactionMap.hpp"
#include "../Utils/Matrix4x4.hpp"
#include "../Utils/Structure.hpp"
#include "../Utils/WaterMap.hpp"
#include "AdtChunkExtractor.hpp"
#include "AdtStructs.hpp"

// ─────────────────────────────────────────────
// Input from TrinityCore's extractors (master branch, every client since Legion): `mapextractor` writes one `.map`
// file per ADT (terrain heights, holes, liquids, area ids), `vmap4extractor` + `vmap4assembler` write the WMO and M2
// collision models (`.vmo`), their spawns per tile (`.vmtile`) and the game object models
// (`GameObjectModels.dtree`). Formats from TrinityCore's MapDefines.h, TerrainBuilder.cpp, MapTree.cpp,
// ModelInstance.cpp, WorldModel.cpp and GameObjectModel.cpp (October 2026). The same geometry TrinityCore's
// mmaps_generator uses, built by our tile pipeline (climb, fences, jump links).
// ─────────────────────────────────────────────

namespace TcData {
constexpr std::array<char, 4> MAP_MAGIC{'M', 'A', 'P', 'S'};
constexpr uint32_t MAP_VERSION = 10;
constexpr std::array<char, 4> MAP_HEIGHT_MAGIC{'M', 'H', 'G', 'T'};
constexpr std::array<char, 4> MAP_LIQUID_MAGIC{'M', 'L', 'I', 'Q'};
constexpr std::array<char, 4> MAP_AREA_MAGIC{'A', 'R', 'E', 'A'};
constexpr char VMAP_MAGIC[8]{'V', 'M', 'A', 'P', '_', '4', '.', 'E'};

constexpr int V9 = 129;
constexpr int V8 = 128;
constexpr float INVALID_LIQUID_HEIGHT = -2000.0f; // squares without liquid inside the stored liquid rect

constexpr uint32_t HEIGHT_NO_HEIGHT = 0x01;
constexpr uint32_t HEIGHT_AS_INT16 = 0x02;
constexpr uint32_t HEIGHT_AS_INT8 = 0x04;
constexpr uint16_t AREA_NO_AREA = 0x01;
constexpr uint8_t LIQUID_NO_TYPE = 0x01;
constexpr uint8_t LIQUID_NO_HEIGHT = 0x02;

constexpr uint8_t LIQUID_FLAG_WATER = 0x01;
constexpr uint8_t LIQUID_FLAG_OCEAN = 0x02;
constexpr uint8_t LIQUID_FLAG_MAGMA = 0x04;
constexpr uint8_t LIQUID_FLAG_SLIME = 0x08;
constexpr uint8_t LIQUID_FLAG_DARK_WATER = 0x10; // deep ocean, nobody goes there

constexpr uint8_t MOD_PATH_ONLY = 0x04;
constexpr uint32_t MODEL_IS_M2 = 0x01;

#pragma pack(push, 1)
struct MapFileHeader
{
    std::array<char, 4> mapMagic;
    uint32_t versionMagic;
    uint32_t buildMagic;
    uint32_t areaMapOffset;
    uint32_t areaMapSize;
    uint32_t heightMapOffset;
    uint32_t heightMapSize;
    uint32_t liquidMapOffset;
    uint32_t liquidMapSize;
    uint32_t holesOffset;
    uint32_t holesSize;
};

struct MapAreaHeader
{
    std::array<char, 4> areaMagic;
    uint16_t flags;
    uint16_t gridArea;
};

struct MapHeightHeader
{
    std::array<char, 4> heightMagic;
    uint32_t flags;
    float gridHeight;
    float gridMaxHeight;
};

struct MapLiquidHeader
{
    std::array<char, 4> liquidMagic;
    uint8_t flags;
    uint8_t liquidFlags;
    uint16_t liquidType;
    uint8_t offsetX;
    uint8_t offsetY;
    uint8_t width;
    uint8_t height;
    float liquidLevel;
};
#pragma pack(pop)

static_assert(sizeof(MapFileHeader) == 44 && sizeof(MapAreaHeader) == 8 && sizeof(MapHeightHeader) == 16 &&
              sizeof(MapLiquidHeader) == 16);

/// Bounds checked reads from a file in memory.
class ByteReader
{
    const unsigned char* Data;
    size_t Size;
    size_t Pos = 0;

public:
    ByteReader(const std::vector<unsigned char>& bytes) noexcept : Data(bytes.data()), Size(bytes.size()) {}

    bool Seek(size_t pos) noexcept
    {
        Pos = pos;
        return pos <= Size;
    }

    size_t Position() const noexcept { return Pos; }

    bool Bytes(void* out, size_t length) noexcept
    {
        if (Pos > Size || length > Size - Pos)
        {
            return false;
        }

        if (length)
        {
            std::memcpy(out, Data + Pos, length);
        }

        Pos += length;
        return true;
    }

    template <typename T>
    bool Read(T& out) noexcept
    {
        return Bytes(&out, sizeof(T));
    }

    /// Read count values (count checked against the remaining size first, a corrupt count can't allocate GBs).
    template <typename T>
    bool ReadArray(std::vector<T>& out, size_t count)
    {
        if (Pos > Size || count > (Size - Pos) / sizeof(T))
        {
            return false;
        }

        out.resize(count);
        return Bytes(out.data(), count * sizeof(T));
    }

    bool Chunk(std::string_view magic) noexcept
    {
        char buffer[8]{};
        return magic.size() <= sizeof(buffer) && Bytes(buffer, magic.size()) &&
               std::memcmp(buffer, magic.data(), magic.size()) == 0;
    }
};

inline std::vector<unsigned char> ReadFile(const std::filesystem::path& path)
{
    std::ifstream in(path, std::ios::binary | std::ios::ate);

    if (!in)
    {
        return {};
    }

    const auto size = static_cast<std::streamoff>(in.tellg());

    if (size <= 0 || size > (512ll << 20))
    {
        return {};
    }

    std::vector<unsigned char> bytes(static_cast<size_t>(size));
    in.seekg(0);
    in.read(reinterpret_cast<char*>(bytes.data()), size);
    return in ? bytes : std::vector<unsigned char>{};
}

// ── .map files ──

/// One `.map` file. Squares and cells are indexed [row][column]: rows go south (decreasing WoW x), columns go east
/// (decreasing WoW y), 128 squares and 16 cells per ADT.
struct MapTile
{
    int adtX = 0; // ADT file name x (columns, WoW y), the second number in TrinityCore's file name
    int adtY = 0; // ADT file name y (rows, WoW x), the first number

    bool hasTerrain = false;
    std::vector<float> v9;                                          // 129 x 129 square corners
    std::vector<float> v8;                                          // 128 x 128 square centers
    std::array<std::array<std::array<uint8_t, 8>, 16>, 16> holes{}; // [cell row][cell column][row] bit column
    std::array<std::array<uint16_t, 16>, 16> areas{};

    bool hasLiquid = false;
    std::array<std::array<uint8_t, 16>, 16> liquidFlags{};
    int liquidOffsetX = 0;            // column of the first stored liquid vertex
    int liquidOffsetY = 0;            // row
    int liquidWidth = 0;              // stored vertices per row
    int liquidHeight = 0;             // stored rows
    std::vector<float> liquidHeights; // empty = flat at liquidLevel
    float liquidLevel = 0.0f;

    /// WoW coordinates of a square corner.
    float WowX(int row) const noexcept { return (32 - adtY) * TILESIZE - row * UNITSIZE; }
    float WowY(int column) const noexcept { return (32 - adtX) * TILESIZE - column * UNITSIZE; }

    bool IsHole(int row, int column) const noexcept
    {
        return (holes[row / 8][column / 8][row % 8] & (1u << (column % 8))) != 0;
    }

    /// Liquid height at a square corner, nullopt where there's none.
    std::optional<float> LiquidAt(int row, int column) const noexcept
    {
        if (liquidHeights.empty())
        {
            return liquidLevel;
        }

        const int y = row - liquidOffsetY;
        const int x = column - liquidOffsetX;

        if (y < 0 || x < 0 || y >= liquidHeight || x >= liquidWidth)
        {
            return std::nullopt;
        }

        const float h = liquidHeights[static_cast<size_t>(y * liquidWidth + x)];
        return std::isfinite(h) && h > INVALID_LIQUID_HEIGHT + 1.0f ? std::optional(h) : std::nullopt;
    }
};

/// Parse a `.map` file. nullopt if it isn't one (or not the version TrinityCore master writes).
inline std::optional<MapTile> ParseMapTile(const std::vector<unsigned char>& bytes, int adtX, int adtY)
{
    ByteReader reader(bytes);
    MapFileHeader header{};

    if (!reader.Read(header) || header.mapMagic != MAP_MAGIC || header.versionMagic != MAP_VERSION)
    {
        return std::nullopt;
    }

    MapTile tile;
    tile.adtX = adtX;
    tile.adtY = adtY;

    // Area ids per cell.
    MapAreaHeader area{};

    if (header.areaMapOffset && reader.Seek(header.areaMapOffset) && reader.Read(area) &&
        area.areaMagic == MAP_AREA_MAGIC)
    {
        if (area.flags & AREA_NO_AREA)
        {
            for (auto& row : tile.areas)
            {
                row.fill(area.gridArea);
            }
        }
        else if (!reader.Bytes(tile.areas.data(), sizeof(tile.areas)))
        {
            return std::nullopt;
        }
    }

    // Heights: floats, or uint8/uint16 steps between the min and max height, or flat (no height data).
    MapHeightHeader height{};

    if (!reader.Seek(header.heightMapOffset) || !reader.Read(height) || height.heightMagic != MAP_HEIGHT_MAGIC ||
        !std::isfinite(height.gridHeight) || !std::isfinite(height.gridMaxHeight))
    {
        return std::nullopt;
    }

    tile.hasTerrain = true;
    tile.v9.assign(V9 * V9, height.gridHeight);
    tile.v8.assign(V8 * V8, height.gridHeight);

    // TrinityCore writes flat terrain without heights (and its mmaps_generator then leaves the terrain out), it is
    // flat at gridHeight.
    if (!(height.flags & HEIGHT_NO_HEIGHT))
    {
        const auto unpack = [&]<typename T>(T, float steps) {
            std::vector<T> packed9, packed8;

            if (!reader.ReadArray(packed9, V9 * V9) || !reader.ReadArray(packed8, V8 * V8))
            {
                return false;
            }

            const float multiplier = steps > 0.0f ? (height.gridMaxHeight - height.gridHeight) / steps : 1.0f;

            for (size_t i = 0; i < packed9.size(); ++i)
            {
                tile.v9[i] = steps > 0.0f ? static_cast<float>(packed9[i]) * multiplier + height.gridHeight
                                          : static_cast<float>(packed9[i]);
            }

            for (size_t i = 0; i < packed8.size(); ++i)
            {
                tile.v8[i] = steps > 0.0f ? static_cast<float>(packed8[i]) * multiplier + height.gridHeight
                                          : static_cast<float>(packed8[i]);
            }

            return true;
        };

        const bool ok = (height.flags & HEIGHT_AS_INT8)    ? unpack(uint8_t{}, 255.0f)
                        : (height.flags & HEIGHT_AS_INT16) ? unpack(uint16_t{}, 65535.0f)
                                                           : unpack(float{}, 0.0f);

        if (!ok)
        {
            return std::nullopt;
        }

        // Corrupt heights: Structure::Clean drops the triangles using them.
    }

    // Liquids.
    MapLiquidHeader liquid{};

    if (header.liquidMapOffset && reader.Seek(header.liquidMapOffset) && reader.Read(liquid) &&
        liquid.liquidMagic == MAP_LIQUID_MAGIC)
    {
        if (liquid.flags & LIQUID_NO_TYPE)
        {
            for (auto& row : tile.liquidFlags)
            {
                row.fill(liquid.liquidFlags);
            }
        }
        else
        {
            std::array<std::array<uint16_t, 16>, 16> entries{};

            if (!reader.Bytes(entries.data(), sizeof(entries)) ||
                !reader.Bytes(tile.liquidFlags.data(), sizeof(tile.liquidFlags)))
            {
                return std::nullopt;
            }
        }

        tile.hasLiquid = true;
        tile.liquidLevel = liquid.liquidLevel;
        tile.liquidOffsetX = liquid.offsetX;
        tile.liquidOffsetY = liquid.offsetY;
        tile.liquidWidth = liquid.width;
        tile.liquidHeight = liquid.height;

        if (!(liquid.flags & LIQUID_NO_HEIGHT) &&
            !reader.ReadArray(tile.liquidHeights, static_cast<size_t>(liquid.width) * liquid.height))
        {
            return std::nullopt;
        }

        // Flat liquid at the "no liquid" height: no square had liquid.
        if (tile.liquidHeights.empty() &&
            !(std::isfinite(tile.liquidLevel) && tile.liquidLevel > INVALID_LIQUID_HEIGHT + 1.0f))
        {
            tile.hasLiquid = false;
        }
    }

    if (header.holesSize)
    {
        if (header.holesSize != sizeof(tile.holes) || !reader.Seek(header.holesOffset) ||
            !reader.Bytes(tile.holes.data(), sizeof(tile.holes)))
        {
            return std::nullopt;
        }
    }

    return tile;
}

/// Liquid area of a cell's TrinityCore liquid flags, 0 = none.
constexpr TriAreaId LiquidFlagsToArea(uint8_t flags) noexcept
{
    if (flags & LIQUID_FLAG_MAGMA)
        return LIQUID_LAVA;
    if (flags & LIQUID_FLAG_SLIME)
        return LIQUID_SLIME;
    if (flags & LIQUID_FLAG_OCEAN)
        return LIQUID_OCEAN;
    if (flags & LIQUID_FLAG_WATER)
        return LIQUID_WATER;
    return static_cast<TriAreaId>(0);
}

/// Add a `.map` tile: terrain (without holes), liquid surfaces and the water map, city and faction areas. Deep ocean
/// (TrinityCore's dark water) is left out like TrinityCore does, terrain and liquid.
inline void AddMapTile(const MapTile& tile, Structure* structure, WaterMap* water, CityMap* cities,
                       FactionMap* factions, const std::unordered_set<unsigned int>& cityAreas,
                       const std::unordered_map<unsigned int, unsigned char>& factionAreas)
{
    const auto deep = [&](int row, int column) {
        return tile.hasLiquid && (tile.liquidFlags[row / 8][column / 8] & LIQUID_FLAG_DARK_WATER);
    };

    if (tile.hasTerrain)
    {
        const size_t base = structure->verts.size();

        for (int r = 0; r < V9; ++r)
        {
            for (int c = 0; c < V9; ++c)
            {
                structure->verts.push_back(Vector3{tile.WowX(r), tile.WowY(c), tile.v9[r * V9 + c]}.ToRDCoords());
            }
        }

        for (int r = 0; r < V8; ++r)
        {
            for (int c = 0; c < V8; ++c)
            {
                structure->verts.push_back(
                    Vector3{tile.WowX(r) - HALFUNITSIZE, tile.WowY(c) - HALFUNITSIZE, tile.v8[r * V8 + c]}
                        .ToRDCoords());
            }
        }

        const auto corner = [&](int r, int c) { return base + static_cast<size_t>(r * V9 + c); };

        for (int r = 0; r < V8; ++r)
        {
            for (int c = 0; c < V8; ++c)
            {
                if (tile.IsHole(r, c) || deep(r, c))
                {
                    continue;
                }

                // Four triangles around the center, wound like the ADT terrain (ExtractTerrain).
                const size_t center = base + V9 * V9 + static_cast<size_t>(r * V8 + c);
                structure->tris.emplace_back(Tri{corner(r, c), center, corner(r, c + 1)});
                structure->tris.emplace_back(Tri{corner(r + 1, c + 1), center, corner(r + 1, c)});
                structure->tris.emplace_back(Tri{corner(r, c + 1), center, corner(r + 1, c + 1)});
                structure->tris.emplace_back(Tri{corner(r + 1, c), center, corner(r, c)});
                structure->triTypes.insert(structure->triTypes.end(), 4, TERRAIN_GROUND);
            }
        }
    }

    if (tile.hasLiquid)
    {
        for (int r = 0; r < V8; ++r)
        {
            for (int c = 0; c < V8; ++c)
            {
                const TriAreaId area = LiquidFlagsToArea(tile.liquidFlags[r / 8][c / 8]);

                // A square has liquid if its first corner has (the extractor writes the "no liquid" height there).
                const auto h = tile.LiquidAt(r, c);

                if (!area || deep(r, c) || !h)
                {
                    continue;
                }

                // Other corners without a height (squares next to ones without liquid) take the square's.
                const float hNE = tile.LiquidAt(r, c + 1).value_or(*h);
                const float hSW = tile.LiquidAt(r + 1, c).value_or(*h);
                const float hSE = tile.LiquidAt(r + 1, c + 1).value_or(*h);
                AddLiquidQuad(tile.WowX(r), tile.WowY(c), *h, hNE, hSW, hSE, area, water, structure);
            }
        }
    }

    for (int r = 0; r < 16; ++r)
    {
        for (int c = 0; c < 16; ++c)
        {
            const unsigned int areaId = tile.areas[r][c];

            if (!areaId)
            {
                continue;
            }

            const Vector3 nw{tile.WowX(r * 8), tile.WowY(c * 8), 0.0f};
            const Vector3 se{nw.x - CHUNKSIZE, nw.y - CHUNKSIZE, 0.0f};

            if (cities && cityAreas.contains(areaId))
            {
                cities->AddRect(nw, se);
            }

            if (const auto it = factionAreas.find(areaId); factions && it != factionAreas.end() && it->second != 0)
            {
                factions->AddRect(nw, se, it->second);
            }
        }
    }
}

// ── vmaps ──

/// One model spawn of a `.vmtile`. Positions are in TrinityCore's vmap space: WoW x = 32 * TILESIZE - pos.x, WoW y
/// = 32 * TILESIZE - pos.y.
struct ModelSpawn
{
    uint8_t flags = 0;
    uint8_t adtId = 0;
    uint32_t id = 0;
    float pos[3]{};
    float rot[3]{}; // degrees
    float scale = 1.0f;
    std::string name;
};

/// The spawns of a `.vmtile` file (empty if it isn't one).
inline std::vector<ModelSpawn> ParseTileSpawns(const std::vector<unsigned char>& bytes)
{
    ByteReader reader(bytes);
    std::vector<ModelSpawn> spawns;
    uint32_t count = 0;

    if (!reader.Chunk({VMAP_MAGIC, 8}) || !reader.Read(count))
    {
        return spawns;
    }

    for (uint32_t i = 0; i < count; ++i)
    {
        ModelSpawn spawn;
        uint32_t nameLength = 0;
        float bounds[6]{};

        if (!reader.Read(spawn.flags) || !reader.Read(spawn.adtId) || !reader.Read(spawn.id) ||
            !reader.Read(spawn.pos) || !reader.Read(spawn.rot) || !reader.Read(spawn.scale) ||
            ((spawn.flags & 0x01) && !reader.Read(bounds)) // MOD_HAS_BOUND
            || !reader.Read(nameLength) || nameLength > 500)
        {
            break;
        }

        spawn.name.resize(nameLength);

        if (!reader.Bytes(spawn.name.data(), nameLength))
        {
            break;
        }

        spawns.push_back(std::move(spawn));
    }

    return spawns;
}

/// A `.vmo` collision model (converted WMO or M2) in its own space.
struct WorldModel
{
    struct Liquid
    {
        uint32_t tilesX = 0;
        uint32_t tilesY = 0;
        float corner[3]{};
        uint32_t type = 0;          // LiquidType id
        std::vector<float> heights; // (tilesX + 1) * (tilesY + 1), vertex = y * (tilesX + 1) + x
        std::vector<uint8_t> flags; // tilesX * tilesY, tile = x + y * tilesX, 0x0F = no liquid
    };

    struct Group
    {
        std::vector<Vector3> verts;
        std::vector<uint32_t> indices;
        std::optional<Liquid> liquid;
    };

    bool isM2 = false;
    std::vector<Group> groups;
};

inline std::optional<WorldModel> ParseWorldModel(const std::vector<unsigned char>& bytes)
{
    ByteReader reader(bytes);
    WorldModel model;
    uint32_t chunkSize = 0, flags = 0, rootId = 0, groupCount = 0;

    if (!reader.Chunk({VMAP_MAGIC, 8}) || !reader.Chunk("WMOD") || !reader.Read(chunkSize) || !reader.Read(flags) ||
        !reader.Read(rootId))
    {
        return std::nullopt;
    }

    model.isM2 = (flags & MODEL_IS_M2) != 0;

    if (!reader.Chunk("GMOD"))
    {
        return model; // no groups
    }

    if (!reader.Read(groupCount) || groupCount > 4096)
    {
        return std::nullopt;
    }

    // BIH: bounds, tree nodes, object indices (only needed for ray casts).
    const auto skipBih = [&]() {
        float bounds[6];
        uint32_t treeSize = 0, objectCount = 0;
        return reader.Read(bounds) && reader.Read(treeSize) && reader.Seek(reader.Position() + 4ull * treeSize) &&
               reader.Read(objectCount) && reader.Seek(reader.Position() + 4ull * objectCount) &&
               reader.Position() <= bytes.size();
    };

    for (uint32_t g = 0; g < groupCount; ++g)
    {
        WorldModel::Group group;
        float bounds[6];
        uint32_t mogpFlags = 0, groupId = 0, count = 0;

        if (!reader.Read(bounds) || !reader.Read(mogpFlags) || !reader.Read(groupId) || !reader.Chunk("VERT") ||
            !reader.Read(chunkSize) || !reader.Read(count))
        {
            return std::nullopt;
        }

        // Groups without vertices end here (no triangles, BIH or liquid follow).
        if (count == 0)
        {
            model.groups.push_back(std::move(group));
            continue;
        }

        if (!reader.ReadArray(group.verts, count) || !reader.Chunk("TRIM") || !reader.Read(chunkSize) ||
            !reader.Read(count) || count > (1u << 28) || !reader.ReadArray(group.indices, size_t{count} * 3) ||
            !reader.Chunk("MBIH") || !skipBih() || !reader.Chunk("LIQU") || !reader.Read(chunkSize))
        {
            return std::nullopt;
        }

        if (chunkSize > 0)
        {
            WorldModel::Liquid liquid;

            if (!reader.Read(liquid.tilesX) || !reader.Read(liquid.tilesY) || !reader.Read(liquid.corner) ||
                !reader.Read(liquid.type))
            {
                return std::nullopt;
            }

            if (liquid.tilesX && liquid.tilesY)
            {
                if (liquid.tilesX > 4096 || liquid.tilesY > 4096 ||
                    !reader.ReadArray(liquid.heights, size_t{liquid.tilesX + 1} * (liquid.tilesY + 1)) ||
                    !reader.ReadArray(liquid.flags, size_t{liquid.tilesX} * liquid.tilesY))
                {
                    return std::nullopt;
                }

                group.liquid = std::move(liquid);
            }
            else if (!reader.Seek(reader.Position() + sizeof(float))) // a level without tiles: nothing to walk on
            {
                return std::nullopt;
            }
        }

        model.groups.push_back(std::move(group));
    }

    return model;
}

/// Rotation of a vmap spawn (TrinityCore: G3D::Matrix3::fromEulerAnglesZYX(rot.y, rot.x, rot.z), degrees).
inline std::array<float, 9> SpawnRotation(const float rot[3]) noexcept
{
    constexpr float toRadians = 3.14159265358979f / 180.0f;
    const float cz = std::cos(rot[1] * toRadians), sz = std::sin(rot[1] * toRadians);
    const float cy = std::cos(rot[0] * toRadians), sy = std::sin(rot[0] * toRadians);
    const float cx = std::cos(rot[2] * toRadians), sx = std::sin(rot[2] * toRadians);

    // Rz * Ry * Rx, row major.
    return {cz * cy,
            cz * sy * sx - sz * cx,
            cz * sy * cx + sz * sx,
            sz * cy,
            sz * sy * sx + cz * cx,
            sz * sy * cx - cz * sx,
            -sy,
            cy * sx,
            cy * cx};
}

/// Model space to WoW coordinates for a vmap spawn (TrinityCore's TerrainBuilder::loadVMap).
inline Vector3 SpawnToWow(const ModelSpawn& spawn, const std::array<float, 9>& r, const Vector3& v) noexcept
{
    const float x = (r[0] * v.x + r[1] * v.y + r[2] * v.z) * spawn.scale + spawn.pos[0];
    const float y = (r[3] * v.x + r[4] * v.y + r[5] * v.z) * spawn.scale + spawn.pos[1];
    const float z = (r[6] * v.x + r[7] * v.y + r[8] * v.z) * spawn.scale + spawn.pos[2];
    return {32.0f * TILESIZE - x, 32.0f * TILESIZE - y, z};
}

/// Liquid area of a LiquidType id, from a table (LiquidType.db2's SoundBank: 0 water, 1 ocean, 2 magma, 3 slime) or
/// the classic ids (1-20 come in water/ocean/magma/slime fours, the WMO ones TrinityCore's extractor writes are 13,
/// 14, 19 and 20).
inline TriAreaId LiquidTypeIdToArea(uint32_t type, const std::unordered_map<unsigned int, LiquidType>& table) noexcept
{
    if (const auto it = table.find(type); it != table.end())
    {
        return LiquidTypeToArea(it->second);
    }

    if (type >= 1 && type <= 20)
    {
        return LiquidTypeToArea(static_cast<LiquidType>((type - 1) & 3));
    }

    switch (type)
    {
        case 15:
        case 121:
        case 141:
            return LIQUID_LAVA;
        case 21:
        case 181:
            return LIQUID_SLIME;
        default:
            return LIQUID_WATER;
    }
}

/// Add a collision model with a model space to WoW transform. M2 triangles are flipped back (TrinityCore's
/// extractor reverses them, its mmaps_generator flips them again).
inline void AddWorldModel(const WorldModel& model, const std::function<Vector3(const Vector3&)>& toWow,
                          Structure* structure, TriAreaId area,
                          const std::unordered_map<unsigned int, LiquidType>& liquidTypes)
{
    for (const WorldModel::Group& group : model.groups)
    {
        if (!group.verts.empty())
        {
            std::vector<Vector3> verts;
            verts.reserve(group.verts.size());

            for (const Vector3& v : group.verts)
            {
                verts.push_back(toWow(v).ToRDCoords());
            }

            AppendMesh(
                structure, verts, static_cast<unsigned int>(group.indices.size() / 3),
                [&](unsigned int t, unsigned int& a, unsigned int& b, unsigned int& c) {
                    a = group.indices[t * 3];
                    b = group.indices[t * 3 + 1];
                    c = group.indices[t * 3 + 2];

                    if (model.isM2)
                    {
                        std::swap(a, c);
                    }

                    return true;
                },
                area);
        }

        if (!group.liquid)
        {
            continue;
        }

        const WorldModel::Liquid& liquid = *group.liquid;
        const TriAreaId liquidArea = LiquidTypeIdToArea(liquid.type, liquidTypes);
        const uint32_t vertsX = liquid.tilesX + 1;
        const auto vertex = [&](uint32_t x, uint32_t y) {
            const Vector3 local{liquid.corner[0] + x * UNITSIZE, liquid.corner[1] + y * UNITSIZE,
                                liquid.heights[y * vertsX + x]};
            return toWow(local).ToRDCoords();
        };

        std::lock_guard lock(structure->mutex);

        for (uint32_t y = 0; y < liquid.tilesY; ++y)
        {
            for (uint32_t x = 0; x < liquid.tilesX; ++x)
            {
                if ((liquid.flags[x + y * liquid.tilesX] & 0x0F) == 0x0F)
                {
                    continue;
                }

                // Same layout and winding as WMO liquids from the client files (ExtractWmo).
                const size_t base = structure->verts.size();
                structure->verts.push_back(vertex(x, y));
                structure->verts.push_back(vertex(x + 1, y));
                structure->verts.push_back(vertex(x, y + 1));
                structure->verts.push_back(vertex(x + 1, y + 1));
                structure->tris.emplace_back(Tri{base + 2, base, base + 1});
                structure->tris.emplace_back(Tri{base + 1, base + 3, base + 2});
                structure->triTypes.push_back(liquidArea);
                structure->triTypes.push_back(liquidArea);
            }
        }
    }
}

/// GameObjectModels.dtree: display id -> model file in the vmaps folder.
inline std::unordered_map<unsigned int, std::string> ParseGameObjectModels(const std::vector<unsigned char>& bytes)
{
    std::unordered_map<unsigned int, std::string> models;
    ByteReader reader(bytes);

    if (!reader.Chunk({VMAP_MAGIC, 8}))
    {
        return models;
    }

    while (true)
    {
        uint32_t displayId = 0, nameLength = 0;
        float bounds[6];

        if (!reader.Read(displayId) || !reader.Read(nameLength) || nameLength >= 500)
        {
            break;
        }

        std::string name(nameLength, '\0');

        if (!reader.Bytes(name.data(), nameLength) || !reader.Read(bounds))
        {
            break;
        }

        models[displayId] = std::move(name);
    }

    return models;
}

/// What TrinityCore's extractors wrote for each map: `.map` tiles (ADT x, y) and `.vmtile` tiles.
struct MapFiles
{
    std::vector<std::pair<int, int>> mapTiles;
    std::vector<std::pair<int, int>> vmapTiles;
};

/// Parse "NNNN_YY_XX.ext" (TrinityCore writes the tile row, i.e. the ADT's y, first). nullopt if it doesn't match.
inline std::optional<std::array<int, 3>> ParseTileFileName(const std::string& name, std::string_view extension)
{
    int map = 0, first = 0, second = 0;
    char dot = 0;

    if (name.size() != 4 + 1 + 2 + 1 + 2 + extension.size() || !name.ends_with(extension) ||
        std::sscanf(name.c_str(), "%4d_%2d_%2d%c", &map, &first, &second, &dot) != 4 || dot != '.' || first < 0 ||
        first > 63 || second < 0 || second > 63)
    {
        return std::nullopt;
    }

    return std::array{map, second, first}; // map, ADT x, ADT y
}

/// Maps with files in a TrinityCore data folder (`maps/`, `vmaps/`).
inline std::unordered_map<int, MapFiles> ListMaps(const std::filesystem::path& dataDir)
{
    std::unordered_map<int, MapFiles> maps;
    std::error_code ec;

    for (const auto& entry : std::filesystem::directory_iterator(dataDir / "maps", ec))
    {
        if (const auto tile = ParseTileFileName(entry.path().filename().string(), ".map"))
        {
            maps[(*tile)[0]].mapTiles.emplace_back((*tile)[1], (*tile)[2]);
        }
    }

    for (const auto& entry : std::filesystem::recursive_directory_iterator(dataDir / "vmaps", ec))
    {
        if (const auto tile = ParseTileFileName(entry.path().filename().string(), ".vmtile"))
        {
            maps[(*tile)[0]].vmapTiles.emplace_back((*tile)[1], (*tile)[2]);
        }
    }

    for (auto& [id, files] : maps)
    {
        std::sort(files.mapTiles.begin(), files.mapTiles.end());
        std::sort(files.vmapTiles.begin(), files.vmapTiles.end());
    }

    return maps;
}

/// Path of a tile file: maps/NNNN_YY_XX.map or vmaps/NNNN/NNNN_YY_XX.vmtile.
inline std::filesystem::path TileFile(const std::filesystem::path& dataDir, int mapId, int adtX, int adtY, bool vmap)
{
    char name[32];
    std::snprintf(name, sizeof(name), "%04d_%02d_%02d.%s", mapId, adtY, adtX, vmap ? "vmtile" : "map");

    if (!vmap)
    {
        return dataDir / "maps" / name;
    }

    char folder[8];
    std::snprintf(folder, sizeof(folder), "%04d", mapId);
    return dataDir / "vmaps" / folder / name;
}
} // namespace TcData
