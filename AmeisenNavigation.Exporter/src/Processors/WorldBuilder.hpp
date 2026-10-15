#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <map>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include <optional>
#include <string>
#include <type_traits>

#include "../../../AmeisenNavigation.Pack/src/AnpWorld.hpp"
#include "../Utils/CityMap.hpp"
#include "../Utils/FactionMap.hpp"
#include "../Utils/RoadMap.hpp"
#include "../Utils/Structure.hpp"
#include "../Utils/WaterMap.hpp"
#include "../Wow/AdtStructs.hpp"
#include "AdtTileProcessor.hpp"

/// What a navmesh tile is rebuilt from at runtime (game objects): the export settings ("build" entry of the .anw)
/// and the area rects around each ADT ("r_XX_YY" entries).
namespace WorldExtras {
constexpr const char* BUILD_ENTRY = "build";
constexpr char BUILD_MAGIC[4]{'A', 'N', 'W', 'B'};
constexpr char RECTS_MAGIC[4]{'A', 'N', 'W', 'R'};
constexpr uint32_t VERSION = 1;

/// Rects within this distance of an ADT are stored with it (the tile border reaches beyond the ADT).
constexpr float RECT_MARGIN = 16.0f;

inline std::string RectEntryName(int adtX, int adtY) { return "r_" + Anp::TileEntryName(adtX, adtY); }

static_assert(std::is_trivially_copyable_v<TileBuildConfig> && std::is_trivially_copyable_v<dtNavMeshParams>);
static_assert(std::is_trivially_copyable_v<WaterRect> && std::is_trivially_copyable_v<FactionRect>);

struct BuildInfo
{
    TileBuildConfig config;
    dtNavMeshParams params;
};

inline std::vector<unsigned char> SerializeBuild(TileBuildConfig config, const dtNavMeshParams& params)
{
    config.debugBmp = false;
    const uint32_t sizes[3]{VERSION, static_cast<uint32_t>(sizeof(config)), static_cast<uint32_t>(sizeof(params))};
    std::vector<unsigned char> out(4 + sizeof(sizes) + sizeof(config) + sizeof(params));
    std::memcpy(out.data(), BUILD_MAGIC, 4);
    std::memcpy(out.data() + 4, sizes, sizeof(sizes));
    std::memcpy(out.data() + 4 + sizeof(sizes), &config, sizeof(config));
    std::memcpy(out.data() + 4 + sizeof(sizes) + sizeof(config), &params, sizeof(params));
    return out;
}

/// nullopt if the entry was written by another layout of the settings (another exporter version).
inline std::optional<BuildInfo> DeserializeBuild(const std::vector<unsigned char>& bytes)
{
    uint32_t sizes[3]{};
    BuildInfo info{};

    if (bytes.size() != 4 + sizeof(sizes) + sizeof(info.config) + sizeof(info.params) ||
        std::memcmp(bytes.data(), BUILD_MAGIC, 4) != 0)
    {
        return std::nullopt;
    }

    std::memcpy(sizes, bytes.data() + 4, sizeof(sizes));

    if (sizes[0] != VERSION || sizes[1] != sizeof(info.config) || sizes[2] != sizeof(info.params))
    {
        return std::nullopt;
    }

    std::memcpy(&info.config, bytes.data() + 4 + sizeof(sizes), sizeof(info.config));
    std::memcpy(&info.params, bytes.data() + 4 + sizeof(sizes) + sizeof(info.config), sizeof(info.params));

    if (!ValidateNavMeshParams(info.params) || info.config.meshResolution < 16 || info.config.meshResolution > 16384 ||
        !(info.config.agentRadius >= 0.0f) || !(info.config.agentHeight > 0.0f) || !(info.config.agentClimb >= 0.0f) ||
        !(info.config.maxWadeDepth >= 0.0f && info.config.maxWadeDepth <= 1000.0f))
    {
        return std::nullopt;
    }

    info.config.debugBmp = false;
    return info;
}

/// Area rects of one ADT.
struct AdtRects
{
    std::vector<WaterRect> water;
    std::vector<RdRect> roads;
    std::vector<RdRect> cities;
    std::vector<FactionRect> factions;
};

/// Rects overlapping ADT (adtX, adtY) plus RECT_MARGIN (RD bounds of the ADT: x = WoW y, z = WoW x).
inline AdtRects CollectRects(int adtX, int adtY, const WaterMap* water, const RoadMap* roads, const CityMap* cities,
                             const FactionMap* factions)
{
    const float maxX = (32 - adtX) * TILESIZE + RECT_MARGIN;
    const float minX = (31 - adtX) * TILESIZE - RECT_MARGIN;
    const float maxZ = (32 - adtY) * TILESIZE + RECT_MARGIN;
    const float minZ = (31 - adtY) * TILESIZE - RECT_MARGIN;
    AdtRects out;

    if (water)
        water->Query(minX, minZ, maxX, maxZ, [&](const WaterRect& r) { out.water.push_back(r); });
    if (roads)
        roads->Query(minX, minZ, maxX, maxZ, [&](const RdRect& r) { out.roads.push_back(r); });
    if (cities)
        cities->Query(minX, minZ, maxX, maxZ, [&](const RdRect& r) { out.cities.push_back(r); });
    if (factions)
        factions->Query(minX, minZ, maxX, maxZ, [&](const FactionRect& r) { out.factions.push_back(r); });

    return out;
}

inline std::vector<unsigned char> SerializeRects(const AdtRects& rects)
{
    const uint32_t header[9]{VERSION,
                             static_cast<uint32_t>(sizeof(WaterRect)),
                             static_cast<uint32_t>(sizeof(RdRect)),
                             static_cast<uint32_t>(sizeof(FactionRect)),
                             static_cast<uint32_t>(rects.water.size()),
                             static_cast<uint32_t>(rects.roads.size()),
                             static_cast<uint32_t>(rects.cities.size()),
                             static_cast<uint32_t>(rects.factions.size()),
                             0};
    std::vector<unsigned char> out(4 + sizeof(header));
    std::memcpy(out.data(), RECTS_MAGIC, 4);
    std::memcpy(out.data() + 4, header, sizeof(header));
    const auto append = [&out](const auto& items) {
        const auto* p = reinterpret_cast<const unsigned char*>(items.data());
        out.insert(out.end(), p, p + items.size() * sizeof(items[0]));
    };
    append(rects.water);
    append(rects.roads);
    append(rects.cities);
    append(rects.factions);
    return out;
}

/// Add an ADT's stored rects to the maps (dropping invalid ones). False if the entry is corrupt.
inline bool DeserializeRects(const std::vector<unsigned char>& bytes, WaterMap& water, RoadMap& roads, CityMap& cities,
                             FactionMap& factions)
{
    uint32_t header[9]{};

    if (bytes.size() < 4 + sizeof(header) || std::memcmp(bytes.data(), RECTS_MAGIC, 4) != 0)
    {
        return false;
    }

    std::memcpy(header, bytes.data() + 4, sizeof(header));

    if (header[0] != VERSION || header[1] != sizeof(WaterRect) || header[2] != sizeof(RdRect) ||
        header[3] != sizeof(FactionRect) ||
        bytes.size() != 4 + sizeof(header) + uint64_t{header[4]} * sizeof(WaterRect) +
                            (uint64_t{header[5]} + header[6]) * sizeof(RdRect) +
                            uint64_t{header[7]} * sizeof(FactionRect))
    {
        return false;
    }

    const unsigned char* p = bytes.data() + 4 + sizeof(header);
    const auto finite = [](const RdRect& r) {
        return r.minX <= r.maxX && r.minZ <= r.maxZ; // RectMap::Add drops non-finite and absurd coordinates
    };

    for (uint32_t i = 0; i < header[4]; ++i, p += sizeof(WaterRect))
    {
        WaterRect r;
        std::memcpy(&r, p, sizeof(r));
        const auto neutral = GetNeutralArea(r.type);

        if (finite(r) &&
            std::all_of(std::begin(r.heights), std::end(r.heights), [](float h) { return std::isfinite(h); }) &&
            (neutral == LIQUID_WATER || neutral == LIQUID_OCEAN || neutral == LIQUID_LAVA || neutral == LIQUID_SLIME))
        {
            water.Add(r);
        }
    }

    for (uint32_t i = 0; i < header[5] + header[6]; ++i, p += sizeof(RdRect))
    {
        RdRect r;
        std::memcpy(&r, p, sizeof(r));

        if (finite(r))
        {
            (i < header[5] ? static_cast<RectMap<RdRect>&>(roads) : static_cast<RectMap<RdRect>&>(cities)).Add(r);
        }
    }

    for (uint32_t i = 0; i < header[7]; ++i, p += sizeof(FactionRect))
    {
        FactionRect r;
        std::memcpy(&r, p, sizeof(r));

        if (finite(r) && (r.faction == AreaFaction::Alliance || r.faction == AreaFaction::Horde))
        {
            factions.Add(r);
        }
    }

    return true;
}
} // namespace WorldExtras

/// Game object models for runtime objects: "gameobjects.ano" next to the maps, a .anw style archive with
/// "displays" (uint32 displayId, uint32 model)[] and one "m{model}" tile per model (model space, ADT 0,0).
namespace ObjectModels {
constexpr const char* FILE_NAME = "gameobjects.ano";
constexpr const char* DISPLAYS_ENTRY = "displays";

inline std::string ModelEntryName(uint32_t model) { return "m" + std::to_string(model); }

/// Model space structure as a tile (sources kept, chunk areas unused).
inline AnpWorld::Tile ToTile(const Structure& s)
{
    AnpWorld::Tile tile;

    for (const Vector3& v : s.verts)
    {
        tile.verts.insert(tile.verts.end(), {v.x, v.y, v.z});
    }

    for (size_t t = 0; t < s.tris.size(); ++t)
    {
        for (const int i : s.tris[t].points)
        {
            tile.indices.push_back(static_cast<uint32_t>(i));
        }

        tile.areas.push_back(static_cast<uint8_t>(s.triTypes[t]));
        tile.sources.push_back(s.SourceOf(t));
    }

    return tile;
}

/// Write the models (index = model number) and which display uses which.
inline bool Write(const std::filesystem::path& outputDir, const std::vector<std::pair<uint32_t, uint32_t>>& displays,
                  const std::vector<AnpWorld::Tile>& models)
{
    AnpWorld::WorldWriter writer(-1);
    std::vector<unsigned char> table(displays.size() * 8);
    std::memcpy(table.data(), displays.data(), table.size());
    bool ok = writer.AddEntry(DISPLAYS_ENTRY, table);

    for (size_t m = 0; m < models.size() && ok; ++m)
    {
        ok = writer.AddEntry(ModelEntryName(static_cast<uint32_t>(m)), AnpWorld::Serialize(models[m]));
    }

    return ok && writer.SaveAs(outputDir, FILE_NAME);
}
} // namespace ObjectModels

/// AreaTable ids per ADT chunk, keyed by ADT (x, y). Filled by the extraction threads.
struct ChunkAreaMap
{
    std::mutex mutex;
    std::map<std::pair<int, int>, std::array<uint16_t, 256>> areas; // [row (WoW x) * 16 + column (WoW y)]

    void Set(int adtX, int adtY, int row, int column, uint32_t areaId)
    {
        if (row < 0 || row > 15 || column < 0 || column > 15)
        {
            return;
        }

        const std::lock_guard lock(mutex);
        areas[{adtX, adtY}][static_cast<size_t>(row * 16 + column)] = static_cast<uint16_t>(std::min(areaId, 0xFFFFu));
    }
};

struct WorldBuildStats
{
    int tiles = 0;
    size_t triangles = 0; // stored, triangles crossing ADT borders count once per ADT
    size_t bytes = 0;     // compressed
};

/// Write the static world (`{mapId:03}.anw`) of the exported ADTs: the triangles the navmesh was built from (after
/// steep ones got the null area), bucketed per ADT, with their sources and the chunk area ids.
/// The build settings and area maps (optional) let the server rebuild tiles around game objects.
inline bool WriteWorld(const Structure& geometry, const std::vector<std::pair<int, int>>& adts,
                       const ChunkAreaMap& chunkAreas, int mapId, const std::filesystem::path& outputDir,
                       WorldBuildStats* stats = nullptr, const TileBuildConfig* config = nullptr,
                       const dtNavMeshParams* params = nullptr, const WaterMap* water = nullptr,
                       const RoadMap* roads = nullptr, const CityMap* cities = nullptr,
                       const FactionMap* factions = nullptr)
{
    std::map<std::pair<int, int>, size_t> adtIndex;

    for (const auto& adt : adts)
    {
        adtIndex.emplace(adt, adtIndex.size());
    }

    // RD x = WoW y -> ADT x, RD z = WoW x -> ADT y.
    const auto toAdt = [](float rd) { return std::clamp(31 - static_cast<int>(std::floor(rd / TILESIZE)), 0, 63); };
    std::vector<std::vector<uint32_t>> buckets(adtIndex.size());

    for (size_t t = 0; t < geometry.tris.size(); ++t)
    {
        const Tri& tri = geometry.tris[t];
        float bmin[3]{}, bmax[3]{};

        for (int a = 0; a < 3; ++a)
        {
            bmin[a] =
                std::min({geometry.verts[tri.a].pos[a], geometry.verts[tri.b].pos[a], geometry.verts[tri.c].pos[a]});
            bmax[a] =
                std::max({geometry.verts[tri.a].pos[a], geometry.verts[tri.b].pos[a], geometry.verts[tri.c].pos[a]});
        }

        for (int x = toAdt(bmax[0]); x <= toAdt(bmin[0]); ++x)
        {
            for (int y = toAdt(bmax[2]); y <= toAdt(bmin[2]); ++y)
            {
                if (const auto it = adtIndex.find({x, y}); it != adtIndex.end())
                {
                    buckets[it->second].push_back(static_cast<uint32_t>(t));
                }
            }
        }
    }

    AnpWorld::WorldWriter writer(mapId);
    std::atomic<size_t> triangles{0};
    std::atomic<bool> ok{true};
    std::vector<std::pair<int, int>> keys(adtIndex.size());

    for (const auto& [adt, index] : adtIndex)
    {
        keys[index] = adt;
    }

#pragma omp parallel for schedule(dynamic, 1)
    for (int b = 0; b < static_cast<int>(buckets.size()); ++b)
    {
        const auto& bucket = buckets[static_cast<size_t>(b)];
        const auto adt = keys[static_cast<size_t>(b)];

        if (bucket.empty())
        {
            continue;
        }

        AnpWorld::Tile tile;
        tile.adtX = adt.first;
        tile.adtY = adt.second;

        if (const auto it = chunkAreas.areas.find(adt); it != chunkAreas.areas.end())
        {
            tile.chunkAreas = it->second;
        }

        std::unordered_map<int, uint32_t> remap;
        remap.reserve(bucket.size() * 2);
        tile.indices.reserve(bucket.size() * 3);
        tile.areas.reserve(bucket.size());
        tile.sources.reserve(bucket.size());

        for (const uint32_t t : bucket)
        {
            for (const int v : geometry.tris[t].points)
            {
                const auto [it, inserted] = remap.try_emplace(v, static_cast<uint32_t>(tile.verts.size() / 3));

                if (inserted)
                {
                    const Vector3& p = geometry.verts[static_cast<size_t>(v)];
                    tile.verts.insert(tile.verts.end(), {p.x, p.y, p.z});
                }

                tile.indices.push_back(it->second);
            }

            const TriAreaId area = geometry.triTypes[t];
            uint32_t source = geometry.SourceOf(t);
            const auto neutral = GetNeutralArea(area);

            if (source == 0 && (neutral == LIQUID_WATER || neutral == LIQUID_OCEAN || neutral == LIQUID_LAVA ||
                                neutral == LIQUID_SLIME))
            {
                source = AnpWorld::MakeSource(AnpWorld::SourceKind::LIQUID, 0);
            }

            tile.areas.push_back(static_cast<uint8_t>(area));
            tile.sources.push_back(source);
        }

        triangles += bucket.size();

        if (!writer.AddTile(tile))
        {
            ok = false;
        }

        if (config && !writer.AddEntry(WorldExtras::RectEntryName(adt.first, adt.second),
                                       WorldExtras::SerializeRects(WorldExtras::CollectRects(
                                           adt.first, adt.second, water, roads, cities, factions))))
        {
            ok = false;
        }
    }

    if (config && params && !writer.AddEntry(WorldExtras::BUILD_ENTRY, WorldExtras::SerializeBuild(*config, *params)))
    {
        ok = false;
    }

    if (stats)
    {
        stats->tiles = writer.GetTileCount();
        stats->triangles = triangles;
        stats->bytes = writer.GetStoredBytes();
    }

    return ok && writer.GetTileCount() > 0 && writer.Save(outputDir);
}
