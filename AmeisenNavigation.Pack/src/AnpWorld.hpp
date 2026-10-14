#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "../../AmeisenNavigation/src/Utils/Logger.hpp"

#include "Anp.hpp"
#include "miniz/miniz.h"

/// ANW ("AmeisenNavigation World") static world: the collision geometry the navmesh was built from, with the
/// provenance of every triangle (terrain, WMO, M2, liquid, game object and its id), for queries the navmesh can't
/// answer (line of sight, every surface under a point, sweeping a body along a segment).
///
/// A .anw file is a ZIP archive named "{mapId:03}.anw" next to the .anp with the entries:
///   "world"  -> WorldHeader
///   "XX_YY"  -> one tile per ADT (XX = ADT x, YY = ADT y, like the ADT file names): TileHeader, then
///               uint16 vertices[vertCount][3]  (RD coordinates quantized between bmin and bmax)
///               uint32 indices[triCount][3]
///               uint8  areas[triCount]        (navmesh area id of the triangle, 0 = too steep to stand on)
///               uint32 sources[triCount]      (SourceKind << 28 | id)
///               uint16 chunkAreas[16][16]     (AreaTable id per ADT chunk [row: WoW x][column: WoW y], 0 = none)
/// Triangles crossing ADT borders are in every tile they overlap.
namespace AnpWorld {
constexpr char WORLD_MAGIC[4]{'A', 'N', 'W', 'H'};
constexpr char TILE_MAGIC[4]{'A', 'N', 'W', 'T'};
constexpr uint32_t VERSION = 1;
constexpr const char* WORLD_ENTRY = "world";

/// Upper bounds for a tile, everything above is a corrupt file.
constexpr uint32_t MAX_TILE_VERTS = 16u << 20;
constexpr uint32_t MAX_TILE_TRIS = 32u << 20;

constexpr float ADT_SIZE = 533.33333f;

inline std::string FileName(int mapId) { return std::format("{:03}.anw", mapId); }

/// ADT cell of a WoW position.
inline bool AdtOf(float wowX, float wowY, int& adtX, int& adtY) noexcept
{
    if (!std::isfinite(wowX) || !std::isfinite(wowY) || std::fabs(wowX) > 32.0f * ADT_SIZE ||
        std::fabs(wowY) > 32.0f * ADT_SIZE)
    {
        return false;
    }

    adtX = std::clamp(31 - static_cast<int>(std::floor(wowY / ADT_SIZE)), 0, 63);
    adtY = std::clamp(31 - static_cast<int>(std::floor(wowX / ADT_SIZE)), 0, 63);
    return true;
}

#pragma pack(push, 1)
struct WorldHeader
{
    char magic[4];
    uint32_t version;
    int32_t mapId;
};

struct TileHeader
{
    char magic[4];
    uint32_t version;
    int32_t adtX;
    int32_t adtY;
    float bmin[3]; // RD
    float bmax[3];
    uint32_t vertCount;
    uint32_t triCount;
};
#pragma pack(pop)

static_assert(sizeof(WorldHeader) == 12 && sizeof(TileHeader) == 48);

/// One ADT of the static world, vertices in RD coordinates.
struct Tile
{
    int adtX = 0;
    int adtY = 0;
    std::vector<float> verts; // x, y, z
    std::vector<uint32_t> indices;
    std::vector<uint8_t> areas;
    std::vector<uint32_t> sources;
    std::array<uint16_t, 256> chunkAreas{};

    size_t TriCount() const noexcept { return indices.size() / 3; }
};

/// Serialize a tile (vertices are quantized to 16 bit between the tile's bounds).
inline std::vector<unsigned char> Serialize(const Tile& tile)
{
    TileHeader header{};
    std::memcpy(header.magic, TILE_MAGIC, 4);
    header.version = VERSION;
    header.adtX = tile.adtX;
    header.adtY = tile.adtY;
    header.vertCount = static_cast<uint32_t>(tile.verts.size() / 3);
    header.triCount = static_cast<uint32_t>(tile.TriCount());

    for (int a = 0; a < 3; ++a)
    {
        header.bmin[a] = header.vertCount ? tile.verts[a] : 0.0f;
        header.bmax[a] = header.bmin[a];
    }

    for (size_t i = 0; i < tile.verts.size(); ++i)
    {
        header.bmin[i % 3] = std::min(header.bmin[i % 3], tile.verts[i]);
        header.bmax[i % 3] = std::max(header.bmax[i % 3], tile.verts[i]);
    }

    std::vector<unsigned char> out(sizeof(header) + header.vertCount * 6ull + header.triCount * (12ull + 1 + 4) +
                                   sizeof(tile.chunkAreas));
    unsigned char* p = out.data();
    const auto put = [&p](const void* data, size_t size) {
        if (size)
        {
            std::memcpy(p, data, size);
            p += size;
        }
    };

    put(&header, sizeof(header));

    for (size_t i = 0; i < tile.verts.size(); ++i)
    {
        const float range = header.bmax[i % 3] - header.bmin[i % 3];
        const float t = range > 0.0f ? (tile.verts[i] - header.bmin[i % 3]) / range : 0.0f;
        const auto q = static_cast<uint16_t>(std::lround(std::clamp(t, 0.0f, 1.0f) * 65535.0f));
        put(&q, 2);
    }

    put(tile.indices.data(), tile.indices.size() * 4);
    put(tile.areas.data(), tile.areas.size());
    put(tile.sources.data(), tile.sources.size() * 4);
    put(tile.chunkAreas.data(), sizeof(tile.chunkAreas));
    return out;
}

/// Parse a tile, nullopt if it is corrupt (sizes, indices, bounds).
inline std::optional<Tile> Deserialize(const unsigned char* data, size_t size)
{
    TileHeader header{};

    if (!data || size < sizeof(header))
    {
        return std::nullopt;
    }

    std::memcpy(&header, data, sizeof(header));

    if (std::memcmp(header.magic, TILE_MAGIC, 4) != 0 || header.version != VERSION ||
        header.vertCount > MAX_TILE_VERTS || header.triCount > MAX_TILE_TRIS || header.adtX < 0 || header.adtX > 63 ||
        header.adtY < 0 || header.adtY > 63)
    {
        return std::nullopt;
    }

    for (int a = 0; a < 3; ++a)
    {
        if (!std::isfinite(header.bmin[a]) || !std::isfinite(header.bmax[a]) || header.bmin[a] > header.bmax[a] ||
            std::fabs(header.bmin[a]) > 1e6f || std::fabs(header.bmax[a]) > 1e6f)
        {
            return std::nullopt;
        }
    }

    const uint64_t expected =
        sizeof(header) + header.vertCount * 6ull + header.triCount * (12ull + 1 + 4) + 256 * sizeof(uint16_t);

    if (expected != size)
    {
        return std::nullopt;
    }

    Tile tile;
    tile.adtX = header.adtX;
    tile.adtY = header.adtY;
    const unsigned char* p = data + sizeof(header);

    tile.verts.resize(header.vertCount * 3ull);

    for (size_t i = 0; i < tile.verts.size(); ++i, p += 2)
    {
        uint16_t q = 0;
        std::memcpy(&q, p, 2);
        const float range = header.bmax[i % 3] - header.bmin[i % 3];
        tile.verts[i] = header.bmin[i % 3] + range * (static_cast<float>(q) / 65535.0f);
    }

    // (memcpy with a null destination is undefined even for 0 bytes: empty tiles have none.)
    tile.indices.resize(header.triCount * 3ull);

    if (!tile.indices.empty())
    {
        std::memcpy(tile.indices.data(), p, tile.indices.size() * 4);
        p += tile.indices.size() * 4;
    }

    if (std::any_of(tile.indices.begin(), tile.indices.end(), [&](uint32_t i) { return i >= header.vertCount; }))
    {
        return std::nullopt;
    }

    tile.areas.assign(p, p + header.triCount);
    p += header.triCount;
    tile.sources.resize(header.triCount);

    if (!tile.sources.empty())
    {
        std::memcpy(tile.sources.data(), p, tile.sources.size() * 4);
        p += tile.sources.size() * 4;
    }
    std::memcpy(tile.chunkAreas.data(), p, sizeof(tile.chunkAreas));
    return tile;
}

/// Thread-safe .anw writer (tiles are deflated outside the lock).
class WorldWriter
{
    int MapId;
    mz_zip_archive Zip{};
    std::mutex Mutex;
    bool Valid = false;
    bool Finalized = false;
    size_t StoredBytes = 0;
    int TileCount = 0;

public:
    explicit WorldWriter(int mapId) noexcept : MapId(mapId)
    {
        mz_zip_zero_struct(&Zip);
        WorldHeader header{};
        std::memcpy(header.magic, WORLD_MAGIC, 4);
        header.version = VERSION;
        header.mapId = mapId;
        Valid = mz_zip_writer_init_heap(&Zip, 0, 64 * 1024) &&
                mz_zip_writer_add_mem(&Zip, WORLD_ENTRY, &header, sizeof(header), MZ_NO_COMPRESSION);
    }

    ~WorldWriter() noexcept { mz_zip_writer_end(&Zip); }

    WorldWriter(const WorldWriter&) = delete;
    WorldWriter& operator=(const WorldWriter&) = delete;

    int GetTileCount() const noexcept { return TileCount; }
    size_t GetStoredBytes() const noexcept { return StoredBytes; }

    bool AddTile(const Tile& tile)
    {
        const auto bytes = Serialize(tile);
        const auto name = Anp::TileEntryName(tile.adtX, tile.adtY);
        const auto crc = static_cast<mz_uint32>(mz_crc32(MZ_CRC32_INIT, bytes.data(), bytes.size()));
        const mz_uint compFlags =
            tdefl_create_comp_flags_from_zip_params(MZ_DEFAULT_LEVEL, -MZ_DEFAULT_WINDOW_BITS, MZ_DEFAULT_STRATEGY);
        size_t compressedSize = 0;
        void* compressed = tdefl_compress_mem_to_heap(bytes.data(), bytes.size(), &compressedSize, compFlags);
        bool ok = false;

        {
            const std::lock_guard lock(Mutex);

            if (Valid && !Finalized)
            {
                if (compressed && compressedSize > 0 && compressedSize < bytes.size())
                {
                    ok = mz_zip_writer_add_mem_ex(&Zip, name.c_str(), compressed, compressedSize, nullptr, 0,
                                                  static_cast<mz_uint>(MZ_DEFAULT_LEVEL) |
                                                      static_cast<mz_uint>(MZ_ZIP_FLAG_COMPRESSED_DATA),
                                                  static_cast<mz_uint64>(bytes.size()), crc);
                    StoredBytes += ok ? compressedSize : 0;
                }
                else
                {
                    ok = mz_zip_writer_add_mem(&Zip, name.c_str(), bytes.data(), bytes.size(), MZ_NO_COMPRESSION);
                    StoredBytes += ok ? bytes.size() : 0;
                }

                TileCount += ok ? 1 : 0;
            }
        }

        mz_free(compressed);
        return ok;
    }

    /// Write "{outputDir}/{mapId:03}.anw" (atomically via a temp file).
    bool Save(const std::filesystem::path& outputDir)
    {
        const std::lock_guard lock(Mutex);

        if (!Valid || Finalized)
        {
            return false;
        }

        Finalized = true;
        void* zipData = nullptr;
        size_t zipSize = 0;

        if (!mz_zip_writer_finalize_heap_archive(&Zip, &zipData, &zipSize))
        {
            return false;
        }

        const std::unique_ptr<void, decltype(&mz_free)> guard(zipData, &mz_free);

        try
        {
            std::filesystem::create_directories(outputDir);
            const auto finalPath = outputDir / FileName(MapId);
            auto tempPath = finalPath;
            tempPath += ".tmp";

            {
                std::ofstream file(tempPath, std::ios::binary | std::ios::trunc);
                file.write(static_cast<const char*>(zipData), static_cast<std::streamsize>(zipSize));

                if (!file.good())
                {
                    return false;
                }
            }

            std::error_code ec;
            std::filesystem::rename(tempPath, finalPath, ec);

            if (ec)
            {
                std::filesystem::remove(finalPath, ec);
                std::filesystem::rename(tempPath, finalPath, ec);
            }

            return !ec;
        }
        catch (const std::exception& e)
        {
            LogE("Failed to save ", FileName(MapId), ": ", e.what());
            return false;
        }
    }
};

/// Reads tiles of a .anw file on demand (the file stays on disk, only the central directory is in memory).
/// Thread-safe.
class WorldReader
{
    std::ifstream File;
    uint64_t FileSize = 0;
    mz_zip_archive Zip{};
    std::mutex Mutex;
    bool Valid = false;
    int MapId = -1;

    static size_t Read(void* opaque, mz_uint64 offset, void* buffer, size_t size)
    {
        auto* self = static_cast<WorldReader*>(opaque);

        if (offset > self->FileSize || size > self->FileSize - offset)
        {
            return 0;
        }

        self->File.clear();
        self->File.seekg(static_cast<std::streamoff>(offset));
        self->File.read(static_cast<char*>(buffer), static_cast<std::streamsize>(size));
        return static_cast<size_t>(self->File.gcount());
    }

public:
    explicit WorldReader(const std::filesystem::path& path)
    {
        mz_zip_zero_struct(&Zip);
        std::error_code ec;
        FileSize = std::filesystem::file_size(path, ec);

        if (ec || FileSize == 0)
        {
            return;
        }

        File.open(path, std::ios::binary);

        if (!File)
        {
            return;
        }

        Zip.m_pRead = &WorldReader::Read;
        Zip.m_pIO_opaque = this;

        if (!mz_zip_reader_init(&Zip, FileSize, 0))
        {
            return;
        }

        WorldHeader header{};
        const int index = mz_zip_reader_locate_file(&Zip, WORLD_ENTRY, nullptr, 0);
        Valid = index >= 0 &&
                mz_zip_reader_extract_to_mem(&Zip, static_cast<mz_uint>(index), &header, sizeof(header), 0) &&
                std::memcmp(header.magic, WORLD_MAGIC, 4) == 0 && header.version == VERSION;
        MapId = Valid ? header.mapId : -1;
    }

    ~WorldReader() noexcept { mz_zip_reader_end(&Zip); }

    WorldReader(const WorldReader&) = delete;
    WorldReader& operator=(const WorldReader&) = delete;

    bool IsValid() const noexcept { return Valid; }
    int GetMapId() const noexcept { return MapId; }

    bool HasTile(int adtX, int adtY)
    {
        const std::lock_guard lock(Mutex);
        return Valid && mz_zip_reader_locate_file(&Zip, Anp::TileEntryName(adtX, adtY).c_str(), nullptr, 0) >= 0;
    }

    /// The tile of ADT (adtX, adtY), nullopt if there is none or it is corrupt.
    std::optional<Tile> ReadTile(int adtX, int adtY)
    {
        std::vector<unsigned char> bytes;

        {
            const std::lock_guard lock(Mutex);

            if (!Valid)
            {
                return std::nullopt;
            }

            const int index = mz_zip_reader_locate_file(&Zip, Anp::TileEntryName(adtX, adtY).c_str(), nullptr, 0);
            mz_zip_archive_file_stat stat{};

            if (index < 0 || !mz_zip_reader_file_stat(&Zip, static_cast<mz_uint>(index), &stat) ||
                stat.m_uncomp_size > sizeof(TileHeader) + MAX_TILE_VERTS * 6ull + MAX_TILE_TRIS * 17ull + 512)
            {
                return std::nullopt;
            }

            bytes.resize(static_cast<size_t>(stat.m_uncomp_size));

            if (!mz_zip_reader_extract_to_mem(&Zip, static_cast<mz_uint>(index), bytes.data(), bytes.size(), 0))
            {
                return std::nullopt;
            }
        }

        auto tile = Deserialize(bytes.data(), bytes.size());
        return tile && tile->adtX == adtX && tile->adtY == adtY ? tile : std::nullopt;
    }
};
} // namespace AnpWorld
