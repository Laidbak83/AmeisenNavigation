#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <regex>
#include <string>
#include <unordered_map>
#include <vector>

#include "../../Utils/Logger.hpp"
#include "../INavSource.hpp"
#include "MmapFormat.hpp"
#include "MmapTileHeader.hpp"

/// Loads TrinityCore/SkyFire style MMAPs (one .mmap with dtNavMeshParams + one .mmtile per tile).
class MmapNavSource : public INavSource
{
public:
    struct Patterns
    {
        std::string mmap;   // std::format pattern, arg 0 = mapId
        std::string mmtile; // std::format pattern, args: mapId, x, y
    };

private:
    static constexpr int TILE_GRID_SIZE = 64;

    /// Tile data allocated with dtAlloc, ownership moves into the navmesh (DT_TILE_FREE_DATA).
    struct TileBlob
    {
        unsigned char* data = nullptr;
        int size = 0;
    };

    std::filesystem::path MmapFolder;
    MmapFormat Format;
    Patterns FilePatterns;
    std::unordered_map<int, int> ParentMaps; // child map id -> parent map id (Map.db2 ParentMapID)
    NavMeshCache Cache;

public:
    explicit MmapNavSource(const std::filesystem::path& mmapFolder, MmapFormat format = MmapFormat::UNKNOWN,
                           const Patterns& customPatterns = {}, std::unordered_map<int, int> parentMaps = {})
        : MmapFolder(mmapFolder),
          Format(format == MmapFormat::UNKNOWN ? DetectFormat(mmapFolder) : format),
          ParentMaps(std::move(parentMaps))
    {
        if (Format == MmapFormat::CUSTOM)
        {
            FilePatterns = customPatterns;
        }
        else if (const auto* patterns = GetPatterns(Format))
        {
            FilePatterns = *patterns;
        }

        if (Format == MmapFormat::UNKNOWN)
        {
            LogW("Could not detect the MMAP format in \"", mmapFolder.string(), "\", no maps will be available");
        }
    }

    MmapFormat GetFormat() const noexcept { return Format; }

    static const Patterns* GetPatterns(MmapFormat format) noexcept
    {
        static const Patterns tc335a{"{:03}.mmap", "{:03}{:02}{:02}.mmtile"};
        static const Patterns sf548{"{:04}.mmap", "{:04}_{:02}_{:02}.mmtile"};

        switch (format)
        {
            case MmapFormat::TC335A: return &tc335a;
            case MmapFormat::SF548:
            case MmapFormat::TC_MASTER: return &sf548;
            default: return nullptr;
        }
    }

    /// Detect the naming scheme by looking at the .mmtile files in the folder, and TrinityCore master by the magic
    /// in front of its .mmap files.
    static MmapFormat DetectFormat(const std::filesystem::path& folder) noexcept
    {
        try
        {
            static const std::regex tc335a(R"(^\d{7}\.mmtile$)", std::regex::icase);
            static const std::regex sf548(R"(^\d{4}_\d{2}_\d{2}\.mmtile$)", std::regex::icase);
            static const std::regex sf548Map(R"(^\d{4}\.mmap$)", std::regex::icase);

            std::error_code ec;
            MmapFormat names = MmapFormat::UNKNOWN;
            bool magic = false;

            for (const auto& entry : std::filesystem::directory_iterator(folder, ec))
            {
                const auto name = entry.path().filename().string();

                if (names == MmapFormat::UNKNOWN && std::regex_match(name, tc335a))
                {
                    names = MmapFormat::TC335A;
                }
                else if (names == MmapFormat::UNKNOWN && std::regex_match(name, sf548))
                {
                    names = MmapFormat::SF548;
                }
                else if (!magic && std::regex_match(name, sf548Map))
                {
                    std::ifstream stream(entry.path(), std::ios::binary);
                    uint32_t value = 0;
                    magic = stream.read(reinterpret_cast<char*>(&value), sizeof(value)) && value == MMAP_MAGIC;
                }

                if (names == MmapFormat::TC335A || (names == MmapFormat::SF548 && magic))
                {
                    break;
                }
            }

            return names == MmapFormat::SF548 && magic ? MmapFormat::TC_MASTER : names;
        }
        catch (...)
        {
        }

        return MmapFormat::UNKNOWN;
    }

    dtNavMesh* Get(int mapId) noexcept override
    {
        return Cache.GetOrLoad(mapId, [this](int id) { return Finish(id, Load(id)); });
    }

    /// Read the navmesh parameters of a .mmap file: the raw dtNavMeshParams (TrinityCore 3.3.5a, SkyFire) or the
    /// MMAP v16+ header in front of them (TrinityCore master). The format is told apart by the magic, a raw
    /// dtNavMeshParams starts with orig[0] which never has the magic's bit pattern (2.2e8, far outside the world).
    static bool ReadNavMeshParams(const std::filesystem::path& mmapFile, dtNavMeshParams& params,
                                  uint32_t* offMeshConnections = nullptr)
    {
        std::ifstream stream(mmapFile, std::ios::binary);

        if (!stream.is_open())
        {
            return false;
        }

        uint32_t magic = 0;

        if (!stream.read(reinterpret_cast<char*>(&magic), sizeof(magic)))
        {
            LogE("Truncated mmap file: ", mmapFile.string());
            return false;
        }

        stream.seekg(0);

        if (magic != MMAP_MAGIC)
        {
            if (!stream.read(reinterpret_cast<char*>(&params), sizeof(params)))
            {
                LogE("Truncated mmap file: ", mmapFile.string());
                return false;
            }

            return true;
        }

        MmapNavMeshHeader header{};

        if (!stream.read(reinterpret_cast<char*>(&header), sizeof(header)))
        {
            LogE("Truncated mmap file: ", mmapFile.string());
            return false;
        }

        if (header.mmapVersion < MMAP_VERSION_NAVMESH_HEADER)
        {
            LogE("Unsupported mmap version ", header.mmapVersion, ": ", mmapFile.string());
            return false;
        }

        params = header.params;

        if (offMeshConnections)
        {
            *offMeshConnections = header.offmeshConnectionCount;
        }

        return true;
    }

private:
    NavMeshPtr Load(int mapId) const
    {
        if (FilePatterns.mmap.empty() || FilePatterns.mmtile.empty())
        {
            return nullptr;
        }

        const auto start = std::chrono::steady_clock::now();
        const std::vector<int> chain = MapChain(mapId);
        std::filesystem::path mmapFile;

        // Child maps (phases, Map.db2 ParentMapID) may lack their own files and use the parent's.
        for (const int id : chain)
        {
            mmapFile = MmapFolder / std::vformat(FilePatterns.mmap, std::make_format_args(id));

            if (std::error_code ec; std::filesystem::exists(mmapFile, ec))
            {
                break;
            }
        }

        dtNavMeshParams params{};
        uint32_t offMeshConnections = 0;

        if (std::error_code ec; !std::filesystem::exists(mmapFile, ec))
        {
            LogW("No navmesh for map ", mapId, " (missing ", mmapFile.string(), ")");
            return nullptr;
        }

        if (!ReadNavMeshParams(mmapFile, params, &offMeshConnections))
        {
            return nullptr;
        }

        NavMeshPtr navMesh(ValidateNavMeshParams(params) ? dtAllocNavMesh() : nullptr);

        if (!navMesh || dtStatusFailed(navMesh->init(&params)))
        {
            LogE("Failed to init navmesh for map ", mapId, " from ", mmapFile.string());
            return nullptr;
        }

        std::vector<TileBlob> tiles(TILE_GRID_SIZE * TILE_GRID_SIZE);
        const std::string tilePattern = FilePatterns.mmtile;

        // File I/O is the bottleneck, read tiles in parallel and add them sequentially afterwards.
#pragma omp parallel for schedule(dynamic, 16)
        for (int i = 0; i < TILE_GRID_SIZE * TILE_GRID_SIZE; ++i)
        {
            const int x = i / TILE_GRID_SIZE;
            const int y = i % TILE_GRID_SIZE;

            try
            {
                for (const int id : chain)
                {
                    const auto tileFile = MmapFolder / std::vformat(tilePattern, std::make_format_args(id, x, y));
                    tiles[static_cast<size_t>(i)] = ReadTile(tileFile);

                    if (tiles[static_cast<size_t>(i)].data)
                    {
                        break;
                    }
                }
            }
            catch (...)
            {
            }
        }

        int loaded = 0;
        int rejected = 0;

        for (auto& [data, size] : tiles)
        {
            if (!data)
            {
                continue;
            }

            if (dtStatusSucceed(navMesh->addTile(data, size, DT_TILE_FREE_DATA, 0, nullptr)))
            {
                loaded++;
            }
            else
            {
                rejected++;
                dtFree(data);
            }
        }

        const auto ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        LogI("Loaded map ", mapId, ": ", loaded, " tiles", rejected ? std::format(" ({} rejected)", rejected) : "",
             offMeshConnections ? std::format(", {} off-mesh connections", offMeshConnections) : "", " in ", ms,
             "ms");

        return navMesh;
    }

    /// The map followed by its parents (stops at cycles and after 16 steps).
    std::vector<int> MapChain(int mapId) const
    {
        std::vector<int> chain{mapId};

        for (auto it = ParentMaps.find(mapId); it != ParentMaps.end() && chain.size() < 16;
             it = ParentMaps.find(it->second))
        {
            if (std::find(chain.begin(), chain.end(), it->second) != chain.end())
            {
                break;
            }

            chain.push_back(it->second);
        }

        return chain;
    }

    static TileBlob ReadTile(const std::filesystem::path& tileFile) noexcept
    {
        std::ifstream stream(tileFile, std::ios::binary | std::ios::ate);

        if (!stream.is_open())
        {
            return {};
        }

        const auto fileSize = static_cast<std::streamoff>(stream.tellg());
        stream.seekg(0);

        MmapTileHeader header{};

        if (fileSize < static_cast<std::streamoff>(sizeof(header))
            || !stream.read(reinterpret_cast<char*>(&header), sizeof(header)))
        {
            return {};
        }

        if (header.mmapMagic != MMAP_MAGIC || header.dtVersion != static_cast<uint32_t>(DT_NAVMESH_VERSION)
            || header.mmapVersion < MMAP_VERSION || header.size == 0
            || static_cast<std::streamoff>(header.size) > fileSize - static_cast<std::streamoff>(sizeof(header)))
        {
            LogW("Skipping invalid mmtile: ", tileFile.string());
            return {};
        }

        auto* data = static_cast<unsigned char*>(dtAlloc(header.size, DT_ALLOC_PERM));

        if (!data)
        {
            return {};
        }

        if (!stream.read(reinterpret_cast<char*>(data), header.size) || !ValidateTileData(data, header.size))
        {
            LogW("Skipping corrupt mmtile: ", tileFile.string());
            dtFree(data);
            return {};
        }

        return {data, static_cast<int>(header.size)};
    }
};
