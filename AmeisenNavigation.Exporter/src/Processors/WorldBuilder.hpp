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

#include "../../../AmeisenNavigation.Pack/src/AnpWorld.hpp"
#include "../Utils/Structure.hpp"
#include "../Wow/AdtStructs.hpp"

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
inline bool WriteWorld(const Structure& geometry, const std::vector<std::pair<int, int>>& adts,
                       const ChunkAreaMap& chunkAreas, int mapId, const std::filesystem::path& outputDir,
                       WorldBuildStats* stats = nullptr)
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
    }

    if (stats)
    {
        stats->tiles = writer.GetTileCount();
        stats->triangles = triangles;
        stats->bytes = writer.GetStoredBytes();
    }

    return ok && writer.GetTileCount() > 0 && writer.Save(outputDir);
}
