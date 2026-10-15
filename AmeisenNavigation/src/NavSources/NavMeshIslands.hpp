#pragma once

#include <cstdint>
#include <numeric>
#include <vector>

#include <DetourNavMesh.h>

/// Connected parts ("islands") of a navmesh: polygons joined by any link (neighbours, tile borders, off-mesh
/// connections of every kind). No path joins two islands, whatever the client's filter: a target on another island
/// (a roof, the inside of a closed building, a rock in the sea) is known to be unreachable before any search.
/// Built from a loaded navmesh, rebuilt when its tiles change (ReplaceTiles), read-only afterwards.
class NavMeshIslands
{
public:
    explicit NavMeshIslands(const dtNavMesh& mesh) : Mesh(&mesh)
    {
        const int maxTiles = mesh.getMaxTiles();
        Offsets.assign(static_cast<size_t>(maxTiles) + 1, 0);

        for (int i = 0; i < maxTiles; ++i)
        {
            const dtMeshTile* tile = mesh.getTile(i);
            Offsets[static_cast<size_t>(i) + 1] =
                Offsets[static_cast<size_t>(i)] +
                static_cast<uint32_t>(tile && tile->header ? tile->header->polyCount : 0);
        }

        const uint32_t total = Offsets[static_cast<size_t>(maxTiles)];
        std::vector<uint32_t> parent(total);
        std::iota(parent.begin(), parent.end(), 0u);

        const auto find = [&](uint32_t x) {
            while (parent[x] != x)
            {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }

            return x;
        };

        for (int i = 0; i < maxTiles; ++i)
        {
            const dtMeshTile* tile = mesh.getTile(i);

            if (!tile || !tile->header)
            {
                continue;
            }

            const unsigned int maxLinks = static_cast<unsigned int>(tile->header->maxLinkCount);

            for (int p = 0; p < tile->header->polyCount; ++p)
            {
                const uint32_t a = Offsets[static_cast<size_t>(i)] + static_cast<uint32_t>(p);
                unsigned int steps = 0;

                for (unsigned int k = tile->polys[p].firstLink; k != DT_NULL_LINK && k < maxLinks && steps < maxLinks;
                     k = tile->links[k].next, ++steps)
                {
                    uint32_t b = 0;

                    if (Index(tile->links[k].ref, b))
                    {
                        const uint32_t ra = find(a);
                        const uint32_t rb = find(b);

                        if (ra != rb)
                        {
                            parent[ra] = rb;
                        }
                    }
                }
            }
        }

        // Compact 1-based ids with their sizes.
        Ids.assign(total, 0);
        std::vector<uint32_t> rootId(total, 0);

        for (uint32_t i = 0; i < total; ++i)
        {
            const uint32_t root = find(i);

            if (!rootId[root])
            {
                rootId[root] = static_cast<uint32_t>(Sizes.size()) + 1;
                Sizes.push_back(0);
            }

            Ids[i] = rootId[root];
            Sizes[rootId[root] - 1]++;
        }
    }

    /// Island of a polygon, 0 if the reference is unknown or stale.
    uint32_t Of(dtPolyRef ref) const noexcept
    {
        uint32_t index = 0;
        return Index(ref, index) ? Ids[index] : 0;
    }

    uint32_t Count() const noexcept { return static_cast<uint32_t>(Sizes.size()); }
    uint32_t PolyCount() const noexcept { return static_cast<uint32_t>(Ids.size()); }

    /// Polygons of the biggest island.
    uint32_t LargestSize() const noexcept
    {
        uint32_t largest = 0;

        for (const uint32_t size : Sizes)
        {
            largest = size > largest ? size : largest;
        }

        return largest;
    }

    size_t MemoryBytes() const noexcept { return (Ids.size() + Offsets.size() + Sizes.size()) * sizeof(uint32_t); }

private:
    const dtNavMesh* Mesh;
    std::vector<uint32_t> Offsets; // first polygon index of every tile (by tile index), plus the total
    std::vector<uint32_t> Ids;     // island of every polygon
    std::vector<uint32_t> Sizes;   // polygons per island

    bool Index(dtPolyRef ref, uint32_t& index) const noexcept
    {
        unsigned int salt = 0;
        unsigned int it = 0;
        unsigned int ip = 0;
        Mesh->decodePolyId(ref, salt, it, ip);

        if (it >= static_cast<unsigned int>(Mesh->getMaxTiles()))
        {
            return false;
        }

        const dtMeshTile* tile = Mesh->getTile(static_cast<int>(it));

        if (!tile || !tile->header || tile->salt != salt || ip >= static_cast<unsigned int>(tile->header->polyCount))
        {
            return false;
        }

        index = Offsets[it] + ip;
        return true;
    }
};
