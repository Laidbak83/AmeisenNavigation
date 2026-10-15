#pragma once

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>

#include "../../../../AmeisenNavigation.Pack/src/AnpFormat.hpp"
#include "../../../../AmeisenNavigation.Pack/src/DetourUtils.hpp"
#include "../../Utils/Logger.hpp"
#include "TileRebuilder.hpp"

namespace OffMeshLinks {
/// Connections of one map from a file in TrinityCore's offmesh.txt format (WoW coordinates, bidirectional):
///   mapId tileX,tileY (x y z) (x y z) radius [areaId [flags [id]]]   // comment
/// The tile is ignored (Detour finds it from the start position). Without an area the connection gets
/// AREA_OFFMESH_LINK and walkable flags (polyFlags). The id (1 .. 2^24 - 1, 0 = none) is reported to clients per
/// path point (PATH_EX LINK_IDS) so a bot knows which script a link stands for (an elevator, a boat).
inline std::vector<OffMeshLink> ReadFile(const std::filesystem::path& file, int mapId, unsigned short polyFlags)
{
    std::vector<OffMeshLink> links;
    std::ifstream in(file);

    for (std::string line; std::getline(in, line);)
    {
        unsigned int map = 0;
        unsigned int tileX = 0;
        unsigned int tileY = 0;
        float from[3]{};
        float to[3]{};
        float radius = 0.0f;
        unsigned int area = AREA_OFFMESH_LINK;
        unsigned int flags = polyFlags;
        unsigned int id = 0;

        const int scanned =
            std::sscanf(line.c_str(), "%u %u,%u (%f %f %f) (%f %f %f) %f %u %u %u", &map, &tileX, &tileY, &from[0],
                        &from[1], &from[2], &to[0], &to[1], &to[2], &radius, &area, &flags, &id);

        if (scanned < 10 || static_cast<int>(map) != mapId)
        {
            continue;
        }

        const Vector3 start(from);
        const Vector3 end(to);

        if (!start.IsFinite() || !end.IsFinite() || !std::isfinite(radius) || radius <= 0.0f || radius > 50.0f ||
            area >= DT_MAX_AREAS || flags > 0xffff || id > OFFMESH_LINK_MAX_ID)
        {
            LogW("Skipping invalid off-mesh connection in ", file.string(), ": ", line);
            continue;
        }

        OffMeshLink link;
        start.CopyToRDCoords(link.start);
        end.CopyToRDCoords(link.end);
        link.radius = radius;
        link.bidirectional = true;
        link.area = static_cast<unsigned char>(area);
        link.flags = static_cast<unsigned short>(flags);
        link.userId = MakeOffMeshUserId(OffMeshLinkKind::CUSTOM, id);
        links.push_back(link);
    }

    return links;
}

struct DropSettings
{
    float maxHeight = 0.0f;  // highest drop (0 = off)
    float reach = 2.0f;      // how far beyond the ledge the landing point is searched
    float spacing = 3.0f;    // distance between links along a ledge
    unsigned short flags = NAV_DROP;
    int maxPerTile = 256;
};

/// One way drop-down links from ledges: points along the border edges of the navmesh with walkable ground
/// between the tile's climb height and maxHeight below, that the navmesh doesn't already connect closely.
///
/// The navmesh has no geometry: a ledge can't be told apart from a wall with ground on the other side (a balcony
/// next to the street, the upper floor of a building). Keep the reach small; exports with geometry checks are
/// better (the exporter's jump/drop links).
inline std::vector<OffMeshLink> FindDrops(const dtNavMesh& mesh, const DropSettings& settings)
{
    std::vector<OffMeshLink> links;

    if (settings.maxHeight <= 0.0f || settings.reach <= 0.0f)
    {
        return links;
    }

    NavMeshQueryPtr query(dtAllocNavMeshQuery());
    constexpr int SEARCH_NODES = 1024;

    if (!query || dtStatusFailed(query->init(&mesh, SEARCH_NODES)))
    {
        return links;
    }

    dtQueryFilter filter; // every polygon (flags 0xffff)
    constexpr int MAX_COLUMN = 32;
    constexpr int MAX_WALK_POLYS = 256;
    dtPolyRef column[MAX_COLUMN];
    dtPolyRef path[MAX_WALK_POLYS];

    for (int t = 0; t < mesh.getMaxTiles(); ++t)
    {
        const dtMeshTile* tile = mesh.getTile(t);

        if (!tile || !tile->header)
        {
            continue;
        }

        const dtMeshHeader& h = *tile->header;
        const float minDrop = std::max(h.walkableClimb, 0.5f) + 0.25f;
        const dtPolyRef base = mesh.getPolyRefBase(tile);
        const size_t tileStart = links.size();

        for (int p = 0; p < h.offMeshBase && static_cast<int>(links.size() - tileStart) < settings.maxPerTile; ++p)
        {
            const dtPoly& poly = tile->polys[p];
            const dtPolyRef ref = base | static_cast<dtPolyRef>(p);
            float cx = 0.0f;
            float cz = 0.0f;

            for (int v = 0; v < poly.vertCount; ++v)
            {
                cx += tile->verts[poly.verts[v] * 3] / static_cast<float>(poly.vertCount);
                cz += tile->verts[poly.verts[v] * 3 + 2] / static_cast<float>(poly.vertCount);
            }

            for (int e = 0; e < poly.vertCount; ++e)
            {
                if (poly.neis[e] != 0)
                {
                    continue; // inner edge or portal
                }

                const float* va = &tile->verts[poly.verts[e] * 3];
                const float* vb = &tile->verts[poly.verts[(e + 1) % poly.vertCount] * 3];
                const float dx = vb[0] - va[0];
                const float dz = vb[2] - va[2];
                const float length = std::sqrt(dx * dx + dz * dz);

                if (length < 0.5f)
                {
                    continue;
                }

                // Outward normal: perpendicular, away from the polygon's center.
                float nx = dz / length;
                float nz = -dx / length;

                if (nx * ((va[0] + vb[0]) * 0.5f - cx) + nz * ((va[2] + vb[2]) * 0.5f - cz) < 0.0f)
                {
                    nx = -nx;
                    nz = -nz;
                }

                const int samples = std::max(1, static_cast<int>(length / settings.spacing));

                for (int k = 0; k < samples; ++k)
                {
                    const float s = (static_cast<float>(k) + 0.5f) / static_cast<float>(samples);
                    Vector3 edge{va[0] + dx * s, 0.0f, va[2] + dz * s};
                    Vector3 onPoly;

                    if (dtStatusFailed(query->closestPointOnPoly(ref, edge, onPoly, nullptr)))
                    {
                        continue;
                    }

                    edge = onPoly;

                    // Not right next to another drop on this tile.
                    bool close = false;

                    for (size_t l = tileStart; l < links.size() && !close; ++l)
                    {
                        close = dtVdist(links[l].start, edge) < settings.spacing * 0.9f;
                    }

                    if (close)
                    {
                        continue;
                    }

                    for (const float d : {settings.reach * 0.5f, settings.reach})
                    {
                        const Vector3 probe{edge.x + nx * d, edge.y - (settings.maxHeight + minDrop) * 0.5f,
                                            edge.z + nz * d};
                        const float extents[3]{0.2f, (settings.maxHeight - minDrop) * 0.5f + 0.1f, 0.2f};
                        int count = 0;

                        if (dtStatusFailed(query->queryPolygons(probe, extents, &filter, column, &count, MAX_COLUMN)))
                        {
                            continue;
                        }

                        dtPolyRef landingRef = 0;
                        float landingHeight = -1e30f;

                        for (int c = 0; c < count; ++c)
                        {
                            float height = 0.0f;

                            if (column[c] != ref && dtStatusSucceed(query->getPolyHeight(column[c], probe, &height))
                                && edge.y - height >= minDrop && edge.y - height <= settings.maxHeight
                                && height > landingHeight)
                            {
                                landingRef = column[c];
                                landingHeight = height;
                            }
                        }

                        if (!landingRef)
                        {
                            continue;
                        }

                        const Vector3 landing{probe.x, landingHeight, probe.z};

                        // Skip it if walking there is about as short.
                        int pathCount = 0;
                        const dtStatus status =
                            query->findPath(ref, landingRef, edge, landing, &filter, path, &pathCount, MAX_WALK_POLYS);

                        if (dtStatusSucceed(status) && pathCount > 0 && path[pathCount - 1] == landingRef
                            && !dtStatusDetail(status, DT_OUT_OF_NODES))
                        {
                            float straight[MAX_WALK_POLYS * 3];
                            int corners = 0;
                            float walk = 0.0f;

                            if (dtStatusSucceed(query->findStraightPath(edge, landing, path, pathCount, straight,
                                                                        nullptr, nullptr, &corners, MAX_WALK_POLYS)))
                            {
                                for (int i = 1; i < corners; ++i)
                                {
                                    walk += dtVdist(&straight[(i - 1) * 3], &straight[i * 3]);
                                }
                            }

                            if (corners > 0 && walk < std::max(dtVdist(edge, landing) * 3.0f, 20.0f))
                            {
                                break;
                            }
                        }

                        OffMeshLink link;
                        link.start = Vector3{edge.x - nx * 0.2f, edge.y, edge.z - nz * 0.2f};
                        link.end = landing;
                        link.radius = 0.5f;
                        link.bidirectional = false;
                        link.area = AREA_OFFMESH_DROP;
                        link.flags = settings.flags;
                        link.userId = static_cast<unsigned int>(OffMeshLinkKind::DROP);
                        links.push_back(link);
                        break;
                    }
                }
            }
        }
    }

    return links;
}
} // namespace OffMeshLinks
