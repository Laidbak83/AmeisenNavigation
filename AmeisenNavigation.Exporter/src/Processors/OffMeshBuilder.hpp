#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>

#include "../../../AmeisenNavigation.Pack/src/AnpFormat.hpp"
#include "../../../AmeisenNavigation.Pack/src/DetourUtils.hpp"
#include "../../../AmeisenNavigation/src/NavSources/OffMesh/TileRebuilder.hpp"
#include "../Utils/Structure.hpp"

// ─────────────────────────────────────────────
// Jump and drop links of a tile, checked against the source geometry.
//
// Candidates start on the border edges of the tile's navmesh (where walking ends: a fence, a wall, a ledge) and land
// on navmesh beyond them that walking doesn't reach closely. The vertical profile of the geometry between takeoff and
// landing decides what the link is:
//   - JUMP: over an obstacle no higher than maxJumpObstacle above the takeoff (a fence, a low wall, a railing), over
//           a gap, or up onto something no higher than maxJumpUp. Nothing may be in the way above it (a wall, a
//           ceiling) up to the agent's height.
//   - DROP: down a ledge (up to maxDropHeight) with nothing higher than a step in between.
// A wall between a ledge and the ground behind it rejects the link (what the server can't check, it has no geometry).
// ─────────────────────────────────────────────

struct OffMeshLinkConfig
{
    bool jumps = true;
    bool drops = true;
    float spacing = 4.0f;          // distance between links along an edge
    float maxJumpObstacle = 1.5f;  // highest obstacle (above the takeoff) jumped over
    float maxJumpUp = 1.25f;       // highest landing above the takeoff
    float maxJumpDistance = 4.0f;  // horizontal
    float maxDropHeight = 10.0f;
    int maxLinksPerTile = 2048;
};

/// Heights of the solid triangles of a tile along vertical lines (x/z grid of triangle indices).
class GeometryColumns
{
    const Structure& S;
    float MinX = 0.0f;
    float MinZ = 0.0f;
    float Cell = 2.0f;
    int Width = 0;
    int Height = 0;
    std::vector<uint32_t> Offsets;
    std::vector<uint32_t> Items;

public:
    /// Index the triangles [begin, end) (indices into s.tris) inside bmin/bmax (x/z). Liquids are left out: they
    /// aren't obstacles.
    GeometryColumns(const Structure& s, const uint32_t* begin, const uint32_t* end, const float* bmin,
                    const float* bmax, float cell = 2.0f)
        : S(s),
          MinX(bmin[0]),
          MinZ(bmin[2]),
          Cell(cell),
          Width(std::max(1, static_cast<int>(std::ceil((bmax[0] - bmin[0]) / cell)))),
          Height(std::max(1, static_cast<int>(std::ceil((bmax[2] - bmin[2]) / cell))))
    {
        Offsets.assign(static_cast<size_t>(Width) * Height + 1, 0);

        for (int pass = 0; pass < 2; ++pass)
        {
            std::vector<uint32_t> cursor;

            if (pass == 1)
            {
                for (size_t i = 1; i < Offsets.size(); ++i)
                {
                    Offsets[i] += Offsets[i - 1];
                }

                Items.resize(Offsets.back());
                cursor.assign(Offsets.begin(), Offsets.end() - 1);
            }

            for (const uint32_t* it = begin; it != end; ++it)
            {
                if (IsLiquidArea(S.triTypes[*it]))
                {
                    continue;
                }

                const Tri& t = S.tris[*it];
                const Vector3& a = S.verts[static_cast<size_t>(t.a)];
                const Vector3& b = S.verts[static_cast<size_t>(t.b)];
                const Vector3& c = S.verts[static_cast<size_t>(t.c)];
                const int x0 = ClampX(std::min({a.x, b.x, c.x}));
                const int x1 = ClampX(std::max({a.x, b.x, c.x}));
                const int z0 = ClampZ(std::min({a.z, b.z, c.z}));
                const int z1 = ClampZ(std::max({a.z, b.z, c.z}));

                for (int z = z0; z <= z1; ++z)
                {
                    for (int x = x0; x <= x1; ++x)
                    {
                        const size_t cellIndex = static_cast<size_t>(x + z * Width);

                        if (pass == 0)
                        {
                            Offsets[cellIndex + 1]++;
                        }
                        else
                        {
                            Items[cursor[cellIndex]++] = *it;
                        }
                    }
                }
            }
        }
    }

    /// Height intervals [lo, hi] where the vertical line through (x, z) meets a solid triangle: a point for floors
    /// and slopes, the triangle's height range for walls (steeper than ~78 degrees, they are hit when the line
    /// passes within 0.1 yards: rotated wall models are rarely exactly vertical).
    void HeightsAt(float x, float z, std::vector<std::pair<float, float>>& out) const
    {
        out.clear();
        const float fx = (x - MinX) / Cell;
        const float fz = (z - MinZ) / Cell;

        if (!(fx >= 0.0f && fz >= 0.0f && fx < static_cast<float>(Width) && fz < static_cast<float>(Height)))
        {
            return;
        }

        const size_t cellIndex = static_cast<size_t>(static_cast<int>(fx) + static_cast<int>(fz) * Width);

        for (uint32_t i = Offsets[cellIndex]; i < Offsets[cellIndex + 1]; ++i)
        {
            const Tri& t = S.tris[Items[i]];
            const Vector3& a = S.verts[static_cast<size_t>(t.a)];
            const Vector3& b = S.verts[static_cast<size_t>(t.b)];
            const Vector3& c = S.verts[static_cast<size_t>(t.c)];

            const Vector3 e1 = b - a;
            const Vector3 e2 = c - a;
            const float ny = e1.z * e2.x - e1.x * e2.z;
            const float nx = e1.y * e2.z - e1.z * e2.y;
            const float nz = e1.x * e2.y - e1.y * e2.x;
            const float n = std::sqrt(nx * nx + ny * ny + nz * nz);

            if (n < 1e-8f)
            {
                continue;
            }

            if (std::fabs(ny) < 0.2f * n)
            {
                if (DistanceToTriangleXZ(x, z, a, b, c) < 0.1f)
                {
                    out.emplace_back(std::min({a.y, b.y, c.y}), std::max({a.y, b.y, c.y}));
                }

                continue;
            }

            // Barycentric coordinates in x/z.
            const float d = (b.z - c.z) * (a.x - c.x) + (c.x - b.x) * (a.z - c.z);
            const float l1 = ((b.z - c.z) * (x - c.x) + (c.x - b.x) * (z - c.z)) / d;
            const float l2 = ((c.z - a.z) * (x - c.x) + (a.x - c.x) * (z - c.z)) / d;
            const float l3 = 1.0f - l1 - l2;
            constexpr float eps = -1e-4f;

            if (l1 >= eps && l2 >= eps && l3 >= eps)
            {
                const float h = l1 * a.y + l2 * b.y + l3 * c.y;
                out.emplace_back(h, h);
            }
        }
    }

private:
    int ClampX(float x) const noexcept
    {
        return std::clamp(static_cast<int>(std::floor((x - MinX) / Cell)), 0, Width - 1);
    }

    int ClampZ(float z) const noexcept
    {
        return std::clamp(static_cast<int>(std::floor((z - MinZ) / Cell)), 0, Height - 1);
    }

    static float DistanceToSegmentXZ(float x, float z, const Vector3& p, const Vector3& q) noexcept
    {
        const float dx = q.x - p.x;
        const float dz = q.z - p.z;
        const float lengthSq = dx * dx + dz * dz;
        const float t = lengthSq > 0.0f ? std::clamp(((x - p.x) * dx + (z - p.z) * dz) / lengthSq, 0.0f, 1.0f) : 0.0f;
        const float ex = p.x + dx * t - x;
        const float ez = p.z + dz * t - z;
        return std::sqrt(ex * ex + ez * ez);
    }

    static float DistanceToTriangleXZ(float x, float z, const Vector3& a, const Vector3& b, const Vector3& c) noexcept
    {
        return std::min({DistanceToSegmentXZ(x, z, a, b), DistanceToSegmentXZ(x, z, b, c),
                         DistanceToSegmentXZ(x, z, c, a)});
    }
};

namespace OffMeshBuilder {
/// What the geometry between takeoff and landing allows.
enum class LinkType
{
    NONE,
    JUMP,
    DROP,
};

/// Classify a link from takeoff s to landing q (RD coordinates) by the vertical profile of the geometry between them.
inline LinkType Classify(const GeometryColumns& geometry, const Vector3& s, const Vector3& q, float climb,
                         float agentHeight, const OffMeshLinkConfig& config,
                         std::vector<std::pair<float, float>>& heights)
{
    const float dy = q.y - s.y;
    const float limit = s.y + config.maxJumpObstacle;
    const float distance = dtVdist2D(s, q);
    const int samples = std::max(2, static_cast<int>(distance / 0.15f));
    float obstacle = -1e30f;
    bool gap = false;

    // Skip the first and last 0.3 yards: the ground at takeoff and landing.
    for (int i = 1; i < samples; ++i)
    {
        const float t = static_cast<float>(i) / static_cast<float>(samples);

        if (t * distance < 0.3f || (1.0f - t) * distance < 0.3f)
        {
            continue;
        }

        geometry.HeightsAt(s.x + (q.x - s.x) * t, s.z + (q.z - s.z) * t, heights);
        bool ground = false;

        for (const auto& [lo, hi] : heights)
        {
            // Anything reaching above the jump limit within head height blocks (wall, ceiling, tall obstacle).
            if (hi > limit && lo < limit + agentHeight)
            {
                return LinkType::NONE;
            }

            if (lo <= limit)
            {
                obstacle = std::max(obstacle, std::min(hi, limit));
            }

            ground |= hi >= std::min(s.y, q.y) - 1.0f && lo <= std::max(s.y, q.y) + climb;
        }

        gap |= !ground;
    }

    const bool overObstacle = obstacle > s.y + climb;

    if (dy < -climb)
    {
        if (-dy > config.maxDropHeight)
        {
            return LinkType::NONE;
        }

        // Over a railing it is a jump down.
        return overObstacle ? (config.jumps ? LinkType::JUMP : LinkType::NONE)
                            : (config.drops ? LinkType::DROP : LinkType::NONE);
    }

    if (!config.jumps || dy > config.maxJumpUp || distance > config.maxJumpDistance)
    {
        return LinkType::NONE;
    }

    // Up onto something, over an obstacle or over a gap. Otherwise the navmesh ends for another reason.
    return dy > climb || overObstacle || gap ? LinkType::JUMP : LinkType::NONE;
}

/// Links of a tile built as a navmesh of its own (tile x/z bounds tbmin/tbmax: landings outside are skipped, Detour
/// only connects off-mesh links to direct neighbours and the neighbour isn't built yet).
inline std::vector<OffMeshLink> FindLinks(const dtNavMesh& mesh, const GeometryColumns& geometry, float climb,
                                          float agentHeight, const float* tbmin, const float* tbmax,
                                          const OffMeshLinkConfig& config)
{
    std::vector<OffMeshLink> links;

    if (!config.jumps && !config.drops)
    {
        return links;
    }

    NavMeshQueryPtr query(dtAllocNavMeshQuery());
    constexpr int SEARCH_NODES = 2048;

    if (!query || dtStatusFailed(query->init(&mesh, SEARCH_NODES)))
    {
        return links;
    }

    dtQueryFilter filter;
    constexpr int MAX_COLUMN = 32;
    constexpr int MAX_PATH = 512;
    dtPolyRef column[MAX_COLUMN];
    dtPolyRef path[MAX_PATH];
    float straight[MAX_PATH * 3];
    std::vector<std::pair<float, float>> heights;
    const float maxDown = std::max(config.drops ? config.maxDropHeight : 0.0f,
                                   config.jumps ? config.maxJumpObstacle : 0.0f);
    const float maxUp = config.jumps ? config.maxJumpUp : 0.0f;
    const float maxReach = config.jumps ? config.maxJumpDistance : 3.0f;

    for (int t = 0; t < mesh.getMaxTiles(); ++t)
    {
        const dtMeshTile* tile = mesh.getTile(t);

        if (!tile || !tile->header)
        {
            continue;
        }

        const dtPolyRef base = mesh.getPolyRefBase(tile);

        for (int p = 0; p < tile->header->offMeshBase && static_cast<int>(links.size()) < config.maxLinksPerTile; ++p)
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
                    continue; // inner edge or tile portal
                }

                const float* va = &tile->verts[poly.verts[e] * 3];
                const float* vb = &tile->verts[poly.verts[(e + 1) % poly.vertCount] * 3];
                const float dx = vb[0] - va[0];
                const float dz = vb[2] - va[2];
                const float length = std::sqrt(dx * dx + dz * dz);

                if (length < 0.4f)
                {
                    continue;
                }

                float nx = dz / length;
                float nz = -dx / length;

                if (nx * ((va[0] + vb[0]) * 0.5f - cx) + nz * ((va[2] + vb[2]) * 0.5f - cz) < 0.0f)
                {
                    nx = -nx;
                    nz = -nz;
                }

                const int samples = std::max(1, static_cast<int>(length / config.spacing));

                for (int k = 0; k < samples && static_cast<int>(links.size()) < config.maxLinksPerTile; ++k)
                {
                    const float st = (static_cast<float>(k) + 0.5f) / static_cast<float>(samples);
                    Vector3 s;

                    if (dtStatusFailed(query->closestPointOnPoly(
                            ref, Vector3{va[0] + dx * st, va[1] + (vb[1] - va[1]) * st, va[2] + dz * st}, s, nullptr)))
                    {
                        continue;
                    }

                    if (std::any_of(links.begin(), links.end(),
                                    [&](const OffMeshLink& l) { return dtVdist(l.start, s) < config.spacing * 0.9f; }))
                    {
                        continue;
                    }

                    // Nearest landing beyond the edge.
                    for (float d = 0.8f; d <= maxReach; d += 0.4f)
                    {
                        const Vector3 probe{s.x + nx * d, s.y + (maxUp - maxDown) * 0.5f, s.z + nz * d};

                        if (probe.x < tbmin[0] + 0.5f || probe.x > tbmax[0] - 0.5f || probe.z < tbmin[2] + 0.5f
                            || probe.z > tbmax[2] - 0.5f)
                        {
                            break;
                        }

                        const float extents[3]{0.2f, (maxUp + maxDown) * 0.5f, 0.2f};
                        int count = 0;

                        if (dtStatusFailed(query->queryPolygons(probe, extents, &filter, column, &count, MAX_COLUMN)))
                        {
                            continue;
                        }

                        // Closest surface to the takeoff height.
                        dtPolyRef landingRef = 0;
                        float landingHeight = 0.0f;

                        for (int c = 0; c < count; ++c)
                        {
                            float h = 0.0f;

                            if (column[c] != ref && dtStatusSucceed(query->getPolyHeight(column[c], probe, &h))
                                && h - s.y <= maxUp && s.y - h <= maxDown
                                && (!landingRef || std::fabs(h - s.y) < std::fabs(landingHeight - s.y)))
                            {
                                landingRef = column[c];
                                landingHeight = h;
                            }
                        }

                        if (!landingRef)
                        {
                            continue;
                        }

                        const Vector3 q{probe.x, landingHeight, probe.z};
                        const LinkType type = Classify(geometry, s, q, climb, agentHeight, config, heights);

                        if (type == LinkType::NONE)
                        {
                            break; // the nearest landing decides, further ones would cross the same obstacle
                        }

                        // Skip it if walking there is about as short.
                        int pathCount = 0;
                        const dtStatus status =
                            query->findPath(ref, landingRef, s, q, &filter, path, &pathCount, MAX_PATH);

                        if (dtStatusSucceed(status) && pathCount > 0 && path[pathCount - 1] == landingRef)
                        {
                            int corners = 0;
                            float walk = 0.0f;

                            if (dtStatusSucceed(query->findStraightPath(s, q, path, pathCount, straight, nullptr,
                                                                        nullptr, &corners, MAX_PATH)))
                            {
                                for (int i = 1; i < corners; ++i)
                                {
                                    walk += dtVdist(&straight[(i - 1) * 3], &straight[i * 3]);
                                }
                            }

                            if (corners > 0 && walk < std::max(dtVdist(s, q) * 4.0f, dtVdist(s, q) + 10.0f))
                            {
                                break;
                            }
                        }

                        OffMeshLink link;
                        link.start = Vector3{s.x - nx * 0.1f, s.y, s.z - nz * 0.1f};
                        link.end = q;
                        link.radius = 0.5f;
                        link.bidirectional = false;
                        link.area = type == LinkType::JUMP ? AREA_OFFMESH_JUMP : AREA_OFFMESH_DROP;
                        link.flags = type == LinkType::JUMP ? NAV_JUMP : NAV_DROP;
                        link.userId = static_cast<unsigned int>(type == LinkType::JUMP ? OffMeshLinkKind::JUMP
                                                                                       : OffMeshLinkKind::DROP);
                        links.push_back(link);
                        break;
                    }
                }
            }
        }
    }

    return links;
}
} // namespace OffMeshBuilder
