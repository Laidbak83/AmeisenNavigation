#pragma once

#include <algorithm>
#include <cmath>
#include <span>
#include <unordered_map>
#include <vector>

#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>

#include "../../Utils/Vector3.hpp"

/// An off-mesh connection to add to a loaded navmesh (RD coordinates).
struct OffMeshLink
{
    Vector3 start;
    Vector3 end;
    float radius = 1.0f; // how far from start/end the connection snaps to the navmesh
    bool bidirectional = false;
    unsigned char area = 0;
    unsigned short flags = 0;
    unsigned int userId = 0; // OffMeshLinkKind, reported to clients
};

/// What an off-mesh connection stands for, stored in the low byte of its userId (PATH_EX reports it per point); the
/// upper 24 bits hold the link's id from the off-mesh file (0 = none).
enum class OffMeshLinkKind : unsigned int
{
    UNKNOWN = 0, // from the navmesh file (e.g. TrinityCore's offmesh.txt baked in by the generator)
    CUSTOM = 1,  // sOffMeshFile
    DROP = 2,    // walk off a ledge (falls down)
    JUMP = 3,    // jump over a low obstacle or a gap
};

constexpr unsigned int OFFMESH_LINK_MAX_ID = 0xffffffu;

constexpr unsigned int MakeOffMeshUserId(OffMeshLinkKind kind, unsigned int id) noexcept
{
    return static_cast<unsigned int>(kind) | ((id & OFFMESH_LINK_MAX_ID) << 8);
}

constexpr OffMeshLinkKind OffMeshKindOf(unsigned int userId) noexcept
{
    return static_cast<OffMeshLinkKind>(userId & 0xffu);
}

constexpr unsigned int OffMeshIdOf(unsigned int userId) noexcept { return userId >> 8; }

/// Adds off-mesh connections to tiles that are already in a navmesh. Detour only accepts off-mesh connections when a
/// tile is created (dtCreateNavMeshData), so each affected tile is turned back into dtNavMeshCreateParams, gets the
/// new connections and replaces the old tile at the same index (and salt). The geometry is unchanged: x/z of the
/// polygon vertices are exact (quantized with the tile's cell size), heights are quantized with a cell height of at
/// most 1/60000 of the tile's height range, the detail mesh is copied unchanged.
///
/// Must not run while queries use the navmesh (poly refs of the rebuilt tiles change).
class TileRebuilder
{
    static constexpr unsigned short MESH_NULL_IDX = 0xffff;

    struct Buffers
    {
        std::vector<unsigned short> verts;
        std::vector<unsigned short> polys;
        std::vector<unsigned short> polyFlags;
        std::vector<unsigned char> polyAreas;
        std::vector<unsigned int> detailMeshes;
        std::vector<float> detailVerts;
        std::vector<unsigned char> detailTris;
        std::vector<float> offVerts;
        std::vector<float> offRadius;
        std::vector<unsigned short> offFlags;
        std::vector<unsigned char> offAreas;
        std::vector<unsigned char> offDir;
        std::vector<unsigned int> offUserId;
    };

public:
    /// Add the links to the tiles holding their start and end points (Detour stores a connection in its start
    /// tile, the end tile needs room for the link back). Links whose start isn't over a tile are dropped, as are
    /// connections ending more than one tile away (Detour only links direct neighbours). Returns the number of
    /// tiles rebuilt, -1 if a tile couldn't be rebuilt (it is left as it was).
    static int AddOffMeshLinks(dtNavMesh& mesh, std::span<const OffMeshLink> links)
    {
        std::unordered_map<const dtMeshTile*, std::vector<const OffMeshLink*>> perTile;

        for (const OffMeshLink& link : links)
        {
            if (!link.start.IsFinite() || !link.end.IsFinite() || !std::isfinite(link.radius) || link.radius <= 0.0f)
            {
                continue;
            }

            const dtMeshTile* startTile = TileAt(mesh, link.start);

            if (!startTile)
            {
                continue;
            }

            perTile[startTile].push_back(&link);

            if (const dtMeshTile* endTile = TileAt(mesh, link.end); endTile && endTile != startTile)
            {
                perTile[endTile].push_back(&link);
            }
        }

        int rebuilt = 0;
        bool failed = false;

        for (const auto& [tile, tileLinks] : perTile)
        {
            if (Rebuild(mesh, tile, tileLinks))
            {
                rebuilt++;
            }
            else
            {
                failed = true;
            }
        }

        return failed ? -1 : rebuilt;
    }

    /// Rebuild a tile with extra connections (empty = plain round trip, used by tests).
    static bool Rebuild(dtNavMesh& mesh, const dtMeshTile* tile, std::span<const OffMeshLink* const> extra)
    {
        if (!tile || !tile->header)
        {
            return false;
        }

        Buffers b;
        dtNavMeshCreateParams params{};

        if (!Extract(*tile, extra, b, params))
        {
            return false;
        }

        unsigned char* data = nullptr;
        int dataSize = 0;

        if (!dtCreateNavMeshData(&params, &data, &dataSize))
        {
            return false;
        }

        const dtTileRef ref = mesh.getTileRef(tile);

        if (dtStatusFailed(mesh.removeTile(ref, nullptr, nullptr)))
        {
            dtFree(data);
            return false;
        }

        // Same index and salt as before (tiles were added with DT_TILE_FREE_DATA, removeTile freed the old data).
        if (dtStatusFailed(mesh.addTile(data, dataSize, DT_TILE_FREE_DATA, ref, nullptr)))
        {
            dtFree(data);
            return false;
        }

        return true;
    }

private:
    static const dtMeshTile* TileAt(const dtNavMesh& mesh, const Vector3& pos)
    {
        int tx = 0;
        int ty = 0;
        mesh.calcTileLoc(pos, &tx, &ty);
        return mesh.getTileAt(tx, ty, 0);
    }

    /// dtPoly::neis -> rcPolyMesh neighbour encoding (see dtCreateNavMeshData).
    static unsigned short EncodeNeighbour(unsigned short nei) noexcept
    {
        if (nei == 0)
        {
            return MESH_NULL_IDX; // border
        }

        if ((nei & DT_EXT_LINK) == 0)
        {
            return static_cast<unsigned short>(nei - 1);
        }

        switch (nei & 0xff)
        {
            case 4:
                return 0x8000 | 0; // x-
            case 2:
                return 0x8000 | 1; // z+
            case 0:
                return 0x8000 | 2; // x+
            case 6:
                return 0x8000 | 3; // z-
            default:
                return MESH_NULL_IDX;
        }
    }

    static bool Extract(const dtMeshTile& tile, std::span<const OffMeshLink* const> extra, Buffers& b,
                        dtNavMeshCreateParams& params)
    {
        const dtMeshHeader& h = *tile.header;

        if (h.bvQuantFactor <= 0.0f || h.offMeshBase <= 0 || h.offMeshBase > h.polyCount)
        {
            return false;
        }

        const int groundPolys = h.offMeshBase;
        const int groundVerts = h.vertCount - h.offMeshConCount * 2;

        if (groundVerts <= 0)
        {
            return false;
        }

        const float cs = 1.0f / h.bvQuantFactor;
        float bmin[3]{h.bmin[0], h.bmin[1], h.bmin[2]};
        float maxY = h.bmax[1];

        for (int i = 0; i < groundVerts; ++i)
        {
            bmin[1] = std::min(bmin[1], tile.verts[i * 3 + 1]);
            maxY = std::max(maxY, tile.verts[i * 3 + 1]);
        }

        const float ch = std::max((maxY - bmin[1]) / 60000.0f, 1e-4f);
        b.verts.resize(static_cast<size_t>(groundVerts) * 3);

        for (int i = 0; i < groundVerts; ++i)
        {
            const float* v = &tile.verts[i * 3];
            const float q[3]{(v[0] - bmin[0]) / cs, (v[1] - bmin[1]) / ch, (v[2] - bmin[2]) / cs};

            for (int k = 0; k < 3; ++k)
            {
                const float r = std::round(q[k]);

                // x/z were quantized with this cell size when the tile was built, anything else isn't a Recast tile.
                if (r < 0.0f || r > 65535.0f || (k != 1 && std::fabs(q[k] - r) > 0.01f))
                {
                    return false;
                }

                b.verts[static_cast<size_t>(i) * 3 + k] = static_cast<unsigned short>(r);
            }
        }

        constexpr int nvp = DT_VERTS_PER_POLYGON;
        b.polys.assign(static_cast<size_t>(groundPolys) * nvp * 2, MESH_NULL_IDX);
        b.polyFlags.resize(static_cast<size_t>(groundPolys));
        b.polyAreas.resize(static_cast<size_t>(groundPolys));

        const bool hasDetail = tile.detailMeshes && h.detailMeshCount >= groundPolys;

        if (hasDetail)
        {
            b.detailMeshes.resize(static_cast<size_t>(groundPolys) * 4);
        }

        for (int i = 0; i < groundPolys; ++i)
        {
            const dtPoly& p = tile.polys[i];

            if (p.getType() != DT_POLYTYPE_GROUND)
            {
                return false;
            }

            unsigned short* dst = &b.polys[static_cast<size_t>(i) * nvp * 2];

            for (int j = 0; j < p.vertCount; ++j)
            {
                dst[j] = p.verts[j];
                dst[nvp + j] = EncodeNeighbour(p.neis[j]);
            }

            b.polyFlags[static_cast<size_t>(i)] = p.flags;
            b.polyAreas[static_cast<size_t>(i)] = p.getArea();

            if (hasDetail)
            {
                // Detail vertices of a poly: its own vertices first, then the extra detail vertices. Triangle
                // indices use the same numbering in both representations.
                const dtPolyDetail& pd = tile.detailMeshes[i];
                unsigned int* dm = &b.detailMeshes[static_cast<size_t>(i) * 4];
                dm[0] = static_cast<unsigned int>(b.detailVerts.size() / 3);
                dm[1] = p.vertCount + pd.vertCount;
                dm[2] = static_cast<unsigned int>(b.detailTris.size() / 4);
                dm[3] = pd.triCount;

                for (int j = 0; j < p.vertCount; ++j)
                {
                    b.detailVerts.insert(b.detailVerts.end(), &tile.verts[p.verts[j] * 3],
                                         &tile.verts[p.verts[j] * 3] + 3);
                }

                b.detailVerts.insert(b.detailVerts.end(), &tile.detailVerts[pd.vertBase * 3],
                                     &tile.detailVerts[(pd.vertBase + pd.vertCount) * 3]);
                b.detailTris.insert(b.detailTris.end(), &tile.detailTris[pd.triBase * 4],
                                    &tile.detailTris[(pd.triBase + pd.triCount) * 4]);
            }
        }

        const auto addConnection = [&](const float* start, const float* end, float radius, bool bidirectional,
                                       unsigned char area, unsigned short flags, unsigned int userId) {
            b.offVerts.insert(b.offVerts.end(), start, start + 3);
            b.offVerts.insert(b.offVerts.end(), end, end + 3);
            b.offRadius.push_back(radius);
            b.offDir.push_back(bidirectional ? DT_OFFMESH_CON_BIDIR : 0);
            b.offAreas.push_back(area);
            b.offFlags.push_back(flags);
            b.offUserId.push_back(userId);
        };

        // Connections already in the tile, then the new ones.
        for (int i = 0; i < h.offMeshConCount; ++i)
        {
            const dtOffMeshConnection& con = tile.offMeshCons[i];

            if (con.poly >= h.polyCount)
            {
                return false;
            }

            const dtPoly& p = tile.polys[con.poly];
            addConnection(&con.pos[0], &con.pos[3], con.rad, (con.flags & DT_OFFMESH_CON_BIDIR) != 0, p.getArea(),
                          p.flags, con.userId);
        }

        for (const OffMeshLink* link : extra)
        {
            addConnection(link->start, link->end, link->radius, link->bidirectional, link->area, link->flags,
                          link->userId);
        }

        params.verts = b.verts.data();
        params.vertCount = groundVerts;
        params.polys = b.polys.data();
        params.polyFlags = b.polyFlags.data();
        params.polyAreas = b.polyAreas.data();
        params.polyCount = groundPolys;
        params.nvp = nvp;

        if (hasDetail)
        {
            params.detailMeshes = b.detailMeshes.data();
            params.detailVerts = b.detailVerts.data();
            params.detailVertsCount = static_cast<int>(b.detailVerts.size() / 3);
            params.detailTris = b.detailTris.data();
            params.detailTriCount = static_cast<int>(b.detailTris.size() / 4);
        }

        params.offMeshConCount = static_cast<int>(b.offRadius.size());

        if (params.offMeshConCount > 0)
        {
            params.offMeshConVerts = b.offVerts.data();
            params.offMeshConRad = b.offRadius.data();
            params.offMeshConFlags = b.offFlags.data();
            params.offMeshConAreas = b.offAreas.data();
            params.offMeshConDir = b.offDir.data();
            params.offMeshConUserID = b.offUserId.data();
        }

        params.userId = h.userId;
        params.tileX = h.x;
        params.tileY = h.y;
        params.tileLayer = h.layer;
        dtVcopy(params.bmin, bmin);
        dtVcopy(params.bmax, h.bmax);
        params.bmax[1] = std::max(params.bmax[1], maxY);
        params.walkableHeight = h.walkableHeight;
        params.walkableRadius = h.walkableRadius;
        params.walkableClimb = h.walkableClimb;
        params.cs = cs;
        params.ch = ch;
        params.buildBvTree = h.bvNodeCount > 0;
        return true;
    }
};
