#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <DetourNavMesh.h>

#include "AnpFormat.hpp"

/// Writes a navmesh as a binary PLY file (MeshLab, Blender, CloudCompare open it): the detail triangles in WoW
/// coordinates (x north, y west, z up) with a color per triangle, and off-mesh connections as narrow vertical ribbons
/// (jump yellow, drop magenta, other links white).
namespace NavMeshPly {
enum class ColorMode
{
    SLOPE, // blue (flat) -> green -> yellow -> red (60 degrees and more)
    AREA,  // by area: ground, road, city, structures, liquids, faction tint
};

struct Options
{
    ColorMode color = ColorMode::SLOPE;
    bool box = false;      // only triangles with a vertex inside the box (WoW x/y)
    float boxMin[2]{};
    float boxMax[2]{};
};

struct Stats
{
    size_t triangles = 0;
    size_t offMeshConnections = 0;
};

struct Rgb
{
    unsigned char r, g, b;
};

/// Slope gradient like a terrain analysis: 0 blue, 15 cyan, 30 green, 45 yellow, 60+ red.
inline Rgb SlopeColor(float degrees) noexcept
{
    static constexpr Rgb stops[]{{40, 70, 220}, {40, 200, 230}, {60, 200, 80}, {240, 220, 50}, {220, 50, 40}};
    const float t = std::clamp(degrees / 15.0f, 0.0f, 4.0f);
    const int i = std::min(static_cast<int>(t), 3);
    const float f = t - static_cast<float>(i);
    const auto mix = [f](unsigned char a, unsigned char b) {
        return static_cast<unsigned char>(static_cast<float>(a) + (static_cast<float>(b) - static_cast<float>(a)) * f);
    };
    return {mix(stops[i].r, stops[i + 1].r), mix(stops[i].g, stops[i + 1].g), mix(stops[i].b, stops[i + 1].b)};
}

inline Rgb AreaColor(unsigned char area) noexcept
{
    Rgb c{150, 150, 150};

    switch (GetNeutralArea(area))
    {
        case TERRAIN_GROUND: c = {110, 170, 80}; break;
        case TERRAIN_ROAD: c = {170, 130, 80}; break;
        case TERRAIN_CITY: c = {200, 200, 200}; break;
        case WMO: c = {150, 120, 200}; break;
        case DOODAD: c = {200, 150, 100}; break;
        case LIQUID_WATER: c = {60, 120, 230}; break;
        case LIQUID_OCEAN: c = {30, 70, 180}; break;
        case LIQUID_LAVA: c = {240, 80, 20}; break;
        case LIQUID_SLIME: c = {130, 220, 40}; break;
        default:
            // TrinityCore MMAP areas: 11 ground, 10 steep, 9 water, 8 magma/slime.
            if (area == 11)
                c = {110, 170, 80};
            else if (area == 10)
                c = {200, 160, 60};
            else if (area == 9)
                c = {60, 120, 230};
            else if (area == 8)
                c = {240, 80, 20};
            break;
    }

    switch (GetAreaFaction(area))
    {
        case AreaFaction::Alliance: c.b = static_cast<unsigned char>(std::min(255, c.b + 60)); break;
        case AreaFaction::Horde: c.r = static_cast<unsigned char>(std::min(255, c.r + 60)); break;
        default: break;
    }

    return c;
}

/// Write the navmesh. Returns false if the file couldn't be written.
inline bool Write(const dtNavMesh& mesh, const std::string& path, const Options& options = {}, Stats* stats = nullptr)
{
    struct Vertex
    {
        float x, y, z; // WoW coordinates
        unsigned char r, g, b;
    };

    std::vector<Vertex> vertices;
    const auto add = [&](const float* rd, Rgb color) { vertices.push_back({rd[2], rd[0], rd[1], color.r, color.g, color.b}); };
    const auto inBox = [&](const float* rd) {
        return !options.box || (rd[2] >= options.boxMin[0] && rd[2] <= options.boxMax[0] && rd[0] >= options.boxMin[1]
                                && rd[0] <= options.boxMax[1]);
    };

    Stats local;

    for (int t = 0; t < mesh.getMaxTiles(); ++t)
    {
        const dtMeshTile* tile = mesh.getTile(t);

        if (!tile || !tile->header)
        {
            continue;
        }

        for (int p = 0; p < tile->header->polyCount; ++p)
        {
            const dtPoly& poly = tile->polys[p];

            if (poly.getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
            {
                const float* a = &tile->verts[poly.verts[0] * 3];
                const float* b = &tile->verts[poly.verts[1] * 3];

                if (!inBox(a) && !inBox(b))
                {
                    continue;
                }

                const unsigned char area = poly.getArea();
                const Rgb color = area == AREA_OFFMESH_JUMP   ? Rgb{250, 230, 40}
                                  : area == AREA_OFFMESH_DROP ? Rgb{230, 40, 230}
                                                              : Rgb{250, 250, 250};

                // A ribbon from a to b, 0.4 yards high, slightly arched in the middle (two triangles per half).
                const float mid[3]{(a[0] + b[0]) * 0.5f, std::max(a[1], b[1]) + 0.8f, (a[2] + b[2]) * 0.5f};
                const float* points[3]{a, mid, b};

                for (int s = 0; s < 2; ++s)
                {
                    const float* p0 = points[s];
                    const float* p1 = points[s + 1];
                    const float p0Up[3]{p0[0], p0[1] + 0.4f, p0[2]};
                    const float p1Up[3]{p1[0], p1[1] + 0.4f, p1[2]};
                    add(p0, color);
                    add(p1, color);
                    add(p1Up, color);
                    add(p0, color);
                    add(p1Up, color);
                    add(p0Up, color);
                }

                local.offMeshConnections++;
                continue;
            }

            if (!tile->detailMeshes || p >= tile->header->detailMeshCount)
            {
                continue;
            }

            const dtPolyDetail& pd = tile->detailMeshes[p];

            for (int k = 0; k < pd.triCount; ++k)
            {
                const unsigned char* tri = &tile->detailTris[(pd.triBase + k) * 4];
                const float* v[3];

                for (int j = 0; j < 3; ++j)
                {
                    v[j] = tri[j] < poly.vertCount ? &tile->verts[poly.verts[tri[j]] * 3]
                                                   : &tile->detailVerts[(pd.vertBase + (tri[j] - poly.vertCount)) * 3];
                }

                if (!inBox(v[0]) && !inBox(v[1]) && !inBox(v[2]))
                {
                    continue;
                }

                Rgb color;

                if (options.color == ColorMode::SLOPE)
                {
                    const float e0[3]{v[1][0] - v[0][0], v[1][1] - v[0][1], v[1][2] - v[0][2]};
                    const float e1[3]{v[2][0] - v[0][0], v[2][1] - v[0][1], v[2][2] - v[0][2]};
                    const float n[3]{e0[1] * e1[2] - e0[2] * e1[1], e0[2] * e1[0] - e0[0] * e1[2],
                                     e0[0] * e1[1] - e0[1] * e1[0]};
                    const float length = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
                    const float slope = length > 0.0f ? std::acos(std::min(1.0f, std::fabs(n[1]) / length)) : 0.0f;
                    color = SlopeColor(slope * 57.29578f);
                }
                else
                {
                    color = AreaColor(poly.getArea());
                }

                for (int j = 0; j < 3; ++j)
                {
                    add(v[j], color);
                }

                local.triangles++;
            }
        }
    }

    std::ofstream out(path, std::ios::binary | std::ios::trunc);

    if (!out)
    {
        return false;
    }

    const size_t faces = vertices.size() / 3;
    out << "ply\nformat binary_little_endian 1.0\ncomment AmeisenNavigation navmesh, WoW coordinates\n"
        << "element vertex " << vertices.size() << "\nproperty float x\nproperty float y\nproperty float z\n"
        << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        << "element face " << faces << "\nproperty list uchar int vertex_indices\nend_header\n";

    for (const Vertex& v : vertices)
    {
        out.write(reinterpret_cast<const char*>(&v.x), sizeof(float) * 3);
        out.write(reinterpret_cast<const char*>(&v.r), 3);
    }

    for (size_t f = 0; f < faces; ++f)
    {
        const unsigned char count = 3;
        const int32_t indices[3]{static_cast<int32_t>(f * 3), static_cast<int32_t>(f * 3 + 1),
                                 static_cast<int32_t>(f * 3 + 2)};
        out.write(reinterpret_cast<const char*>(&count), 1);
        out.write(reinterpret_cast<const char*>(indices), sizeof(indices));
    }

    if (stats)
    {
        *stats = local;
    }

    return out.good();
}
} // namespace NavMeshPly
