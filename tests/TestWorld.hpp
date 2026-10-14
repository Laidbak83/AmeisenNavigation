#pragma once

#include <filesystem>
#include <mutex>

#include "Processors/AdtTileProcessor.hpp"
#include "Processors/WorldBuilder.hpp"
#include "Utils/Structure.hpp"

/// Synthetic two-ADT world exported through the real exporter pipeline (AdtTileProcessor + AnpWriter).
///
/// Layout (RD coordinates, x/z horizontal, y up). ADT (33,32) covers x [-1066.7, -533.3], ADT (32,32) covers
/// x [-533.3, 0], both cover z [-533.3, 0]:
///   - flat ground at y = 0 over both ADTs
///   - a 10 yard high wall at x = -300 from z = -520 to z = -100 (paths must go around it near z > -100)
///   - a platform ("bridge deck") at y = 5 over x [-450, -400], z [-450, -400]
///   - a water pool (surface y = 0.5) over x [-120, -60], z [-460, -400]
///   - a road over x [-200, -150], z [-300, -250]
///   - an Alliance faction chunk over x [-100, -40], z [-100, -40]
///   - a 1 x 1 yard, 6 yard high pillar (a doodad) over x [-900, -899], z [-450, -449]
/// The static world (001.anw) has the walls as WMO 77, the platform as WMO 78, the pillar as doodad 99, the chunk
/// area 12 everywhere except area 1519 in the chunk at RD x [-33.3, 0], z [-33.3, 0].
namespace TestWorld {
constexpr int MAP_ID = 1;
constexpr float WALL_X = -300.0f;
constexpr float WALL_Z_MIN = -520.0f;
constexpr float WALL_Z_MAX = -100.0f;
constexpr float PLATFORM_Y = 5.0f;
constexpr float PILLAR_X = -900.0f;
constexpr float PILLAR_Z = -450.0f;
constexpr uint32_t WALL_SOURCE = (1u << 28) | 77; // AnpWorld::MakeSource(WMO, 77)
constexpr uint32_t PLATFORM_SOURCE = (1u << 28) | 78;
constexpr uint32_t PILLAR_SOURCE = (2u << 28) | 99; // doodad
/// gameobjects.ano: display 777 is a 120 yard long, 4 yard high double wall along the model's x axis (a gate).
constexpr uint32_t GATE_DISPLAY = 777;
constexpr float GATE_HALF_LENGTH = 60.0f;

/// RD -> WoW
inline Vector3 Wow(float rdX, float rdY, float rdZ) { return Vector3(rdZ, rdX, rdY); }

inline void AddQuad(Structure& s, float x0, float z0, float x1, float z1, float y, TriAreaId area)
{
    const int base = static_cast<int>(s.verts.size());
    s.verts.push_back({x0, y, z0});
    s.verts.push_back({x0, y, z1});
    s.verts.push_back({x1, y, z0});
    s.verts.push_back({x1, y, z1});
    // Winding with an up facing normal (see rcClearUnwalkableTriangles).
    s.AddTri(Tri{base + 0, base + 1, base + 2}, area);
    s.AddTri(Tri{base + 2, base + 1, base + 3}, area);
}

/// Vertical double sided quad in the x = const plane.
inline void AddWallX(Structure& s, float x, float z0, float z1, float y0, float y1)
{
    const int base = static_cast<int>(s.verts.size());
    s.verts.push_back({x, y0, z0});
    s.verts.push_back({x, y1, z0});
    s.verts.push_back({x, y0, z1});
    s.verts.push_back({x, y1, z1});
    s.AddTri(Tri{base + 0, base + 1, base + 2}, WMO);
    s.AddTri(Tri{base + 2, base + 1, base + 3}, WMO);
    s.AddTri(Tri{base + 2, base + 1, base + 0}, WMO);
    s.AddTri(Tri{base + 3, base + 1, base + 2}, WMO);
}

inline TileBuildConfig FastBuildConfig()
{
    // Coarser than production (cs ~1.04 instead of ~0.21) so the test runs in seconds.
    TileBuildConfig config;
    config.meshResolution = 512;
    config.subTileSize = 64;
    config.minRegionSize = 2;
    config.mergeRegionSize = 8;
    config.links.jumps = false; // the server tests add their own off-mesh links
    config.links.drops = false;
    return config;
}

struct Result
{
    std::filesystem::path meshDir;
    int tilesBuilt = 0;
    int tilesFailed = 0;
};

/// Build the world once per process and return the directory containing "001.anp".
inline const Result& Get()
{
    static std::once_flag once;
    static Result result;

    std::call_once(once, [] {
        result.meshDir = std::filesystem::temp_directory_path() / "anav_tests" / "world";
        std::filesystem::remove_all(result.meshDir);
        std::filesystem::create_directories(result.meshDir);

        Structure geometry;
        AddQuad(geometry, -2 * TILESIZE, -TILESIZE, 0.0f, 0.0f, 0.0f, TERRAIN_GROUND);
        size_t first = geometry.tris.size();
        AddWallX(geometry, WALL_X, WALL_Z_MIN, WALL_Z_MAX, -1.0f, 10.0f);
        AddWallX(geometry, WALL_X + 1.0f, WALL_Z_MIN, WALL_Z_MAX, -1.0f, 10.0f);
        geometry.TagSources(first, WALL_SOURCE);
        first = geometry.tris.size();
        AddQuad(geometry, -450.0f, -450.0f, -400.0f, -400.0f, PLATFORM_Y, WMO);
        geometry.TagSources(first, PLATFORM_SOURCE);
        AddQuad(geometry, -120.0f, -460.0f, -60.0f, -400.0f, 0.5f, LIQUID_WATER);
        first = geometry.tris.size();
        AddWallX(geometry, PILLAR_X, PILLAR_Z, PILLAR_Z + 1.0f, -0.5f, 6.0f);
        AddWallX(geometry, PILLAR_X + 1.0f, PILLAR_Z, PILLAR_Z + 1.0f, -0.5f, 6.0f);
        AddQuad(geometry, PILLAR_X, PILLAR_Z, PILLAR_X + 1.0f, PILLAR_Z + 1.0f, 6.0f, DOODAD);
        geometry.TagSources(first, PILLAR_SOURCE);

        WaterMap water;
        RoadMap roads;
        CityMap cities;
        FactionMap factions;

        // Rects are added in WoW coordinates (NW / SE corners): WoW x = RD z, WoW y = RD x.
        water.AddRect({-400.0f, -60.0f, 0.0f}, {-460.0f, -120.0f, 0.0f}, 0.5f, 0.5f, 0.5f, 0.5f, LIQUID_WATER);
        roads.AddRect({-250.0f, -150.0f, 0.0f}, {-300.0f, -200.0f, 0.0f});
        factions.AddRect({-40.0f, -40.0f, 0.0f}, {-100.0f, -100.0f, 0.0f}, 1);

        water.BuildSpatialIndex();
        roads.BuildSpatialIndex();
        cities.BuildSpatialIndex();
        factions.BuildSpatialIndex();

        // Same grid setup as the exporter: ADT (32,32) and (33,32), maxAdtX = 33, maxAdtY = 32.
        dtNavMeshParams params{};
        params.orig[0] = (31 - 33) * TILESIZE;
        params.orig[2] = (31 - 32) * TILESIZE;
        params.tileWidth = TILESIZE;
        params.tileHeight = TILESIZE;
        params.maxTiles = 64 * 64;
        params.maxPolys = 1 << 20;

        const std::vector<TileCoord> tiles{{1, 0, 32, 32}, {0, 0, 33, 32}};

        Anp::AnpWriter writer(MAP_ID, params);
        AdtTileProcessor processor(&writer, result.meshDir, "TestWorld", FastBuildConfig());
        processor.Process(&geometry, tiles, &water, &roads, &factions, &cities);

        result.tilesBuilt = processor.GetStats().built.load();
        result.tilesFailed = processor.GetStats().failed.load();
        writer.Save(result.meshDir);

        ChunkAreaMap chunkAreas;

        for (int r = 0; r < 16; ++r)
        {
            for (int c = 0; c < 16; ++c)
            {
                chunkAreas.Set(32, 32, r, c, r == 0 && c == 0 ? 1519 : 12);
                chunkAreas.Set(33, 32, r, c, 12);
            }
        }

        const TileBuildConfig config = FastBuildConfig();
        WriteWorld(geometry, {{32, 32}, {33, 32}}, chunkAreas, MAP_ID, result.meshDir, nullptr, &config, &params, &water,
                   &roads, &cities, &factions);

        // Model space in RD order (RD x = model y, RD z = model x): walls in the RD x = 0 plane along RD z.
        Structure gate;
        AddWallX(gate, -0.2f, -GATE_HALF_LENGTH, GATE_HALF_LENGTH, -0.5f, 4.0f);
        AddWallX(gate, 0.2f, -GATE_HALF_LENGTH, GATE_HALF_LENGTH, -0.5f, 4.0f);
        ObjectModels::Write(result.meshDir, {{GATE_DISPLAY, 0u}}, {ObjectModels::ToTile(gate)});
    });

    return result;
}
} // namespace TestWorld
