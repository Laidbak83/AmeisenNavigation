#include "TestFramework.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <format>
#include <map>
#include <mutex>

#include "AmeisenNavigation.hpp"
#include "TestWorld.hpp"

// Obstacles a bot has to get past or around, built with the production Recast settings (cell size ~0.21 yards):
// fences of several heights, stairs, a ledge, narrow passages and low ceilings. Each scenario sits in its own lane
// fenced off by high walls (RD coordinates, ground at y = 0, ADT (32,32) covers x/z [-533, 0]).
namespace {
using TestWorld::Wow;

constexpr int MAP_ID = 3;
constexpr float LANE_WIDTH = 12.0f;
constexpr float LANE_X0 = -200.0f; // lanes run along x
constexpr float LANE_X1 = -20.0f;
constexpr float OBSTACLE_X = -110.0f;

/// Closed box, double sided enough for Recast (outward winding).
void AddBox(Structure& s, Vector3 min, Vector3 max, TriAreaId area)
{
    const int base = static_cast<int>(s.verts.size());

    for (int k = 0; k < 8; ++k)
    {
        s.verts.push_back({(k & 1) ? max.x : min.x, (k & 2) ? max.y : min.y, (k & 4) ? max.z : min.z});
    }

    // Faces: -y, +y, -x, +x, -z, +z (two triangles each, up-facing top for the walkable test).
    const int faces[12][3] = {{0, 1, 4}, {1, 5, 4}, {2, 6, 3}, {3, 6, 7}, {0, 4, 2}, {2, 4, 6},
                              {1, 3, 5}, {3, 7, 5}, {0, 2, 1}, {1, 2, 3}, {4, 5, 6}, {5, 7, 6}};

    for (const auto& f : faces)
    {
        s.AddTri(Tri{base + f[0], base + f[1], base + f[2]}, area);
    }
}

struct Lane
{
    std::string name;
    float z0;
    float z1;
    float center() const { return (z0 + z1) * 0.5f; }
};

struct Scene
{
    std::filesystem::path dir;
    std::map<std::string, Lane> lanes;
    double buildSeconds = 0.0;
    int polys = 0;
    int verts = 0;
};

/// Build the scene once per climb setting.
const Scene& Build(float agentClimb)
{
    static std::mutex mutex;
    static std::map<float, Scene> scenes;
    std::lock_guard lock(mutex);

    if (const auto it = scenes.find(agentClimb); it != scenes.end())
    {
        return it->second;
    }

    Scene& scene = scenes[agentClimb];
    scene.dir = std::filesystem::temp_directory_path() / "anav_tests" / std::format("obstacles_{:.2f}", agentClimb);
    std::filesystem::remove_all(scene.dir);
    std::filesystem::create_directories(scene.dir);

    Structure s;
    float z = -20.0f;

    const auto addLane = [&](const std::string& name) {
        const Lane lane{name, z - LANE_WIDTH, z};
        scene.lanes[name] = lane;
        // 5 yard walls on both sides (shared between neighbours).
        AddBox(s, {LANE_X0, 0.0f, lane.z0 - 0.5f}, {LANE_X1, 5.0f, lane.z0}, WMO);
        AddBox(s, {LANE_X0, 0.0f, lane.z1}, {LANE_X1, 5.0f, lane.z1 + 0.5f}, WMO);
        z -= LANE_WIDTH + 0.5f;
        return lane;
    };

    // Fences across the lane: thin boxes (0.15 yards) like the collision of fence models.
    for (const float h : {0.5f, 0.9f, 1.1f, 1.3f, 1.6f, 2.2f})
    {
        const Lane lane = addLane(std::format("fence {:.1f}", h));
        AddBox(s, {OBSTACLE_X, 0.0f, lane.z0}, {OBSTACLE_X + 0.15f, h, lane.z1}, DOODAD);
    }

    // Stairs: 0.45 yard risers, 0.9 yard treads, up to a platform at 4.5 yards.
    {
        const Lane lane = addLane("stairs");

        for (int step = 1; step <= 10; ++step)
        {
            const float x = OBSTACLE_X + static_cast<float>(step - 1) * 0.9f;
            AddBox(s, {x, 0.0f, lane.z0}, {LANE_X1, static_cast<float>(step) * 0.45f, lane.z1}, WMO);
        }
    }

    // A ledge: a 3 yard high block, no way up.
    {
        const Lane lane = addLane("ledge");
        AddBox(s, {OBSTACLE_X, 0.0f, lane.z0}, {LANE_X1, 3.0f, lane.z1}, WMO);
    }

    // Narrow passages: a wall across the lane with a gap.
    for (const float gap : {1.0f, 1.8f})
    {
        const Lane lane = addLane(std::format("gap {:.1f}", gap));
        const float c = lane.center();
        AddBox(s, {OBSTACLE_X, 0.0f, lane.z0}, {OBSTACLE_X + 0.5f, 5.0f, c - gap * 0.5f}, WMO);
        AddBox(s, {OBSTACLE_X, 0.0f, c + gap * 0.5f}, {OBSTACLE_X + 0.5f, 5.0f, lane.z1}, WMO);
    }

    // Low ceilings: a slab over the whole lane width for 10 yards.
    for (const float ceiling : {1.6f, 2.4f})
    {
        const Lane lane = addLane(std::format("ceiling {:.1f}", ceiling));
        AddBox(s, {OBSTACLE_X, ceiling, lane.z0}, {OBSTACLE_X + 10.0f, ceiling + 0.3f, lane.z1}, WMO);
    }

    // Ground under everything.
    TestWorld::AddQuad(s, LANE_X0, z, LANE_X1, -20.0f, 0.0f, TERRAIN_GROUND); // lanes are closed at both ends

    dtNavMeshParams params{};
    params.orig[0] = (31 - 32) * TILESIZE;
    params.orig[2] = (31 - 32) * TILESIZE;
    params.tileWidth = TILESIZE;
    params.tileHeight = TILESIZE;
    params.maxTiles = 64 * 64;
    params.maxPolys = 1 << 20;

    TileBuildConfig config; // production settings
    config.agentClimb = agentClimb;

    WaterMap water;
    RoadMap roads;
    water.BuildSpatialIndex();
    roads.BuildSpatialIndex();

    const auto start = std::chrono::steady_clock::now();
    Anp::AnpWriter writer(MAP_ID, params);
    AdtTileProcessor processor(&writer, scene.dir, "Obstacles", config);
    processor.Process(&s, {{0, 0, 32, 32}}, &water, &roads);
    scene.buildSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    REQUIRE(processor.GetStats().built.load() == 1);
    REQUIRE(writer.Save(scene.dir));

    const auto loaded = Anp::Load(scene.dir / Anp::FileName(MAP_ID));
    REQUIRE(loaded.navMesh);

    for (int i = 0; i < loaded.navMesh->getMaxTiles(); ++i)
    {
        if (const dtMeshTile* tile = loaded.navMesh->getTile(i); tile && tile->header)
        {
            scene.polys += tile->header->polyCount;
            scene.verts += tile->header->vertCount;
        }
    }

    std::printf("obstacle scene (climb %.2f): %d polys, %d verts, built in %.2fs\n", agentClimb, scene.polys,
                scene.verts, scene.buildSeconds);
    return scene;
}

/// Complete path along a lane from before to after the obstacle?
bool Passable(AmeisenNavigation& nav, const Lane& lane, float endY = 0.0f, float endX = LANE_X1 - 5.0f)
{
    Path path(512);
    bool partial = true;
    return nav.GetPath(1, MAP_ID, Wow(LANE_X0 + 5.0f, 0.0f, lane.center()), Wow(endX, endY, lane.center()), path,
                       &partial)
           && !partial;
}

AmeisenNavigation MakeNav(const Scene& scene)
{
    AmeisenNavigationSettings settings;
    settings.meshFolder = scene.dir;
    settings.useAnp = true;
    settings.maxPointPath = 512;
    return AmeisenNavigation(settings);
}
} // namespace

TEST_CASE(Obstacles_FencesTallerThanAStepBlock)
{
    const Scene& scene = Build(TileBuildConfig{}.agentClimb);
    AmeisenNavigation nav = MakeNav(scene);
    REQUIRE(nav.NewClient(1));

    // Low rails are stepped over, anything from 1.1 yards on needs a jump: no path over it.
    CHECK(Passable(nav, scene.lanes.at("fence 0.5")));
    CHECK(Passable(nav, scene.lanes.at("fence 0.9")));
    CHECK(!Passable(nav, scene.lanes.at("fence 1.1")));
    CHECK(!Passable(nav, scene.lanes.at("fence 1.3")));
    CHECK(!Passable(nav, scene.lanes.at("fence 1.6")));
    CHECK(!Passable(nav, scene.lanes.at("fence 2.2")));
}

TEST_CASE(Obstacles_StairsLedgesGapsAndCeilings)
{
    const Scene& scene = Build(TileBuildConfig{}.agentClimb);
    AmeisenNavigation nav = MakeNav(scene);
    REQUIRE(nav.NewClient(1));

    // Up the stairs onto the platform (4.5 yards), not onto the 3 yard ledge.
    CHECK(Passable(nav, scene.lanes.at("stairs"), 4.5f, -40.0f));
    CHECK(!Passable(nav, scene.lanes.at("ledge"), 3.0f, -40.0f));

    // The agent (radius 0.6) doesn't fit through 1 yard, does through 1.8.
    CHECK(!Passable(nav, scene.lanes.at("gap 1.0")));
    CHECK(Passable(nav, scene.lanes.at("gap 1.8")));

    // Agent height 2: not under 1.6, under 2.4.
    CHECK(!Passable(nav, scene.lanes.at("ceiling 1.6")));
    CHECK(Passable(nav, scene.lanes.at("ceiling 2.4")));
}

TEST_CASE(Obstacles_OldClimbWalkedOverLowFences)
{
    // The historic 1.2 yard climb let paths cross 1.1 yard fences (bots ran into them).
    const Scene& scene = Build(1.2f);
    AmeisenNavigation nav = MakeNav(scene);
    REQUIRE(nav.NewClient(1));
    CHECK(Passable(nav, scene.lanes.at("fence 1.1")));
    CHECK(!Passable(nav, scene.lanes.at("fence 1.3")));
}
