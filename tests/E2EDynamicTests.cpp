#include "TestFramework.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "AmeisenNavigation.hpp"
#include "TestWorld.hpp"
#include "World/DynamicWorld.hpp"

// Game objects at runtime: a gate (display 777) closes the passage north of the test world's wall, the navmesh tiles
// around it are rebuilt and paths go around the south end; opened or removed, they go north again.
namespace {
using TestWorld::Wow;

constexpr int MAP = TestWorld::MAP_ID;
constexpr uint64_t GATE_GUID = 0xF1100000000001ull;

AmeisenNavigationSettings Settings()
{
    AmeisenNavigationSettings settings;
    settings.meshFolder = TestWorld::Get().meshDir;
    settings.useAnp = true;
    settings.maxPointPath = 256;
    return settings;
}

/// RD z of the point where the path crosses the wall's plane (x = -300), NAN if it doesn't.
float CrossingZ(AmeisenNavigation& nav)
{
    Path path(256);
    bool partial = false;
    REQUIRE(nav.GetPath(1, MAP, Wow(-700.0f, 0.0f, -300.0f), Wow(-200.0f, 0.0f, -300.0f), path, &partial));
    REQUIRE(!partial);

    for (int i = 1; i < path.pointCount; ++i)
    {
        // WoW y = RD x.
        const float a = path[i - 1].y + 300.0f;
        const float b = path[i].y + 300.0f;

        if ((a < 0.0f) != (b < 0.0f))
        {
            const float t = a / (a - b);
            return path[i - 1].x + (path[i].x - path[i - 1].x) * t; // WoW x = RD z
        }
    }

    return NAN;
}

WorldObject Gate(bool blocking)
{
    WorldObject gate;
    gate.guid = GATE_GUID;
    gate.displayId = TestWorld::GATE_DISPLAY;
    gate.position = Wow(TestWorld::WALL_X, 0.0f, -50.0f); // from the wall's end to past the map's edge
    gate.blocking = blocking;
    return gate;
}
} // namespace

TEST_CASE(Dynamic_GateChangesTheNavmesh)
{
    AmeisenNavigation nav(Settings());
    REQUIRE(nav.NewClient(1));
    DynamicWorld dynamic(nav, TestWorld::Get().meshDir);

    // Open passage: paths cross north of the wall's end (RD z -100).
    CHECK(CrossingZ(nav) > -100.0f);

    // Closed gate: the line of sight is blocked at once, the path after the rebuild goes around the south end.
    const WorldObject closed = Gate(true);
    REQUIRE(dynamic.SetObjects(MAP, std::span(&closed, 1), {}) == 1);
    CHECK_EQ(dynamic.ObjectCount(MAP), size_t{1});

    World::Hit hit;
    REQUIRE(nav.LineOfSight(MAP, Wow(-310.0f, 1.5f, -50.0f), Wow(-290.0f, 1.5f, -50.0f), 0, hit));
    CHECK(hit.hit);
    CHECK_EQ(hit.source, AnpWorld::MakeSource(AnpWorld::SourceKind::GAME_OBJECT, TestWorld::GATE_DISPLAY));
    REQUIRE(
        nav.LineOfSight(MAP, Wow(-310.0f, 1.5f, -50.0f), Wow(-290.0f, 1.5f, -50.0f), World::IGNORE_GAME_OBJECTS, hit));
    CHECK(!hit.hit);

    REQUIRE(dynamic.WaitIdle(std::chrono::seconds(60)));
    CHECK(CrossingZ(nav) < -520.0f);

    // The same state again changes nothing.
    REQUIRE(dynamic.SetObjects(MAP, std::span(&closed, 1), {}) == 1);
    CHECK_EQ(dynamic.PendingTiles(), size_t{0});

    // Opened: kept, but out of the way.
    const WorldObject open = Gate(false);
    REQUIRE(dynamic.SetObjects(MAP, std::span(&open, 1), {}) == 1);
    REQUIRE(nav.LineOfSight(MAP, Wow(-310.0f, 1.5f, -50.0f), Wow(-290.0f, 1.5f, -50.0f), 0, hit));
    CHECK(!hit.hit);
    REQUIRE(dynamic.WaitIdle(std::chrono::seconds(60)));
    CHECK(CrossingZ(nav) > -100.0f);

    // Closed and removed.
    REQUIRE(dynamic.SetObjects(MAP, std::span(&closed, 1), {}) == 1);
    REQUIRE(dynamic.WaitIdle(std::chrono::seconds(60)));
    CHECK(CrossingZ(nav) < -520.0f);
    const uint64_t guid = GATE_GUID;
    REQUIRE(dynamic.SetObjects(MAP, {}, std::span(&guid, 1)) == 0);
    CHECK_EQ(dynamic.ObjectCount(MAP), size_t{0});
    REQUIRE(dynamic.WaitIdle(std::chrono::seconds(60)));
    CHECK(CrossingZ(nav) > -100.0f);
}

TEST_CASE(Dynamic_UnknownModelsAndMaps)
{
    AmeisenNavigation nav(Settings());
    DynamicWorld dynamic(nav, TestWorld::Get().meshDir);

    WorldObject unknown = Gate(true);
    unknown.displayId = 12345;
    CHECK_EQ(dynamic.SetObjects(MAP, std::span(&unknown, 1), {}), 0);

    WorldObject broken = Gate(true);
    broken.scale = NAN;
    CHECK_EQ(dynamic.SetObjects(MAP, std::span(&broken, 1), {}), 0);

    const WorldObject gate = Gate(true);
    CHECK_EQ(dynamic.SetObjects(4242, std::span(&gate, 1), {}), -1); // no static world
    CHECK_EQ(dynamic.PendingTiles(), size_t{0});
}

TEST_CASE(Dynamic_RequestsWhileTilesAreReplaced)
{
    AmeisenNavigation nav(Settings());
    REQUIRE(nav.NewClient(1));
    DynamicWorld dynamic(nav, TestWorld::Get().meshDir);
    std::atomic<bool> stop{false};
    std::atomic<int> paths{0};
    std::atomic<int> failures{0};
    std::vector<std::thread> threads;

    for (size_t c = 0; c < 4; ++c)
    {
        REQUIRE(nav.NewClient(10 + c));
        threads.emplace_back([&, c] {
            Path path(256);
            World::Hit hit;

            while (!stop)
            {
                // Every request is answered with a path around one end of the wall or the other, never garbage.
                if (nav.GetPath(10 + c, MAP, Wow(-700.0f, 0.0f, -300.0f), Wow(-200.0f, 0.0f, -300.0f), path) &&
                    path.pointCount >= 2)
                {
                    paths++;
                }
                else
                {
                    failures++;
                }

                nav.LineOfSight(MAP, Wow(-310.0f, 1.5f, -50.0f), Wow(-290.0f, 1.5f, -50.0f), 0, hit);
            }
        });
    }

    for (int i = 0; i < 6; ++i)
    {
        const WorldObject gate = Gate(i % 2 == 0);
        REQUIRE(dynamic.SetObjects(MAP, std::span(&gate, 1), {}) == 1);
        REQUIRE(dynamic.WaitIdle(std::chrono::seconds(60)));
    }

    stop = true;

    for (auto& thread : threads)
    {
        thread.join();
    }

    CHECK(paths > 0);
    CHECK_EQ(failures.load(), 0);
    CHECK(CrossingZ(nav) > -100.0f); // last state: open
}

namespace {
/// Box (RD bounds) as a dynamic mesh: 12 triangles.
std::shared_ptr<const World::DynamicMesh> BoxMesh(Vector3 min, Vector3 max, uint32_t displayId)
{
    auto mesh = std::make_shared<World::DynamicMesh>();
    mesh->source = AnpWorld::MakeSource(AnpWorld::SourceKind::GAME_OBJECT, displayId);

    for (int k = 0; k < 8; ++k)
    {
        mesh->verts.insert(mesh->verts.end(),
                           {(k & 1) ? max.x : min.x, (k & 2) ? max.y : min.y, (k & 4) ? max.z : min.z});
    }

    mesh->indices = {0, 1, 3, 0, 3, 2, 4, 6, 7, 4, 7, 5, 0, 4, 5, 0, 5, 1,
                     2, 3, 7, 2, 7, 6, 0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3};
    mesh->areas.assign(12, DOODAD);
    std::copy(min.pos, min.pos + 3, mesh->bmin);
    std::copy(max.pos, max.pos + 3, mesh->bmax);
    return mesh;
}

int CountFlags(const unsigned char* flags, int count, unsigned char flag)
{
    return static_cast<int>(std::count_if(flags, flags + count, [&](unsigned char f) { return (f & flag) != 0; }));
}
} // namespace

TEST_CASE(ClearBody_GoesAroundWhatTheNavmeshDoesntKnow)
{
    AmeisenNavigation nav(Settings());
    REQUIRE(nav.NewClient(1));
    PathOptions options;
    options.clearBody = true;
    Path path(256);
    unsigned char flags[256]{};

    // The navmesh keeps its radius away from walls: nothing to do around the test world's wall.
    REQUIRE(
        nav.GetPath(1, MAP, Wow(-700.0f, 0.0f, -300.0f), Wow(-200.0f, 0.0f, -300.0f), path, nullptr, &options, flags));
    CHECK_EQ(CountFlags(flags, path.pointCount, PATH_POINT_BYPASS | PATH_POINT_BLOCKED), 0);

    // A post the navmesh doesn't know about (an object whose tile isn't rebuilt yet) on a straight path.
    auto meshes = std::make_shared<World::DynamicMeshes>();
    meshes->push_back(BoxMesh({-550.5f, -0.5f, -50.5f}, {-549.5f, 3.0f, -49.5f}, 900));
    nav.GetWorld(MAP)->SetDynamicMeshes(meshes);

    const Vector3 a = Wow(-700.0f, 0.0f, -50.0f);
    const Vector3 b = Wow(-400.0f, 0.0f, -50.0f);
    REQUIRE(nav.GetPath(1, MAP, a, b, path, nullptr, nullptr, flags));
    CHECK_EQ(path.pointCount, 2); // plain path: straight through it

    REQUIRE(nav.GetPath(1, MAP, a, b, path, nullptr, &options, flags));
    CHECK_EQ(CountFlags(flags, path.pointCount, PATH_POINT_BYPASS), 1);
    CHECK_EQ(CountFlags(flags, path.pointCount, PATH_POINT_BLOCKED), 0);

    for (int i = 1; i < path.pointCount; ++i)
    {
        World::Hit hit;
        REQUIRE(nav.SweepBody(MAP, path[i - 1], path[i], options.bodyRadius, options.bodyHeight, options.bodyStepHeight,
                              0, hit));
        CHECK(!hit.hit);
    }

    // A long wall (a gate whose tile isn't rebuilt): no way around it nearby, the bot has to know.
    meshes = std::make_shared<World::DynamicMeshes>();
    meshes->push_back(BoxMesh({-300.2f, -0.5f, -110.0f}, {-299.8f, 4.0f, 10.0f}, 777));
    nav.GetWorld(MAP)->SetDynamicMeshes(meshes);
    REQUIRE(
        nav.GetPath(1, MAP, Wow(-320.0f, 0.0f, -50.0f), Wow(-280.0f, 0.0f, -50.0f), path, nullptr, &options, flags));
    CHECK_EQ(CountFlags(flags, path.pointCount, PATH_POINT_BLOCKED), 1);

    // Low enough to step over: nothing.
    meshes = std::make_shared<World::DynamicMeshes>();
    meshes->push_back(BoxMesh({-550.5f, -0.5f, -50.5f}, {-549.5f, 0.5f, -49.5f}, 901));
    nav.GetWorld(MAP)->SetDynamicMeshes(meshes);
    REQUIRE(nav.GetPath(1, MAP, a, b, path, nullptr, &options, flags));
    CHECK_EQ(path.pointCount, 2);
    CHECK_EQ(CountFlags(flags, path.pointCount, PATH_POINT_BYPASS | PATH_POINT_BLOCKED), 0);
    nav.GetWorld(MAP)->SetDynamicMeshes(nullptr);
}
