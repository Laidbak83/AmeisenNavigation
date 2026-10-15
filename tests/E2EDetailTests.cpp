#include "TestFramework.hpp"

#include <cmath>
#include <filesystem>
#include <mutex>
#include <vector>

#include "AmeisenNavigation.hpp"
#include "TestWorld.hpp"

// DETAIL: dense waypoints on the navmesh surface. A hill scene (production settings) for the height profile, the test
// world's pool and road for area changes.
namespace {
using TestWorld::Wow;

constexpr int HILL_MAP = 4;
constexpr float HILL_X = -110.0f; // crest (RD x), the hill rises from -120 and falls to -100
constexpr float HILL_TOP = 3.0f;
constexpr float LANE_Z = -40.0f;

/// Sloped quad from (x0, y0) to (x1, y1) along RD x over z [z0, z1], facing up.
void AddSlope(Structure& s, float x0, float y0, float x1, float y1, float z0, float z1)
{
    const int base = static_cast<int>(s.verts.size());
    s.verts.push_back({x0, y0, z0});
    s.verts.push_back({x0, y0, z1});
    s.verts.push_back({x1, y1, z0});
    s.verts.push_back({x1, y1, z1});
    s.AddTri(Tri{base + 0, base + 1, base + 2}, TERRAIN_GROUND);
    s.AddTri(Tri{base + 2, base + 1, base + 3}, TERRAIN_GROUND);
}

const std::filesystem::path& HillScene()
{
    static std::once_flag once;
    static std::filesystem::path dir;

    std::call_once(once, [] {
        dir = std::filesystem::temp_directory_path() / "anav_tests" / "detail";
        std::filesystem::remove_all(dir);
        std::filesystem::create_directories(dir);

        Structure s;
        AddSlope(s, -200.0f, 0.0f, -120.0f, 0.0f, -60.0f, -20.0f);
        AddSlope(s, -120.0f, 0.0f, HILL_X, HILL_TOP, -60.0f, -20.0f);
        AddSlope(s, HILL_X, HILL_TOP, -100.0f, 0.0f, -60.0f, -20.0f);
        AddSlope(s, -100.0f, 0.0f, -20.0f, 0.0f, -60.0f, -20.0f);

        dtNavMeshParams params{};
        params.orig[0] = (31 - 32) * TILESIZE;
        params.orig[2] = (31 - 32) * TILESIZE;
        params.tileWidth = TILESIZE;
        params.tileHeight = TILESIZE;
        params.maxTiles = 64 * 64;
        params.maxPolys = 1 << 20;

        TileBuildConfig config;
        config.links.jumps = config.links.drops = false;
        WaterMap water;
        RoadMap roads;
        water.BuildSpatialIndex();
        roads.BuildSpatialIndex();
        Anp::AnpWriter writer(HILL_MAP, params);
        AdtTileProcessor processor(&writer, dir, "Hill", config);
        processor.Process(&s, {{0, 0, 32, 32}}, &water, &roads);
        REQUIRE(writer.Save(dir));
    });

    return dir;
}

AmeisenNavigation MakeNav(const std::filesystem::path& dir)
{
    AmeisenNavigationSettings settings;
    settings.meshFolder = dir;
    settings.useAnp = true;
    settings.maxPointPath = 512;
    return AmeisenNavigation(settings);
}

/// Height of the hill's surface at RD x.
float HillHeight(float x)
{
    if (x <= -120.0f || x >= -100.0f)
        return 0.0f;
    return x < HILL_X ? HILL_TOP * (x + 120.0f) / 10.0f : HILL_TOP * (-100.0f - x) / 10.0f;
}
} // namespace

TEST_CASE(Detail_FollowsTheHeightProfile)
{
    AmeisenNavigation nav = MakeNav(HillScene());
    REQUIRE(nav.NewClient(1));
    const Vector3 a = Wow(-190.0f, 0.0f, LANE_Z);
    const Vector3 b = Wow(-30.0f, 0.0f, LANE_Z);
    Path path(512);

    // Plain: straight over the hill.
    REQUIRE(nav.GetPath(1, HILL_MAP, a, b, path));
    CHECK_EQ(path.pointCount, 2);

    PathOptions options;
    options.detail = true;
    REQUIRE(nav.GetPath(1, HILL_MAP, a, b, path, nullptr, &options));
    REQUIRE(path.pointCount > 2);
    CHECK(path[0].DistanceTo(a) < 1.0f);
    CHECK(path[path.pointCount - 1].DistanceTo(b) < 1.0f);

    bool crest = false, footUp = false, footDown = false;

    for (int i = 0; i < path.pointCount; ++i)
    {
        const Vector3& p = path[i]; // WoW: y = RD x, z = up
        crest |= std::fabs(p.y - HILL_X) < 0.75f && p.z > HILL_TOP - 0.3f;
        footUp |= std::fabs(p.y + 120.0f) < 0.75f;
        footDown |= std::fabs(p.y + 100.0f) < 0.75f;

        // On the surface (navmesh heights lie a little above it).
        CHECK(p.z >= HillHeight(p.y) - 0.2f && p.z <= HillHeight(p.y) + 0.6f);

        if (i > 0)
        {
            CHECK(path[i - 1].DistanceTo(p) <= options.waypointSpacing + 0.6f); // never further apart than asked
        }
    }

    CHECK(crest);
    CHECK(footUp);
    CHECK(footDown);

    // Coarser spacing: fewer points, the bends stay.
    const int dense = path.pointCount;
    options.waypointSpacing = 20.0f;
    REQUIRE(nav.GetPath(1, HILL_MAP, a, b, path, nullptr, &options));
    CHECK(path.pointCount < dense);
    bool crestKept = false;

    for (int i = 0; i < path.pointCount; ++i)
    {
        crestKept |= std::fabs(path[i].y - HILL_X) < 0.75f;
    }

    CHECK(crestKept);

    // A path buffer too small for the spacing: the spacing grows, the ends stay.
    Path small(12);
    options.waypointSpacing = 2.0f;
    REQUIRE(nav.GetPath(1, HILL_MAP, a, b, small, nullptr, &options));
    CHECK(small.pointCount <= 12 && small.pointCount >= 2);
    CHECK(small[small.pointCount - 1].DistanceTo(b) < 1.0f);
}

TEST_CASE(Detail_MarksAreaChanges)
{
    AmeisenNavigation nav = MakeNav(TestWorld::Get().meshDir);
    REQUIRE(nav.NewClient(1));
    PathOptions options;
    options.detail = true;
    options.waypointSpacing = 50.0f; // only the area changes, no fill
    Path path(256);

    // Across the pool (RD x -120 .. -60, water surface 0.5): points on both sides of both shores.
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, Wow(-150.0f, 0.0f, -430.0f), Wow(-30.0f, 0.0f, -430.0f), path, nullptr,
                        &options));
    int nearShore = 0;

    for (int i = 0; i < path.pointCount; ++i)
    {
        const float x = path[i].y;
        nearShore += std::fabs(x + 120.0f) < 2.0f || std::fabs(x + 60.0f) < 2.0f;
    }

    CHECK(nearShore >= 4);

    // Across the road (RD x -200 .. -150).
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, Wow(-260.0f, 0.0f, -275.0f), Wow(-90.0f, 0.0f, -275.0f), path, nullptr,
                        &options));
    int nearRoad = 0;

    for (int i = 0; i < path.pointCount; ++i)
    {
        nearRoad += std::fabs(path[i].y + 200.0f) < 2.0f || std::fabs(path[i].y + 150.0f) < 2.0f;
    }

    CHECK(nearRoad >= 4);
}

TEST_CASE(Detail_KeepsOffMeshConnectionsAndFlags)
{
    AmeisenNavigation nav = MakeNav(TestWorld::Get().meshDir);
    REQUIRE(nav.NewClient(1));
    PathOptions options;
    options.detail = true;
    Path path(256);
    unsigned char flags[256]{};

    // Around the wall: corners keep their flags, start and end are flagged once.
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, Wow(-700.0f, 0.0f, -300.0f), Wow(-200.0f, 0.0f, -300.0f), path, nullptr,
                        &options, flags));
    CHECK(path.pointCount > 100); // ~600 yards at 4 yards
    CHECK_EQ(flags[0] & PATH_POINT_START, PATH_POINT_START);
    CHECK_EQ(flags[path.pointCount - 1] & PATH_POINT_END, PATH_POINT_END);
    int starts = 0;

    for (int i = 0; i < path.pointCount; ++i)
    {
        starts += (flags[i] & PATH_POINT_START) != 0;
    }

    CHECK_EQ(starts, 1);
}

TEST_CASE(Detail_SurfaceInfo)
{
    // Hill: the crest is marked as such, every point has the ground's area.
    {
        AmeisenNavigation nav = MakeNav(HillScene());
        REQUIRE(nav.NewClient(1));
        PathOptions options;
        options.detail = true;
        Path path(512);
        std::vector<unsigned char> info(512, 0xFF), areas(512, 0xFF);
        REQUIRE(nav.GetPath(1, HILL_MAP, Wow(-190.0f, 0.0f, LANE_Z), Wow(-30.0f, 0.0f, LANE_Z), path, nullptr, &options,
                            nullptr, info.data(), areas.data()));
        bool crest = false, spacing = false;

        for (int i = 0; i < path.pointCount; ++i)
        {
            crest |= (info[i] & POINT_INFO_CREST) && std::fabs(path[i].y - HILL_X) < 0.75f;
            spacing |= (info[i] & POINT_INFO_SPACING) != 0;
            CHECK(!(info[i] & POINT_INFO_CREST) || std::fabs(path[i].y - HILL_X) < 0.75f); // the only crest
            CHECK(!(info[i] & POINT_INFO_SURFACE_CHANGE));                                 // all one ground
            CHECK_EQ(static_cast<int>(areas[i]), static_cast<int>(TERRAIN_GROUND));
        }

        CHECK(crest);
        CHECK(spacing);
    }

    AmeisenNavigation nav = MakeNav(TestWorld::Get().meshDir);
    REQUIRE(nav.NewClient(1));
    Path path(256);
    std::vector<unsigned char> info(256, 0xFF), areas(256, 0xFF);

    // Pool, with DETAIL: the ground changes at both shores, the points in between are in water.
    PathOptions options;
    options.detail = true;
    options.waypointSpacing = 50.0f;
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, Wow(-150.0f, 0.0f, -430.0f), Wow(-30.0f, 0.0f, -430.0f), path, nullptr,
                        &options, nullptr, info.data(), areas.data()));
    int changes = 0, wet = 0;

    for (int i = 0; i < path.pointCount; ++i)
    {
        if (info[i] & POINT_INFO_SURFACE_CHANGE)
        {
            ++changes;
            REQUIRE(i + 1 < path.pointCount);
            CHECK(IsLiquidArea(areas[i]) != IsLiquidArea(areas[i + 1]));
            CHECK(std::fabs(path[i].y + 120.0f) < 2.0f || std::fabs(path[i].y + 60.0f) < 2.0f);
        }

        wet += IsLiquidArea(areas[i]);
        CHECK(areas[i] != 0);
    }

    CHECK(changes >= 2);
    CHECK(wet >= 2);

    // Without DETAIL: areas still filled, the info has only ground changes.
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, Wow(-150.0f, 0.0f, -430.0f), Wow(-30.0f, 0.0f, -430.0f), path, nullptr,
                        nullptr, nullptr, info.data(), areas.data()));

    for (int i = 0; i < path.pointCount; ++i)
    {
        CHECK(areas[i] != 0);
        CHECK((info[i] & ~POINT_INFO_SURFACE_CHANGE) == 0);
    }
}

TEST_CASE(Detail_KeepsAddedPointsFromEdges)
{
    // Along the wall (x = -300), one yard from it: the added points move away from the navmesh edge, the ends and the
    // plain corners stay.
    AmeisenNavigation nav = MakeNav(TestWorld::Get().meshDir);
    REQUIRE(nav.NewClient(1));
    const Vector3 a = Wow(TestWorld::WALL_X - 1.0f, 0.0f, -400.0f);
    const Vector3 b = Wow(TestWorld::WALL_X - 1.0f, 0.0f, -200.0f);
    Path path(512);
    PathOptions options;
    options.detail = true;

    const auto closest = [&](float& ends) {
        float best = 1000.0f;
        ends = std::max(path[0].DistanceTo(a), path[path.pointCount - 1].DistanceTo(b));

        for (int i = 1; i + 1 < path.pointCount; ++i)
        {
            best = std::min(best, TestWorld::WALL_X - path[i].y); // WoW y = RD x, the wall is at larger x
        }

        return best;
    };

    options.edgeMargin = 0.0f;
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, a, b, path, nullptr, &options));
    REQUIRE(path.pointCount > 10);
    float ends = 0.0f;
    const float hugging = closest(ends);
    CHECK(ends < 1.5f);    // the ends snap to the navmesh, two yards from the wall here
    CHECK(hugging < 2.5f); // the coarse test navmesh ends about two yards from the wall

    options.edgeMargin = 2.0f;
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, a, b, path, nullptr, &options));
    REQUIRE(path.pointCount > 10);
    const float away = closest(ends);
    CHECK(ends < 1.5f); // the ends snap to the navmesh, two yards from the wall here
    CHECK(away > hugging + 1.0f);

    // Still a valid path: consecutive points within the spacing, all near the ground.
    for (int i = 1; i < path.pointCount; ++i)
    {
        CHECK(path[i - 1].DistanceTo(path[i]) <= options.waypointSpacing + 2.0f);
        CHECK(std::fabs(path[i].z) < 1.0f);
    }
}

TEST_CASE(Detail_HeightsFromTheStaticWorld)
{
    // Flat ground at 0, platform at 5: with DETAIL and the .anw the points sit on the geometry itself, the plain path
    // keeps the navmesh heights (a little above it).
    AmeisenNavigation nav = MakeNav(TestWorld::Get().meshDir);
    REQUIRE(nav.NewClient(1));
    const Vector3 a = Wow(-700.0f, 0.0f, -50.0f);
    const Vector3 b = Wow(-400.0f, 0.0f, -50.0f);
    Path path(512);

    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, a, b, path));
    float plainMax = 0.0f;

    for (int i = 0; i < path.pointCount; ++i)
    {
        plainMax = std::max(plainMax, std::fabs(path[i].z));
    }

    PathOptions options;
    options.detail = true;
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, a, b, path, nullptr, &options));
    REQUIRE(path.pointCount > 20);

    for (int i = 0; i < path.pointCount; ++i)
    {
        CHECK(std::fabs(path[i].z) < 0.02f);
    }

    CHECK(plainMax > 0.05f); // the navmesh is above the ground, so the test tells the two apart

    // On the platform (the right floor, not the ground 5 yards below).
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, Wow(-445.0f, TestWorld::PLATFORM_Y, -445.0f),
                        Wow(-405.0f, TestWorld::PLATFORM_Y, -405.0f), path, nullptr, &options));
    REQUIRE(path.pointCount > 5);

    for (int i = 0; i < path.pointCount; ++i)
    {
        CHECK(std::fabs(path[i].z - TestWorld::PLATFORM_Y) < 0.02f);
    }
}
