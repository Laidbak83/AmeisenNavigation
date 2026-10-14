#include "TestFramework.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>

#include "AmeisenNavigation.hpp"
#include "TestWorld.hpp"

namespace {
using TestWorld::Wow;

AmeisenNavigationSettings WorldSettings()
{
    AmeisenNavigationSettings settings;
    settings.meshFolder = TestWorld::Get().meshDir;
    settings.useAnp = true;
    settings.maxPointPath = 256;
    return settings;
}

/// Smallest 2D distance (WoW x/y) between a point and the path's segments.
float ClosestApproach(const Path& path, const Vector3& p)
{
    float best = 1e9f;

    for (int i = 1; i < path.pointCount; ++i)
    {
        const Vector3& a = path[i - 1];
        const Vector3& b = path[i];
        const float dx = b.x - a.x, dy = b.y - a.y;
        const float lengthSq = dx * dx + dy * dy;
        const float t = lengthSq > 0.0f ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / lengthSq, 0.0f, 1.0f) : 0.0f;
        const float cx = a.x + dx * t - p.x, cy = a.y + dy * t - p.y;
        best = std::min(best, std::sqrt(cx * cx + cy * cy));
    }

    return best;
}

/// True if a path point or a segment sample lies inside the water pool (RD x [-120, -60], z [-460, -400]).
bool CrossesPool(const Path& path)
{
    for (int i = 1; i < path.pointCount; ++i)
    {
        for (int s = 0; s <= 20; ++s)
        {
            const Vector3 p = path[i - 1] + (path[i] - path[i - 1]) * (static_cast<float>(s) / 20.0f);

            // WoW y = RD x, WoW x = RD z; a margin for the shore.
            if (p.y > -118.0f && p.y < -62.0f && p.x > -458.0f && p.x < -402.0f)
            {
                return true;
            }
        }
    }

    return false;
}
} // namespace

TEST_CASE(Avoid_ForbiddenAreaIsNeverCrossed)
{
    AmeisenNavigation nav(WorldSettings());
    REQUIRE(nav.NewClient(1));
    const Vector3 start = Wow(-140.0f, 0.0f, -430.0f);
    const Vector3 end = Wow(-40.0f, 0.0f, -430.0f);

    Path path(256);
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, start, end, path));
    CHECK(CrossesPool(path)); // swimming straight through is the shortest way

    // A bot that can't swim: water costs 0 = forbidden.
    const AreaCost noWater[]{{LIQUID_WATER, 0.0f}};
    REQUIRE(nav.GetClient(1)->ConfigureQueryFilter(ClientState::NORMAL, noWater));
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, start, end, path));
    CHECK(!CrossesPool(path));
    CHECK(path[path.pointCount - 1].DistanceTo(end) < 1.5f);
}

TEST_CASE(Avoid_ZonesAreGoneAround)
{
    AmeisenNavigation nav(WorldSettings());
    REQUIRE(nav.NewClient(1));
    const Vector3 start = Wow(-700.0f, 0.0f, -50.0f);
    const Vector3 end = Wow(-400.0f, 0.0f, -50.0f);
    const Vector3 center = Wow(-550.0f, 0.0f, -50.0f);

    Path path(256);
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, start, end, path));
    CHECK(ClosestApproach(path, center) < 1.0f); // straight through

    const AvoidZone zone{center, 20.0f, 50.0f};
    REQUIRE(nav.SetAvoidZones(1, TestWorld::MAP_ID, std::span(&zone, 1)));
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, start, end, path));
    CHECK(ClosestApproach(path, center) >= 20.0f);
    CHECK(path[path.pointCount - 1].DistanceTo(end) < 1.5f);

    for (int i = 1; i < path.pointCount; ++i)
    {
        CHECK(nav.CastMovementRay(1, TestWorld::MAP_ID, path[i - 1], path[i]));
    }

    // Other maps and other clients aren't affected, an empty list removes the zones.
    REQUIRE(nav.NewClient(2));
    REQUIRE(nav.GetPath(2, TestWorld::MAP_ID, start, end, path));
    CHECK(ClosestApproach(path, center) < 1.0f);
    REQUIRE(nav.SetAvoidZones(1, TestWorld::MAP_ID, {}));
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, start, end, path));
    CHECK(ClosestApproach(path, center) < 1.0f);

    // Invalid zones are rejected.
    const AvoidZone bad[]{{center, -1.0f, 2.0f}, {center, 5.0f, 0.5f}, {Wow(std::nanf(""), 0, 0), 5.0f, 2.0f}};

    for (const AvoidZone& z : bad)
    {
        CHECK(!nav.SetAvoidZones(1, TestWorld::MAP_ID, std::span(&z, 1)));
    }

    std::vector<AvoidZone> tooMany(MAX_AVOID_ZONES + 1, zone);
    CHECK(!nav.SetAvoidZones(1, TestWorld::MAP_ID, tooMany));
}

TEST_CASE(Avoid_StuckReportsArePersistedAndShared)
{
    const auto file = std::filesystem::temp_directory_path() / "anav_tests" / "stuck_zones.txt";
    std::filesystem::create_directories(file.parent_path());
    std::filesystem::remove(file);

    AmeisenNavigationSettings settings = WorldSettings();
    settings.avoidZonesFile = file;
    const Vector3 start = Wow(-700.0f, 0.0f, -50.0f);
    const Vector3 end = Wow(-400.0f, 0.0f, -50.0f);
    const Vector3 stuck = Wow(-550.0f, 0.0f, -50.0f);

    {
        AmeisenNavigation nav(settings);
        REQUIRE(nav.NewClient(1));
        CHECK(nav.ReportStuck(TestWorld::MAP_ID, stuck, 8.0f));
        CHECK(nav.ReportStuck(TestWorld::MAP_ID, stuck + Vector3(1.0f, 0.0f, 0.0f), 8.0f)); // merged
        CHECK_EQ(nav.GetGlobalZoneCount(TestWorld::MAP_ID), size_t{1});
        CHECK(!nav.ReportStuck(TestWorld::MAP_ID, Wow(std::nanf(""), 0, 0), 8.0f));

        Path path(256);
        REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, start, end, path));
        CHECK(ClosestApproach(path, stuck) >= 8.0f);
    }

    // Loaded again by the next server.
    AmeisenNavigation nav(settings);
    CHECK_EQ(nav.GetGlobalZoneCount(TestWorld::MAP_ID), size_t{1});
    REQUIRE(nav.NewClient(3));
    Path path(256);
    REQUIRE(nav.GetPath(3, TestWorld::MAP_ID, start, end, path));
    CHECK(ClosestApproach(path, stuck) >= 8.0f);
}

TEST_CASE(Avoid_SteerAvoidsUnitsAndWalls)
{
    AmeisenNavigation nav(WorldSettings());
    REQUIRE(nav.NewClient(1));

    // Heading east in the open, a unit comes head-on.
    const Vector3 position = Wow(-600.0f, 0.0f, -50.0f);
    const Vector3 target = Wow(-500.0f, 0.0f, -50.0f);
    const Vector3 east = Wow(7.0f, 0.0f, 0.0f); // RD +x

    SteerResult free;
    REQUIRE(nav.Steer(1, TestWorld::MAP_ID, position, east, target, 7.0f, 0.6f, 0.1f, {}, free));
    CHECK(free.velocity.y > 6.0f); // WoW y = RD x: straight at the target at full speed
    CHECK(std::fabs(free.velocity.x) < 1.0f);
    CHECK(free.position.DistanceTo(position) > 0.5f);

    const SteerUnit oncoming{Wow(-596.0f, 0.0f, -50.0f), Wow(-7.0f, 0.0f, 0.0f), 0.6f};
    SteerResult avoid;
    REQUIRE(nav.Steer(1, TestWorld::MAP_ID, position, east, target, 7.0f, 0.6f, 0.1f, std::span(&oncoming, 1), avoid));
    CHECK(std::fabs(avoid.velocity.x) > 1.0f); // sidesteps (WoW x = RD z)

    // Next to the wall, the target behind it: never through the wall.
    const Vector3 atWall = Wow(TestWorld::WALL_X - 2.0f, 0.0f, -300.0f);
    SteerResult wall;
    REQUIRE(nav.Steer(1, TestWorld::MAP_ID, atWall, east, Wow(TestWorld::WALL_X + 20.0f, 0.0f, -300.0f), 7.0f, 0.6f,
                      0.5f, {}, wall));
    CHECK(wall.position.y < TestWorld::WALL_X);

    // Invalid input.
    SteerResult result;
    CHECK(!nav.Steer(1, TestWorld::MAP_ID, position, east, target, -1.0f, 0.6f, 0.1f, {}, result));
    CHECK(!nav.Steer(1, TestWorld::MAP_ID, position, east, target, 7.0f, 0.6f, 0.0f, {}, result));
    CHECK(!nav.Steer(1, TestWorld::MAP_ID, Wow(std::nanf(""), 0, 0), east, target, 7.0f, 0.6f, 0.1f, {}, result));
}
