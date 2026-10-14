#include "TestFramework.hpp"

#include <cmath>

#include "AmeisenNavigation.hpp"
#include "TestWorld.hpp"

namespace {
using TestWorld::Wow;

AmeisenNavigation& Nav()
{
    static AmeisenNavigation nav = [] {
        AmeisenNavigationSettings settings;
        settings.meshFolder = TestWorld::Get().meshDir;
        settings.useAnp = true;
        settings.maxPointPath = 256;
        return AmeisenNavigation(settings);
    }();
    return nav;
}

constexpr size_t CLIENT = 7;

void EnsureClient() { Nav().NewClient(CLIENT); }

float Length(const Path& path)
{
    float length = 0.0f;

    for (int i = 1; i < path.pointCount; ++i)
    {
        length += path[i].DistanceTo(path[i - 1]);
    }

    return length;
}

/// Every segment walkable along the navmesh.
bool AllSegmentsClear(const Path& path)
{
    for (int i = 1; i < path.pointCount; ++i)
    {
        if (!Nav().CastMovementRay(CLIENT, TestWorld::MAP_ID, path[i - 1], path[i]))
        {
            return false;
        }
    }

    return true;
}

/// Largest direction change at a path point (radians).
float MaxTurn(const Path& path)
{
    float maxTurn = 0.0f;

    for (int i = 1; i + 1 < path.pointCount; ++i)
    {
        const float ax = path[i].x - path[i - 1].x, ay = path[i].y - path[i - 1].y;
        const float bx = path[i + 1].x - path[i].x, by = path[i + 1].y - path[i].y;
        const float la = std::sqrt(ax * ax + ay * ay), lb = std::sqrt(bx * bx + by * by);

        if (la > 1e-3f && lb > 1e-3f)
        {
            maxTurn = std::max(maxTurn, std::acos(std::clamp((ax * bx + ay * by) / (la * lb), -1.0f, 1.0f)));
        }
    }

    return maxTurn;
}

/// Distance (2D) from a point to the end of the wall the paths go around (WoW x = RD z, WoW y = RD x).
float DistanceToWallEnd(const Vector3& p)
{
    const float dx = p.y - (TestWorld::WALL_X + 0.5f);
    const float dz = p.x - TestWorld::WALL_Z_MAX;
    return std::sqrt(dx * dx + dz * dz);
}

const Vector3 START = Wow(-700.0f, 0.0f, -300.0f);
const Vector3 END = Wow(-200.0f, 0.0f, -300.0f);
} // namespace

TEST_CASE(Refine_ShortcutIsNeverLongerAndStaysOnTheMesh)
{
    EnsureClient();

    Path plain(256);
    REQUIRE(Nav().GetPath(CLIENT, TestWorld::MAP_ID, START, END, plain));

    PathOptions options;
    options.shortcut = true;
    Path shortcut(256);
    REQUIRE(Nav().GetPath(CLIENT, TestWorld::MAP_ID, START, END, shortcut, nullptr, &options));

    CHECK(Length(shortcut) <= Length(plain) + 0.01f);
    CHECK(shortcut.pointCount <= plain.pointCount);
    CHECK(AllSegmentsClear(shortcut));
    CHECK(shortcut[shortcut.pointCount - 1].DistanceTo(plain[plain.pointCount - 1]) < 0.01f);
}

TEST_CASE(Refine_WallMarginMovesCornersAwayFromTheWall)
{
    EnsureClient();

    Path plain(256);
    REQUIRE(Nav().GetPath(CLIENT, TestWorld::MAP_ID, START, END, plain));

    PathOptions options;
    options.avoidWalls = true;
    options.tuning.wallMargin = 3.0f;
    Path margin(256);
    REQUIRE(Nav().GetPath(CLIENT, TestWorld::MAP_ID, START, END, margin, nullptr, &options));
    REQUIRE(margin.pointCount == plain.pointCount);

    float plainClosest = 1e9f;
    float marginClosest = 1e9f;

    for (int i = 1; i + 1 < plain.pointCount; ++i)
    {
        plainClosest = std::min(plainClosest, DistanceToWallEnd(plain[i]));
        marginClosest = std::min(marginClosest, DistanceToWallEnd(margin[i]));
    }

    CHECK(marginClosest > plainClosest + 1.5f);
    CHECK(AllSegmentsClear(margin));
}

TEST_CASE(Refine_RoundCornersReplacesSharpTurns)
{
    EnsureClient();

    Path plain(256);
    REQUIRE(Nav().GetPath(CLIENT, TestWorld::MAP_ID, START, END, plain));

    PathOptions options;
    options.roundCorners = true;
    options.tuning.cornerRadius = 6.0f;
    options.tuning.cornerPoints = 6;
    Path round(256);
    REQUIRE(Nav().GetPath(CLIENT, TestWorld::MAP_ID, START, END, round, nullptr, &options));

    CHECK(round.pointCount > plain.pointCount);
    CHECK(MaxTurn(round) < MaxTurn(plain) * 0.5f);

    // The arcs go around the corners: further from the wall end than the plain corners.
    float plainClosest = 1e9f;
    float roundClosest = 1e9f;

    for (const Vector3& p : plain)
    {
        plainClosest = std::min(plainClosest, DistanceToWallEnd(p));
    }

    for (const Vector3& p : round)
    {
        roundClosest = std::min(roundClosest, DistanceToWallEnd(p));
    }

    CHECK(roundClosest > plainClosest + 1.5f);
    CHECK(AllSegmentsClear(round));
    CHECK(round[0].DistanceTo(plain[0]) < 0.01f);
    CHECK(round[round.pointCount - 1].DistanceTo(plain[plain.pointCount - 1]) < 0.01f);
}

TEST_CASE(Refine_StopDistanceEndsInRangeWithLineOfSight)
{
    EnsureClient();

    PathOptions options;
    options.tuning.stopDistance = 10.0f;
    Path path(256);
    REQUIRE(Nav().GetPath(CLIENT, TestWorld::MAP_ID, START, END, path, nullptr, &options));

    const Vector3& last = path[path.pointCount - 1];
    CHECK(last.DistanceTo(END) <= 10.0f + 0.1f);
    CHECK(last.DistanceTo(END) > 9.0f);
    CHECK(Nav().CastMovementRay(CLIENT, TestWorld::MAP_ID, last, END));

    // Target right behind the wall: points within range on this side can't see it, the path goes around.
    const Vector3 behindWall = Wow(TestWorld::WALL_X + 6.0f, 0.0f, -300.0f);
    REQUIRE(Nav().GetPath(CLIENT, TestWorld::MAP_ID, START, behindWall, path, nullptr, &options));
    const Vector3& end = path[path.pointCount - 1];
    CHECK(end.DistanceTo(behindWall) <= 10.1f);
    CHECK(end.y > TestWorld::WALL_X); // RD x = WoW y: east of the wall
    CHECK(Nav().CastMovementRay(CLIENT, TestWorld::MAP_ID, end, behindWall));

    // Already in range: the path is just the start.
    REQUIRE(Nav().GetPath(CLIENT, TestWorld::MAP_ID, END, Wow(-205.0f, 0.0f, -300.0f), path, nullptr, &options));
    CHECK_EQ(path.pointCount, 1);
}

TEST_CASE(Refine_FollowKeepsTheCorridor)
{
    EnsureClient();

    PathOptions follow;
    follow.follow = true;
    Path first(256);
    REQUIRE(Nav().GetPath(CLIENT, TestWorld::MAP_ID, START, END, first, nullptr, &follow));
    REQUIRE(first.pointCount >= 3);

    // A few steps along the first segment, target moved a little: same corners ahead.
    const Vector3 moved = first[0] + (first[1] - first[0]) * (5.0f / first[0].DistanceTo(first[1]));
    const Vector3 target = END + Vector3(1.0f, 0.5f, 0.0f);
    Path second(256);
    REQUIRE(Nav().GetPath(CLIENT, TestWorld::MAP_ID, moved, target, second, nullptr, &follow));
    REQUIRE(second.pointCount == first.pointCount);
    CHECK(second[0].DistanceTo(moved) < 0.5f);

    for (int i = 1; i + 1 < first.pointCount; ++i)
    {
        CHECK(second[i].DistanceTo(first[i]) < 0.01f);
    }

    CHECK(second[second.pointCount - 1].DistanceTo(target) < 1.0f);

    // Teleported elsewhere: searched again, still a valid path.
    Path third(256);
    const Vector3 elsewhere = Wow(-600.0f, 0.0f, -50.0f);
    REQUIRE(Nav().GetPath(CLIENT, TestWorld::MAP_ID, elsewhere, END, third, nullptr, &follow));
    CHECK(third[0].DistanceTo(elsewhere) < 1.0f);
    CHECK(AllSegmentsClear(third));
}

TEST_CASE(Refine_AllOptionsTogether)
{
    EnsureClient();

    PathOptions options;
    options.shortcut = options.avoidWalls = options.roundCorners = true;
    options.tuning.stopDistance = 3.0f;

    for (const auto& [start, end] : {std::pair{START, END}, std::pair{Wow(-470.0f, 0.0f, -470.0f), END}})
    {
        Path path(256);
        REQUIRE(Nav().GetPath(CLIENT, TestWorld::MAP_ID, start, end, path, nullptr, &options));
        CHECK(AllSegmentsClear(path));
        CHECK(path[path.pointCount - 1].DistanceTo(end) <= 3.1f);

        REQUIRE(Nav().GetRandomPath(CLIENT, TestWorld::MAP_ID, start, end, path, 4.0f, nullptr, &options));
        CHECK(AllSegmentsClear(path));
    }

    // Exploration keeps visiting its waypoints, only corners are refined.
    const Vector3 square[]{Wow(-650.0f, 0.0f, -350.0f), Wow(-550.0f, 0.0f, -350.0f), Wow(-550.0f, 0.0f, -250.0f),
                           Wow(-650.0f, 0.0f, -250.0f)};
    Path route(256);
    ExploreResult result;
    REQUIRE(Nav().ExplorePolygon(CLIENT, TestWorld::MAP_ID, START, square, 30.0f, route, &result, &options));
    CHECK(result.reached == result.waypoints);
    CHECK(AllSegmentsClear(route));
}
