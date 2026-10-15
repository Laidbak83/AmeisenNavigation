#include "TestFramework.hpp"

#include <chrono>
#include <cmath>

#include "AmeisenNavigation.hpp"
#include "TestWorld.hpp"

// Navmesh islands: the test world's platform (y = 5, no drop links unless asked) is an island of its own.
namespace {
using TestWorld::Wow;

AmeisenNavigation MakeNav(float autoDropHeight = 0.0f, int maxSearchNodes = 65535)
{
    AmeisenNavigationSettings settings;
    settings.meshFolder = TestWorld::Get().meshDir;
    settings.useAnp = true;
    settings.maxPointPath = 256;
    settings.autoDropHeight = autoDropHeight;
    settings.maxSearchNodes = maxSearchNodes;
    return AmeisenNavigation(settings);
}

const Vector3 GROUND = Wow(-600.0f, 0.0f, -425.0f);
const Vector3 DECK = Wow(-425.0f, TestWorld::PLATFORM_Y, -425.0f);
} // namespace

TEST_CASE(Islands_LabelThePlatformSeparately)
{
    AmeisenNavigation nav = MakeNav();
    const auto islands = nav.GetIslands(TestWorld::MAP_ID);
    REQUIRE(islands != nullptr);
    CHECK(islands->Count() >= 2);
    CHECK(islands->PolyCount() > 10);
    CHECK(islands->LargestSize() > islands->PolyCount() / 2); // the ground
    CHECK_EQ(static_cast<int>(islands->Of(0)), 0);            // unknown reference

    // With drop links the platform joins the ground (one way, but a link is a link).
    AmeisenNavigation linked = MakeNav(8.0f);
    const auto joined = linked.GetIslands(TestWorld::MAP_ID);
    REQUIRE(joined != nullptr);
    CHECK(joined->Count() < islands->Count());
}

TEST_CASE(Islands_UnreachableTargetsFailFast)
{
    AmeisenNavigation nav = MakeNav();
    REQUIRE(nav.NewClient(1));
    Path path(256);
    bool partial = false;
    PathPartialReason reason = PathPartialReason::NONE;

    // Up onto the platform: another island. The path ends at the foot of the platform, not somewhere a search gave up.
    REQUIRE(
        nav.GetPath(1, TestWorld::MAP_ID, GROUND, DECK, path, &partial, nullptr, nullptr, nullptr, nullptr, &reason));
    CHECK(partial);
    CHECK(reason == PathPartialReason::UNREACHABLE);
    REQUIRE(path.pointCount >= 2);
    const Vector3& last = path[path.pointCount - 1];
    CHECK(last.z < 1.0f);                                                          // on the ground
    CHECK(std::fabs(last.x - DECK.x) < 6.0f && std::fabs(last.y - DECK.y) < 6.0f); // right below the deck's edge
    CHECK(last.DistanceTo(DECK) < 10.0f);

    // Same request, timed: the search is skipped (a full search of this small world would still be fast, so this
    // only guards against something absurd).
    const auto start = std::chrono::steady_clock::now();

    for (int i = 0; i < 20; ++i)
    {
        REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, GROUND, DECK, path, &partial, nullptr, nullptr, nullptr, nullptr,
                            &reason));
    }

    CHECK(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() <
          500);

    // Reachable: no reason.
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, GROUND, Wow(-200.0f, 0.0f, -50.0f), path, &partial, nullptr, nullptr,
                        nullptr, nullptr, &reason));
    CHECK(!partial);
    CHECK(reason == PathPartialReason::NONE);

    // Random paths and the follow corridor report it too.
    REQUIRE(nav.GetRandomPath(1, TestWorld::MAP_ID, GROUND, DECK, path, 2.0f, &partial, nullptr, nullptr, nullptr,
                              nullptr, &reason));
    CHECK(partial);
    CHECK(reason == PathPartialReason::UNREACHABLE);

    PathOptions follow;
    follow.follow = true;

    for (int i = 0; i < 2; ++i)
    {
        REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, GROUND, DECK, path, &partial, &follow, nullptr, nullptr, nullptr,
                            &reason));
        CHECK(partial);
        CHECK(reason == PathPartialReason::UNREACHABLE);
    }

    // Down from the platform with drop links: reachable.
    AmeisenNavigation linked = MakeNav(8.0f);
    REQUIRE(linked.NewClient(1));
    REQUIRE(linked.GetPath(1, TestWorld::MAP_ID, DECK, GROUND, path, &partial, nullptr, nullptr, nullptr, nullptr,
                           &reason));
    CHECK(!partial);
    CHECK(reason == PathPartialReason::NONE);
}

TEST_CASE(Islands_SearchLimitIsReported)
{
    // A tiny node budget: the way around the wall can't be found, the path is partial and says why.
    AmeisenNavigation nav = MakeNav(0.0f, 16);
    REQUIRE(nav.NewClient(1));
    Path path(256);
    bool partial = false;
    PathPartialReason reason = PathPartialReason::NONE;
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, Wow(-400.0f, 0.0f, -300.0f), Wow(-200.0f, 0.0f, -300.0f), path, &partial,
                        nullptr, nullptr, nullptr, nullptr, &reason));
    CHECK(partial);
    CHECK(reason == PathPartialReason::SEARCH_LIMIT);
}
