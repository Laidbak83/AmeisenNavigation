#include "TestFramework.hpp"

#include <cmath>
#include <cstring>

#include "AmeisenNavigation.hpp"
#include "TestWorld.hpp"

// Static world (.anw): the test world's geometry with sources (walls = WMO 77, platform = WMO 78, pillar = doodad 99),
// line of sight, surfaces and body sweeps.
namespace {
using TestWorld::Wow;

AmeisenNavigation& Navigation()
{
    static AmeisenNavigation nav = [] {
        AmeisenNavigationSettings settings;
        settings.meshFolder = TestWorld::Get().meshDir;
        settings.useAnp = true;
        settings.worldCacheTiles = 1; // two tiles: queries keep evicting and reloading
        return AmeisenNavigation(settings);
    }();
    return nav;
}

constexpr int MAP = TestWorld::MAP_ID;
} // namespace

TEST_CASE(World_TileFormatRoundTripAndCorruption)
{
    AnpWorld::Tile tile;
    tile.adtX = 32;
    tile.adtY = 31;
    tile.verts = {0.0f, 0.0f, 0.0f, 10.0f, 2.0f, 0.0f, 0.0f, 4.0f, 10.0f};
    tile.indices = {0, 1, 2};
    tile.areas = {WMO};
    tile.sources = {AnpWorld::MakeSource(AnpWorld::SourceKind::WMO, 1234)};
    tile.chunkAreas[17] = 1519;

    const auto bytes = AnpWorld::Serialize(tile);
    const auto back = AnpWorld::Deserialize(bytes.data(), bytes.size());
    REQUIRE(back.has_value());
    CHECK_EQ(back->adtX, 32);
    CHECK_EQ(back->adtY, 31);
    REQUIRE(back->verts.size() == 9);

    for (size_t i = 0; i < 9; ++i)
    {
        CHECK_NEAR(back->verts[i], tile.verts[i], 0.001); // 16 bit quantization over a 10 yard range
    }

    CHECK_EQ(back->sources[0], tile.sources[0]);
    CHECK_EQ(AnpWorld::IdOf(back->sources[0]), 1234u);
    CHECK(AnpWorld::KindOf(back->sources[0]) == AnpWorld::SourceKind::WMO);
    CHECK_EQ(back->chunkAreas[17], 1519);

    // Truncated, an index out of range, a bad magic.
    for (size_t size = 0; size < bytes.size(); ++size)
    {
        CHECK(!AnpWorld::Deserialize(bytes.data(), size).has_value());
    }

    auto bad = bytes;
    const uint32_t index = 3;
    std::memcpy(bad.data() + sizeof(AnpWorld::TileHeader) + 9 * 2, &index, 4);
    CHECK(!AnpWorld::Deserialize(bad.data(), bad.size()).has_value());
    bad = bytes;
    bad[0] = 'X';
    CHECK(!AnpWorld::Deserialize(bad.data(), bad.size()).has_value());
}

TEST_CASE(World_LineOfSightWithProvenance)
{
    World::Hit hit;

    // Through the wall at eye height: blocked by WMO 77.
    REQUIRE(Navigation().LineOfSight(MAP, Wow(-350.0f, 1.7f, -300.0f), Wow(-250.0f, 1.7f, -300.0f), 0, hit));
    CHECK(hit.hit);
    CHECK_EQ(hit.source, TestWorld::WALL_SOURCE);
    CHECK_NEAR(hit.position.y, TestWorld::WALL_X, 0.05); // WoW y = RD x
    CHECK_NEAR(hit.t, 0.5, 0.01);
    CHECK(hit.normal.y < -0.99f); // facing the ray (coming from -350)

    // Over the wall: clear.
    REQUIRE(Navigation().LineOfSight(MAP, Wow(-350.0f, 12.0f, -300.0f), Wow(-250.0f, 12.0f, -300.0f), 0, hit));
    CHECK(!hit.hit);
    CHECK_EQ(hit.t, 1.0f);

    // The pillar is a doodad: blocks, unless doodads are ignored (spells see through trees in some games).
    REQUIRE(Navigation().LineOfSight(MAP, Wow(-920.0f, 2.0f, -449.5f), Wow(-880.0f, 2.0f, -449.5f), 0, hit));
    CHECK(hit.hit);
    CHECK_EQ(hit.source, TestWorld::PILLAR_SOURCE);
    REQUIRE(Navigation().LineOfSight(MAP, Wow(-920.0f, 2.0f, -449.5f), Wow(-880.0f, 2.0f, -449.5f),
                                     World::IGNORE_DOODADS, hit));
    CHECK(!hit.hit);

    // Down through the pool's surface (y 0.5) but not to the ground: liquids never block.
    REQUIRE(Navigation().LineOfSight(MAP, Wow(-110.0f, 1.0f, -430.0f), Wow(-70.0f, 0.2f, -430.0f), 0, hit));
    CHECK(!hit.hit);

    // Into the ground: terrain.
    REQUIRE(Navigation().LineOfSight(MAP, Wow(-600.0f, 2.0f, -200.0f), Wow(-580.0f, -2.0f, -200.0f), 0, hit));
    CHECK(hit.hit);
    CHECK(AnpWorld::KindOf(hit.source) == AnpWorld::SourceKind::TERRAIN);
    CHECK_NEAR(hit.position.z, 0.0, 0.01);

    // Across both ADTs (x = -533.3 is the border).
    REQUIRE(Navigation().LineOfSight(MAP, Wow(-800.0f, 3.0f, -50.0f), Wow(-100.0f, 3.0f, -50.0f), 0, hit));
    CHECK(!hit.hit);

    // No static world: maps without one, outside the world, too long.
    CHECK(!Navigation().LineOfSight(MAP + 1, Wow(-350.0f, 1.7f, -300.0f), Wow(-250.0f, 1.7f, -300.0f), 0, hit));
    CHECK(!Navigation().LineOfSight(MAP, Wow(5000.0f, 1.0f, 5000.0f), Wow(5010.0f, 1.0f, 5000.0f), 0, hit));
    CHECK(!Navigation().LineOfSight(MAP, Wow(-1060.0f, 1.0f, -300.0f), Wow(0.0f, 1.0f, -300.0f), 0, hit));
    CHECK(!Navigation().LineOfSight(MAP, Vector3{NAN, 0.0f, 0.0f}, Wow(0.0f, 1.0f, -300.0f), 0, hit));
}

TEST_CASE(World_SurfacesUnderAPoint)
{
    std::vector<World::Surface> surfaces;
    uint32_t area = 0;

    // Under the platform: the deck (WMO 78) above the ground, both standable.
    const Vector3 deck = Wow(-425.0f, 0.0f, -425.0f);
    REQUIRE(Navigation().GetSurfaces(MAP, deck.x, deck.y, 50.0f, -50.0f, 0, surfaces, area));
    REQUIRE(surfaces.size() == 2);
    CHECK_NEAR(surfaces[0].z, TestWorld::PLATFORM_Y, 0.01);
    CHECK_EQ(surfaces[0].source, TestWorld::PLATFORM_SOURCE);
    CHECK(surfaces[0].standable && !surfaces[0].liquid);
    CHECK(surfaces[0].normal.z > 0.99f);
    CHECK_NEAR(surfaces[1].z, 0.0, 0.01);
    CHECK(AnpWorld::KindOf(surfaces[1].source) == AnpWorld::SourceKind::TERRAIN);
    CHECK_EQ(area, 12u);

    // Only the deck when looking from 6 down to 1, nothing below the ground.
    REQUIRE(Navigation().GetSurfaces(MAP, deck.x, deck.y, 6.0f, 1.0f, 0, surfaces, area));
    CHECK_EQ(surfaces.size(), size_t{1});
    REQUIRE(Navigation().GetSurfaces(MAP, deck.x, deck.y, -1.0f, -50.0f, 0, surfaces, area));
    CHECK(surfaces.empty());

    // The pool: a liquid surface over the ground; ignoring liquids leaves the ground.
    const Vector3 pool = Wow(-90.0f, 0.0f, -430.0f);
    REQUIRE(Navigation().GetSurfaces(MAP, pool.x, pool.y, 10.0f, -10.0f, 0, surfaces, area));
    REQUIRE(surfaces.size() == 2);
    CHECK(surfaces[0].liquid && !surfaces[0].standable);
    CHECK(AnpWorld::KindOf(surfaces[0].source) == AnpWorld::SourceKind::LIQUID);
    CHECK_NEAR(surfaces[0].z, 0.5, 0.01);
    REQUIRE(Navigation().GetSurfaces(MAP, pool.x, pool.y, 10.0f, -10.0f, World::IGNORE_LIQUIDS, surfaces, area));
    CHECK_EQ(surfaces.size(), size_t{1});

    // The pillar's top is a doodad.
    const Vector3 pillar = Wow(TestWorld::PILLAR_X + 0.5f, 0.0f, TestWorld::PILLAR_Z + 0.5f);
    REQUIRE(Navigation().GetSurfaces(MAP, pillar.x, pillar.y, 10.0f, -10.0f, 0, surfaces, area));
    REQUIRE(!surfaces.empty());
    CHECK_EQ(surfaces[0].source, TestWorld::PILLAR_SOURCE);
    CHECK_NEAR(surfaces[0].z, 6.0, 0.01);

    // Chunk area ids: 1519 in the first chunk of ADT (32,32).
    REQUIRE(Navigation().GetSurfaces(MAP, -10.0f, -10.0f, 10.0f, -10.0f, 0, surfaces, area));
    CHECK_EQ(area, 1519u);

    // Invalid ranges and outside the world.
    CHECK(!Navigation().GetSurfaces(MAP, deck.x, deck.y, -50.0f, 50.0f, 0, surfaces, area));
    CHECK(!Navigation().GetSurfaces(MAP, 9000.0f, 9000.0f, 10.0f, -10.0f, 0, surfaces, area));
    CHECK(!Navigation().GetSurfaces(MAP, NAN, deck.y, 10.0f, -10.0f, 0, surfaces, area));
}

TEST_CASE(World_BodySweep)
{
    World::Hit hit;
    const float radius = 0.5f;

    // Walking into the wall: stops a radius before it.
    const Vector3 a = Wow(-320.0f, 0.0f, -300.0f);
    const Vector3 b = Wow(-280.0f, 0.0f, -300.0f);
    REQUIRE(Navigation().SweepBody(MAP, a, b, radius, 2.0f, 0.5f, 0, hit));
    CHECK(hit.hit);
    CHECK_EQ(hit.source, TestWorld::WALL_SOURCE);
    CHECK_NEAR(hit.position.y, TestWorld::WALL_X - radius, 0.05);

    // Along the wall 1.5 yards away: clear. Sideways into it: blocked.
    REQUIRE(Navigation().SweepBody(MAP, Wow(-301.5f, 0.0f, -400.0f), Wow(-301.5f, 0.0f, -200.0f), radius, 2.0f, 0.5f, 0,
                                   hit));
    CHECK(!hit.hit);
    REQUIRE(Navigation().SweepBody(MAP, Wow(-301.6f, 0.0f, -130.0f), Wow(-299.0f, 0.0f, -130.0f), radius, 2.0f, 0.5f, 0,
                                   hit));
    CHECK(hit.hit);

    // Under the platform (deck at 5): a 2 yard body passes.
    REQUIRE(Navigation().SweepBody(MAP, Wow(-470.0f, 0.0f, -425.0f), Wow(-380.0f, 0.0f, -425.0f), radius, 2.0f, 0.5f, 0,
                                   hit));
    CHECK(!hit.hit);

    // The pillar blocks, ignoring doodads it doesn't.
    REQUIRE(Navigation().SweepBody(MAP, Wow(-910.0f, 0.0f, -449.5f), Wow(-890.0f, 0.0f, -449.5f), radius, 2.0f, 0.5f, 0,
                                   hit));
    CHECK(hit.hit);
    CHECK_EQ(hit.source, TestWorld::PILLAR_SOURCE);
    REQUIRE(Navigation().SweepBody(MAP, Wow(-910.0f, 0.0f, -449.5f), Wow(-890.0f, 0.0f, -449.5f), radius, 2.0f, 0.5f,
                                   World::IGNORE_DOODADS, hit));
    CHECK(!hit.hit);

    // Invalid bodies.
    CHECK(!Navigation().SweepBody(MAP, a, b, -1.0f, 2.0f, 0.5f, 0, hit));
    CHECK(!Navigation().SweepBody(MAP, a, b, radius, 0.4f, 0.5f, 0, hit));

    // The cache held one tile at a time through all of this.
    CHECK(Navigation().GetWorld(MAP)->CachedTiles() <= 1);
}
