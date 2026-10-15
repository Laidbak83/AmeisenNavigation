#include "TestFramework.hpp"

#include <cmath>
#include <filesystem>
#include <format>
#include <map>
#include <mutex>

#include "AmeisenNavigation.hpp"
#include "TestWorld.hpp"

// Deep water: the bed of a lake isn't walkable (characters swim there), shallow shores stay water to wade through,
// a tunnel under the lake is no water at all.
namespace {
using TestWorld::Wow;

constexpr int LAKE_MAP = 5;
constexpr float LAKE_DEPTH = 10.0f;
constexpr float SURFACE = -0.3f;
constexpr float TUNNEL_FLOOR = -25.0f;
constexpr float LANE = -200.0f; // RD z of the test paths

/// Sloped quad from x0 to x1 (RD), heights y0 -> y1, over z [z0, z1], facing up.
void AddSlope(Structure& s, float x0, float y0, float x1, float y1, float z0, float z1, TriAreaId area)
{
    const int b = static_cast<int>(s.verts.size());
    s.verts.push_back({x0, y0, z0});
    s.verts.push_back({x0, y0, z1});
    s.verts.push_back({x1, y1, z0});
    s.verts.push_back({x1, y1, z1});
    s.AddTri(Tri{b + 0, b + 1, b + 2}, area);
    s.AddTri(Tri{b + 2, b + 1, b + 3}, area);
}

/// Downward facing quad (a ceiling).
void AddCeiling(Structure& s, float x0, float z0, float x1, float z1, float y)
{
    const int b = static_cast<int>(s.verts.size());
    s.verts.push_back({x0, y, z0});
    s.verts.push_back({x0, y, z1});
    s.verts.push_back({x1, y, z0});
    s.verts.push_back({x1, y, z1});
    s.AddTri(Tri{b + 2, b + 1, b + 0}, WMO);
    s.AddTri(Tri{b + 3, b + 1, b + 2}, WMO);
}

/// RD layout: ground at 0, a lake from x -200 to -100 (shore slopes 10 yards wide, bed at -10, surface at -0.3)
/// across z -400 .. -1, a WMO tunnel 25 yards below the lake (floor, ceiling 4 yards up) from x -180 to -120.
const std::filesystem::path& LakeScene(float wadeDepth)
{
    static std::mutex mutex;
    static std::map<float, std::filesystem::path> built;
    const std::lock_guard lock(mutex);

    if (const auto it = built.find(wadeDepth); it != built.end())
    {
        return it->second;
    }

    const auto dir = std::filesystem::temp_directory_path() / "anav_tests" / std::format("lake_{}", wadeDepth);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);

    Structure s;
    AddSlope(s, -520, 0, -200, 0, -400, -1, TERRAIN_GROUND);
    AddSlope(s, -200, 0, -190, -LAKE_DEPTH, -400, -1, TERRAIN_GROUND);
    AddSlope(s, -190, -LAKE_DEPTH, -110, -LAKE_DEPTH, -400, -1, TERRAIN_GROUND);
    AddSlope(s, -110, -LAKE_DEPTH, -100, 0, -400, -1, TERRAIN_GROUND);
    AddSlope(s, -100, 0, -14, 0, -400, -1, TERRAIN_GROUND);
    TestWorld::AddQuad(s, -200, -400, -100, -1, SURFACE, LIQUID_WATER);
    TestWorld::AddQuad(s, -180, -260, -120, -140, TUNNEL_FLOOR, WMO);
    AddCeiling(s, -180, -260, -120, -140, TUNNEL_FLOOR + 4.0f);

    WaterMap water;
    RoadMap roads;
    // WoW corners: x = RD z, y = RD x.
    water.AddRect({-1.0f, -100.0f, 0.0f}, {-400.0f, -200.0f, 0.0f}, SURFACE, SURFACE, SURFACE, SURFACE, LIQUID_WATER);
    water.BuildSpatialIndex();
    roads.BuildSpatialIndex();

    dtNavMeshParams params{};
    params.orig[0] = (31 - 32) * TILESIZE;
    params.orig[2] = (31 - 32) * TILESIZE;
    params.tileWidth = params.tileHeight = TILESIZE;
    params.maxTiles = 64 * 64;
    params.maxPolys = 1 << 20;

    TileBuildConfig config = TestWorld::FastBuildConfig();
    config.maxWadeDepth = wadeDepth;
    Anp::AnpWriter writer(LAKE_MAP, params);
    AdtTileProcessor processor(&writer, dir, "Lake", config);
    processor.Process(&s, {{0, 0, 32, 32}}, &water, &roads);
    REQUIRE(writer.Save(dir));
    return built[wadeDepth] = dir;
}

AmeisenNavigation MakeNav(const std::filesystem::path& dir)
{
    AmeisenNavigationSettings settings;
    settings.meshFolder = dir;
    settings.useAnp = true;
    settings.maxPointPath = 256;
    return AmeisenNavigation(settings);
}

/// Area of the navmesh polygon nearest to an RD position (0 if none within 2 yards vertically).
unsigned char AreaAt(AmeisenNavigation& nav, float x, float y, float z, float* height = nullptr)
{
    Path path(4);
    std::vector<unsigned char> info(4), areas(4);
    const Vector3 p = Wow(x, y, z);

    if (!nav.GetPath(1, LAKE_MAP, p, p, path, nullptr, nullptr, nullptr, info.data(), areas.data()) ||
        std::fabs(path[0].z - y) > 2.0f)
    {
        return 0;
    }

    if (height)
    {
        *height = path[0].z;
    }

    return areas[0];
}
} // namespace

TEST_CASE(Water_DeepBedsAreSwumOver)
{
    AmeisenNavigation nav = MakeNav(LakeScene(1.5f));
    REQUIRE(nav.NewClient(1));

    // Nothing to stand on at the bottom of the lake, the surface is there.
    CHECK_EQ(static_cast<int>(AreaAt(nav, -150.0f, -LAKE_DEPTH, LANE)), 0);
    float height = 0.0f;
    CHECK(IsLiquidArea(AreaAt(nav, -150.0f, SURFACE, LANE, &height)));
    CHECK(height > SURFACE - 0.1f);

    // Wading depth at the shore: still water, still walkable.
    CHECK(IsLiquidArea(AreaAt(nav, -199.0f, -1.0f, LANE)));

    // A diving bot (8 yards down) gets a complete path out of the water.
    Path path(256);
    bool partial = true;
    PathPartialReason reason = PathPartialReason::UNREACHABLE;
    REQUIRE(nav.GetPath(1, LAKE_MAP, Wow(-150.0f, -8.0f, LANE), Wow(-30.0f, 0.0f, LANE), path, &partial, nullptr,
                        nullptr, nullptr, nullptr, &reason));
    CHECK(!partial);
    CHECK(reason == PathPartialReason::NONE);
    CHECK(path[path.pointCount - 1].DistanceTo(Wow(-30.0f, 0.0f, LANE)) < 1.0f);

    // Across the lake: on the surface.
    REQUIRE(nav.GetPath(1, LAKE_MAP, Wow(-300.0f, 0.0f, LANE), Wow(-30.0f, 0.0f, LANE), path, &partial));
    CHECK(!partial);

    for (int i = 0; i < path.pointCount; ++i)
    {
        CHECK(path[i].z > -1.5f);
    }

    // The lake bed isn't an island of its own any more (ground + surface, the tunnel).
    const auto islands = nav.GetIslands(LAKE_MAP);
    REQUIRE(islands != nullptr);
    CHECK_EQ(static_cast<int>(islands->Count()), 2);
}

TEST_CASE(Water_KeepsBedsWhenAsked)
{
    // Wade depth 0: the old behavior, the bed is walkable water (and its own island).
    AmeisenNavigation nav = MakeNav(LakeScene(0.0f));
    REQUIRE(nav.NewClient(1));
    CHECK(IsLiquidArea(AreaAt(nav, -150.0f, -LAKE_DEPTH, LANE)));
    const auto islands = nav.GetIslands(LAKE_MAP);
    REQUIRE(islands != nullptr);
    CHECK(islands->Count() > 2);
}

TEST_CASE(Water_TunnelUnderTheLakeIsNoWater)
{
    for (const float wade : {1.5f, 0.0f})
    {
        AmeisenNavigation nav = MakeNav(LakeScene(wade));
        REQUIRE(nav.NewClient(1));
        const unsigned char area = AreaAt(nav, -150.0f, TUNNEL_FLOOR, LANE);
        CHECK(area != 0);
        CHECK(!IsLiquidArea(area));
        CHECK(IsStructureArea(area));
    }
}
