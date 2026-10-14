#include "TestFramework.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <format>
#include <fstream>
#include <map>
#include <mutex>

#include "AmeisenNavigation.hpp"
#include "Exporter.hpp"
#include "TestWorld.hpp"
#include "WowDataBuilder.hpp"
#include "Wow/AdtChunkExtractor.hpp"
#include "Wow/GameObjects.hpp"

// Full exporter run on synthetic client data: MPQs (incl. a locale archive and a patch overriding an ADT)
// -> DBCs, WDT, ADT terrain/liquid/textures/areas, WMOs, M2s -> .anp -> navigation queries.
//
// Map 600 "TestContinent": ADTs (32,32) and (33,32). WoW x in [-T, 0], WoW y in [-2T, 0].
// Map 601 "TestDungeon":   WMO-only map (global WMO), a floor with a pillar around the origin.
// Map 602 "TestEmpty":     listed in Map.dbc, no WDT.
namespace {
using namespace WowData;

constexpr unsigned int CONTINENT = 600;
constexpr unsigned int DUNGEON = 601;
constexpr unsigned int EMPTY = 602;

// AreaTable.dbc
constexpr uint32_t AREA_CAPITAL = 1;       // capital city, Alliance
constexpr uint32_t AREA_DISTRICT = 2;      // sub area of the capital: inherits city + faction
constexpr uint32_t AREA_ALLIANCE_ZONE = 3; // Alliance territory
constexpr uint32_t AREA_HORDE_ZONE = 4;    // Horde territory
constexpr uint32_t AREA_CONTESTED = 5;     // contested (no faction)

constexpr uint16_t LIQUID_TYPE_OCEAN = 2; // LiquidType.dbc id with type 1 (ocean)

constexpr float WMO_HALF_SIZE = 30.0f;
constexpr float WMO_TOP = 3.0f; // above the terrain at the placement
constexpr float DOODAD_HALF_SIZE = 6.0f;
constexpr float DOODAD_TOP = 5.0f;
constexpr float DUNGEON_FLOOR_Z = 10.0f;
constexpr float DUNGEON_PILLAR_HALF_SIZE = 20.0f;

/// Terrain height at a WoW position (gently sloped in both directions).
float TerrainHeight(float wowX, float wowY) { return 2.0f + 0.01f * wowX + 0.005f * wowY; }

/// Water surface of the ocean pool: sloped along WoW y so a wrong MH2O height layout shows up.
float WaterHeight(float /*wowX*/, float wowY) { return 8.0f + 0.01f * (wowY + 766.67f); }

/// Center of an ADT chunk in WoW coordinates.
Vector3 ChunkCenter(int adtX, int adtY, int cx, int cy)
{
    const auto [x, y] = ChunkOrigin(adtX, adtY, cx, cy);
    const float wx = x - CHUNKSIZE / 2.0f;
    const float wy = y - CHUNKSIZE / 2.0f;
    return {wx, wy, TerrainHeight(wx, wy)};
}

// Feature placement (WoW coordinates)
const Vector3 ROAD = ChunkCenter(32, 32, 4, 4);
const Vector3 CITY = ChunkCenter(32, 32, 8, 4);
const Vector3 ALLIANCE = ChunkCenter(32, 32, 12, 4);
const Vector3 HORDE = ChunkCenter(32, 32, 12, 8);
const Vector3 CONTESTED = ChunkCenter(32, 32, 12, 12);
const Vector3 HOLE = ChunkCenter(32, 32, 4, 10);
const Vector3 PLAIN = ChunkCenter(32, 32, 1, 14);
const Vector3 WATER = ChunkCenter(33, 32, 5, 5); // pool over chunks 4..6 x 4..6
const Vector3 WMO_POS{-250.0f, -TILESIZE, TerrainHeight(-250.0f, -TILESIZE)}; // on the ADT border
const Vector3 DOODAD_POS{-450.0f, -300.0f, TerrainHeight(-450.0f, -300.0f)};

// Game objects: a fence by display id (GameObjectDisplayInfo.dbc names it .mdx like the real client).
constexpr uint32_t FENCE_DISPLAY_ID = 500;
const char* const FENCE_MODEL = "World\\Generic\\Test\\Fence.mdx";
constexpr float FENCE_HALF_LENGTH = 15.0f;
constexpr float FENCE_HEIGHT = 1.4f;

const char* const WMO_NAME = "World\\wmo\\Test\\Platform.wmo";
const char* const DUNGEON_WMO_NAME = "World\\wmo\\Dungeon\\TestDungeon\\TestDungeon.wmo";
const char* const DOODAD_NAME = "World\\Generic\\Test\\Crate.mdx"; // exporter swaps the extension to .m2

/// ADT placements store (WORLDSIZE - wowY, wowZ, WORLDSIZE - wowX).
MODF::Entry WmoPlacement(const Vector3& wow, uint32_t uniqueId)
{
    MODF::Entry entry{};
    entry.id = 0;
    entry.uniqueId = uniqueId;
    entry.x = WORLDSIZE - wow.y;
    entry.y = wow.z;
    entry.z = WORLDSIZE - wow.x;
    entry.scale = 1024;
    return entry;
}

AdtSpec ContinentAdt(int x, bool withWater)
{
    AdtSpec spec;
    spec.x = x;
    spec.y = 32;
    spec.height = TerrainHeight;
    spec.textures = {"tileset\\generic\\grass.blp", "tileset\\elwynn\\ElwynnCobblestoneBase.blp"};

    // Placements are listed in every ADT they overlap, with the same uniqueId.
    spec.wmoNames = {WMO_NAME};
    spec.wmoPlacements = {WmoPlacement(WMO_POS, 1000)};

    if (x == 32)
    {
        spec.chunkTexture[{4, 4}] = 1;
        spec.chunkArea[{8, 4}] = AREA_DISTRICT;
        spec.chunkArea[{12, 4}] = AREA_ALLIANCE_ZONE;
        spec.chunkArea[{12, 8}] = AREA_HORDE_ZONE;
        spec.chunkArea[{12, 12}] = AREA_CONTESTED;
        spec.holes.insert({4, 10});

        MDDF::Entry doodad{};
        doodad.id = 0;
        doodad.uniqueId = 2000;
        doodad.x = WORLDSIZE - DOODAD_POS.y;
        doodad.y = DOODAD_POS.z;
        doodad.z = WORLDSIZE - DOODAD_POS.x;
        doodad.scale = 1024;
        spec.doodadNames = {DOODAD_NAME};
        spec.doodadPlacements = {doodad};
    }

    if (withWater)
    {
        for (int cy = 4; cy <= 6; ++cy)
        {
            for (int cx = 4; cx <= 6; ++cx)
            {
                // No structured binding: Clang can't capture those in lambdas when OpenMP is enabled.
                const std::pair<float, float> origin = ChunkOrigin(x, 32, cx, cy);
                spec.water.push_back({cx, cy, LIQUID_TYPE_OCEAN, [origin](int vx, int vy) {
                                          return WaterHeight(origin.first - vy * UNITSIZE,
                                                             origin.second - vx * UNITSIZE);
                                      }});
            }
        }
    }

    return spec;
}

Bytes MakeBoxWmoGroup(Vector3 min, Vector3 max)
{
    std::vector<Vector3> verts;
    std::vector<uint16_t> indices;
    AddBox(verts, indices, min, max);
    return MakeWmoGroup(verts, indices);
}

struct Fixture
{
    std::filesystem::path wowDir;
    std::filesystem::path outputDir;
    bool dataOk = false;
    ExportReport report;

    const MapExportStats* Map(unsigned int id) const
    {
        for (const auto& m : report.maps)
        {
            if (m.mapId == id)
            {
                return &m;
            }
        }

        return nullptr;
    }
};

bool WriteClientData(const std::filesystem::path& wowDir)
{
    std::filesystem::remove_all(wowDir);
    const auto dataDir = wowDir / "Data";

    // DBCs (in the locale archive, like the real client)
    StringTable mapNames;
    const auto mapDbc = MakeDbc(2, {{CONTINENT, mapNames.Add("TestContinent")},
                                    {DUNGEON, mapNames.Add("TestDungeon")},
                                    {EMPTY, mapNames.Add("TestEmpty")}},
                                mapNames);

    StringTable liquidNames;
    const auto liquidDbc = MakeDbc(4, {{1, liquidNames.Add("Water"), 0, 0},
                                       {LIQUID_TYPE_OCEAN, liquidNames.Add("Ocean"), 0, 1},
                                       {3, liquidNames.Add("Magma"), 0, 2}},
                                   liquidNames);

    const auto area = [](uint32_t id, uint32_t parent, uint32_t flags, uint32_t factionMask) {
        std::vector<uint32_t> record(36, 0);
        record[0] = id;
        record[2] = parent;
        record[4] = flags;
        record[28] = factionMask;
        return record;
    };

    const auto areaDbc = MakeDbc(36,
                                 {area(AREA_CAPITAL, 0, 0x08, 2), area(AREA_DISTRICT, AREA_CAPITAL, 0, 0),
                                  area(AREA_ALLIANCE_ZONE, 0, 0, 2), area(AREA_HORDE_ZONE, 0, 0, 4),
                                  area(AREA_CONTESTED, 0, 0, 0)},
                                 StringTable{});

    // GameObjectDisplayInfo.dbc: field 0 = id, 1 = model name (19 fields in WotLK).
    StringTable displayNames;
    const auto displayDbc = MakeDbc(19, {{FENCE_DISPLAY_ID, displayNames.Add(FENCE_MODEL)}}, displayNames);

    if (!WriteMpq(dataDir / "enUS" / "locale-enUS.MPQ", {{"DBFilesClient\\Map.dbc", mapDbc},
                                                         {"DBFilesClient\\LiquidType.dbc", liquidDbc},
                                                         {"DBFilesClient\\AreaTable.dbc", areaDbc},
                                                         {"DBFilesClient\\GameObjectDisplayInfo.dbc", displayDbc}}))
    {
        return false;
    }

    // Continent: terrain, textures, areas, hole, WMO across the ADT border, doodad. No water in the base archive.
    std::vector<std::pair<std::string, Bytes>> common;
    common.emplace_back("World\\Maps\\TestContinent\\TestContinent.wdt", MakeWdt({{32, 32}, {33, 32}}));
    common.emplace_back("World\\Maps\\TestContinent\\TestContinent_32_32.adt", MakeAdt(ContinentAdt(32, false)));
    common.emplace_back("World\\Maps\\TestContinent\\TestContinent_33_32.adt", MakeAdt(ContinentAdt(33, false)));

    common.emplace_back(WMO_NAME, MakeWmoRoot(1));
    common.emplace_back("World\\wmo\\Test\\Platform_000.wmo",
                        MakeBoxWmoGroup({-WMO_HALF_SIZE, -WMO_HALF_SIZE, -3.0f},
                                        {WMO_HALF_SIZE, WMO_HALF_SIZE, WMO_TOP}));

    {
        std::vector<Vector3> verts;
        std::vector<uint16_t> indices;
        AddBox(verts, indices, {-DOODAD_HALF_SIZE, -DOODAD_HALF_SIZE, -1.0f},
               {DOODAD_HALF_SIZE, DOODAD_HALF_SIZE, DOODAD_TOP});
        common.emplace_back("World\\Generic\\Test\\Crate.m2", MakeM2(verts, indices));
    }

    {
        // Game object fence: 30 yards along the model's x axis, 1.4 high (vanilla header, like old clients).
        std::vector<Vector3> verts;
        std::vector<uint16_t> indices;
        AddBox(verts, indices, {-FENCE_HALF_LENGTH, -0.1f, -0.3f}, {FENCE_HALF_LENGTH, 0.1f, FENCE_HEIGHT});
        common.emplace_back("World\\Generic\\Test\\Fence.m2", MakeM2(verts, indices, 256));
    }

    // Dungeon: global WMO at the origin, a floor (group 0) and a pillar (group 1).
    MODF::Entry dungeonPlacement{};
    dungeonPlacement.uniqueId = 1;
    common.emplace_back("World\\Maps\\TestDungeon\\TestDungeon.wdt",
                        MakeWdt({}, DUNGEON_WMO_NAME, &dungeonPlacement));
    common.emplace_back(DUNGEON_WMO_NAME, MakeWmoRoot(2));

    {
        std::vector<Vector3> verts;
        std::vector<uint16_t> indices;
        AddQuad(verts, indices, -60.0f, -60.0f, 60.0f, 60.0f, DUNGEON_FLOOR_Z);
        common.emplace_back("World\\wmo\\Dungeon\\TestDungeon\\TestDungeon_000.wmo", MakeWmoGroup(verts, indices));
    }

    common.emplace_back("World\\wmo\\Dungeon\\TestDungeon\\TestDungeon_001.wmo",
                        MakeBoxWmoGroup({-DUNGEON_PILLAR_HALF_SIZE, -DUNGEON_PILLAR_HALF_SIZE, 5.0f},
                                        {DUNGEON_PILLAR_HALF_SIZE, DUNGEON_PILLAR_HALF_SIZE, 30.0f}));

    if (!WriteMpq(dataDir / "common.MPQ", common))
    {
        return false;
    }

    // The patch archive overrides ADT (33,32) with a version that has an ocean pool (MH2O).
    return WriteMpq(dataDir / "patch.MPQ", {{"World\\Maps\\TestContinent\\TestContinent_33_32.adt",
                                             MakeAdt(ContinentAdt(33, true))}});
}

TileBuildConfig BuildConfig()
{
    TileBuildConfig config = TestWorld::FastBuildConfig();
    config.meshResolution = 1024; // cs = ch ~ 0.52
    return config;
}

/// Cell height of the test meshes.
const float CH = TILESIZE / static_cast<float>(BuildConfig().meshResolution);

/// Recast rounds span tops up to the next cell and contour vertices take the highest neighbour span, so
/// navmesh heights lie up to ~2 cells above the source surface, never noticeably below it.
bool OnSurface(float navHeight, float expected)
{
    const bool ok = navHeight >= expected - 0.25f && navHeight <= expected + 2.0f * CH + 0.1f;

    if (!ok)
    {
        std::printf("  height %f not on surface %f (+%f)\n", navHeight, expected, 2.0f * CH + 0.1f);
    }

    return ok;
}

ExportOptions BaseOptions(const Fixture& f)
{
    ExportOptions options;
    options.wowDir = f.wowDir; // client root, the exporter has to find Data itself
    options.outputDir = f.outputDir;
    options.buildConfig = BuildConfig();
    return options;
}

const Fixture& Get()
{
    static std::once_flag once;
    static Fixture fixture;

    std::call_once(once, [] {
        const auto root = std::filesystem::temp_directory_path() / "anav_tests" / "exporter";
        fixture.wowDir = root / "wow";
        fixture.outputDir = root / "meshes";
        std::filesystem::remove_all(fixture.outputDir);

        fixture.dataOk = WriteClientData(fixture.wowDir);

        if (fixture.dataOk)
        {
            fixture.report = RunExport(BaseOptions(fixture));
        }
    });

    return fixture;
}

AmeisenNavigation& Navigation()
{
    static AmeisenNavigation nav = [] {
        AmeisenNavigationSettings settings;
        settings.meshFolder = Get().outputDir;
        settings.useAnp = true;
        settings.maxPointPath = 256;
        return AmeisenNavigation(settings);
    }();
    return nav;
}

constexpr size_t CLIENT = 7;

/// Area of the polygon closest to a WoW position.
unsigned char AreaAt(unsigned int mapId, const Vector3& wow)
{
    static std::map<unsigned int, Anp::LoadResult> meshes;
    auto it = meshes.find(mapId);

    if (it == meshes.end())
    {
        it = meshes.emplace(mapId, Anp::Load(Get().outputDir / Anp::FileName(static_cast<int>(mapId)))).first;
    }

    REQUIRE(it->second.navMesh);

    NavMeshQueryPtr query(dtAllocNavMeshQuery());
    REQUIRE(dtStatusSucceed(query->init(it->second.navMesh.get(), 2048)));

    dtQueryFilter filter;
    const float pos[3]{wow.y, wow.z, wow.x};
    const float extents[3]{0.5f, 1.5f, 0.5f};
    dtPolyRef ref = 0;
    float nearest[3]{};
    REQUIRE(dtStatusSucceed(query->findNearestPoly(pos, extents, &filter, &ref, nearest)));
    REQUIRE(ref != 0);

    const dtMeshTile* tile = nullptr;
    const dtPoly* poly = nullptr;
    REQUIRE(dtStatusSucceed(it->second.navMesh->getTileAndPolyByRef(ref, &tile, &poly)));
    return poly->getArea();
}

/// Navmesh height closest to the probe, NAN if there is none.
float HeightAt(unsigned int mapId, float wowX, float wowY, float probeZ)
{
    Navigation().NewClient(CLIENT);
    Vector3 out;
    return Navigation().GetHeight(CLIENT, static_cast<int>(mapId), {wowX, wowY, probeZ}, out) ? out.z : NAN;
}

float PathLength(const Path& path)
{
    float length = 0.0f;

    for (int i = 1; i < path.pointCount; ++i)
    {
        length += path[i].DistanceTo(path[i - 1]);
    }

    return length;
}
} // namespace

TEST_CASE(Exporter_ListsMapsFromLocaleArchive)
{
    REQUIRE(Get().dataOk);

    const auto maps = ListMaps(Get().wowDir);
    REQUIRE(maps.size() == 3);
    CHECK_EQ(maps[0].first, CONTINENT);
    CHECK(maps[0].second == "TestContinent");
    CHECK_EQ(maps[1].first, DUNGEON);
    CHECK(maps[2].second == "TestEmpty");

    CHECK(ListMaps(Get().wowDir / "does_not_exist").empty());
}

TEST_CASE(Exporter_ExportsAllMaps)
{
    const auto& f = Get();
    REQUIRE(f.dataOk);
    REQUIRE(f.report.setupOk);
    CHECK_EQ(f.report.failedMaps, 0);
    CHECK_EQ(f.report.maps.size(), 3u);

    const MapExportStats* continent = f.Map(CONTINENT);
    REQUIRE(continent);
    CHECK(continent->exported);
    CHECK(!continent->wmoOnly);
    CHECK_EQ(continent->adts, 2);
    CHECK_EQ(continent->tilesBuilt, 2);
    CHECK_EQ(continent->tilesFailed, 0);
    CHECK(std::filesystem::exists(f.outputDir / "600.anp"));

    const MapExportStats* dungeon = f.Map(DUNGEON);
    REQUIRE(dungeon);
    CHECK(dungeon->exported);
    CHECK(dungeon->wmoOnly);
    CHECK_EQ(dungeon->adts, 4); // the floor covers the corners of 4 ADTs around the origin
    CHECK_EQ(dungeon->tilesBuilt, 4);
    CHECK(std::filesystem::exists(f.outputDir / "601.anp"));

    const MapExportStats* empty = f.Map(EMPTY);
    REQUIRE(empty);
    CHECK(empty->skipped);
    CHECK(!empty->exported);
    CHECK(!empty->failed);
    CHECK(!std::filesystem::exists(f.outputDir / "602.anp"));
}

TEST_CASE(Exporter_ExtractsAdtContent)
{
    const MapExportStats* continent = Get().Map(CONTINENT);
    REQUIRE(continent);

    // Listed in both ADTs, extracted once.
    CHECK_EQ(continent->wmoPlacements, 1u);
    CHECK_EQ(continent->doodadPlacements, 1u);

    CHECK_EQ(continent->roadRects, 64u);       // one chunk with the cobblestone texture as dominant layer
    CHECK_EQ(continent->cityRects, 1u);        // district chunk, city through its parent area
    CHECK_EQ(continent->factionRects, 3u);     // district (via parent), Alliance and Horde zone
    CHECK_EQ(continent->waterRects, 9u * 64u); // 3x3 chunks of ocean from the patch archive

    // 2 ADTs x 256 chunks x 256 triangles minus the holed chunk, water surfaces (2 per quad), 2 boxes.
    CHECK_EQ(continent->triangles, 2u * 256u * 256u - 256u + 9u * 64u * 2u + 2u * 12u);
    // Terrain vertices are shared between chunks and ADTs after cleaning: 2 x (129^2 + 128^2) - 129 shared.
    CHECK(continent->vertices >= 2u * (129u * 129u + 128u * 128u) - 129u);
}

TEST_CASE(Exporter_TerrainHeights)
{
    for (const Vector3& p : {PLAIN, ROAD, ALLIANCE, CONTESTED, Vector3(-500.0f, -900.0f, 0.0f),
                             Vector3(-60.0f, -1000.0f, 0.0f)})
    {
        const float expected = TerrainHeight(p.x, p.y);
        CHECK(OnSurface(HeightAt(CONTINENT, p.x, p.y, expected + 1.0f), expected));
    }
}

TEST_CASE(Exporter_AreasFromTexturesAndAreaTable)
{
    CHECK_EQ(AreaAt(CONTINENT, PLAIN), TERRAIN_GROUND);
    CHECK_EQ(AreaAt(CONTINENT, CONTESTED), TERRAIN_GROUND);
    CHECK_EQ(AreaAt(CONTINENT, ROAD), TERRAIN_ROAD);
    CHECK_EQ(AreaAt(CONTINENT, CITY), ALLIANCE_TERRAIN_CITY);
    CHECK_EQ(AreaAt(CONTINENT, ALLIANCE), ALLIANCE_TERRAIN_GROUND);
    CHECK_EQ(AreaAt(CONTINENT, HORDE), HORDE_TERRAIN_GROUND);
}

TEST_CASE(Exporter_Mh2oWaterSurfaceFromPatch)
{
    // Water surface ~12 yards above the ground, sloped along WoW y.
    for (const float dy : {-35.0f, 0.0f, 35.0f})
    {
        const float x = WATER.x;
        const float y = WATER.y + dy;
        const float expected = WaterHeight(x, y);
        CHECK(OnSurface(HeightAt(CONTINENT, x, y, expected + 0.5f), expected));
        CHECK_EQ(AreaAt(CONTINENT, {x, y, expected + CH}), LIQUID_OCEAN);
    }

    // The sea floor below is still there.
    const float ground = TerrainHeight(WATER.x, WATER.y);
    CHECK(OnSurface(HeightAt(CONTINENT, WATER.x, WATER.y, ground + 0.5f), ground));
}

TEST_CASE(Exporter_TerrainHolesHaveNoMesh)
{
    CHECK(std::isnan(HeightAt(CONTINENT, HOLE.x, HOLE.y, HOLE.z)));

    // A path straight across the hole has to go around it.
    Navigation().NewClient(CLIENT);
    const Vector3 start{HOLE.x, HOLE.y + 40.0f, TerrainHeight(HOLE.x, HOLE.y + 40.0f)};
    const Vector3 end{HOLE.x, HOLE.y - 40.0f, TerrainHeight(HOLE.x, HOLE.y - 40.0f)};
    Path path(256);
    REQUIRE(Navigation().GetPath(CLIENT, static_cast<int>(CONTINENT), start, end, path));
    REQUIRE(path.pointCount >= 3);
    CHECK(PathLength(path) > start.DistanceTo(end) + 4.0f);
    CHECK_NEAR(path[path.pointCount - 1].x, end.x, 1.5);
    CHECK_NEAR(path[path.pointCount - 1].y, end.y, 1.5);
}

TEST_CASE(Exporter_WmoAcrossAdtBorderIsComplete)
{
    const float top = WMO_POS.z + WMO_TOP;

    // Both halves of the platform exist (it's listed in both ADTs but must not be clipped or lost).
    for (const float dy : {-20.0f, -5.0f, 5.0f, 20.0f})
    {
        CHECK(OnSurface(HeightAt(CONTINENT, WMO_POS.x, WMO_POS.y + dy, top + 0.5f), top));
    }

    CHECK_EQ(AreaAt(CONTINENT, {WMO_POS.x, WMO_POS.y + 10.0f, top + CH}), WMO);
}

TEST_CASE(Exporter_DoodadCollision)
{
    const float top = DOODAD_POS.z + DOODAD_TOP;
    CHECK(OnSurface(HeightAt(CONTINENT, DOODAD_POS.x, DOODAD_POS.y, top + 0.5f), top));
    CHECK_EQ(AreaAt(CONTINENT, {DOODAD_POS.x, DOODAD_POS.y, top + CH}), DOODAD);
}

TEST_CASE(Exporter_WmoOnlyDungeon)
{
    CHECK(OnSurface(HeightAt(DUNGEON, 40.0f, 40.0f, DUNGEON_FLOOR_Z + 1.0f), DUNGEON_FLOOR_Z));
    CHECK(OnSurface(HeightAt(DUNGEON, -40.0f, 40.0f, DUNGEON_FLOOR_Z + 1.0f), DUNGEON_FLOOR_Z));
    CHECK_EQ(AreaAt(DUNGEON, {40.0f, -40.0f, DUNGEON_FLOOR_Z + CH}), WMO);

    // Around the pillar, across all 4 tiles.
    Navigation().NewClient(CLIENT);
    const Vector3 start{0.0f, -45.0f, DUNGEON_FLOOR_Z};
    const Vector3 end{0.0f, 45.0f, DUNGEON_FLOOR_Z};
    Path path(256);
    REQUIRE(Navigation().GetPath(CLIENT, static_cast<int>(DUNGEON), start, end, path));
    REQUIRE(path.pointCount >= 3);
    CHECK(PathLength(path) > start.DistanceTo(end) + 5.0f);
    CHECK_NEAR(path[path.pointCount - 1].x, end.x, 1.5);
    CHECK_NEAR(path[path.pointCount - 1].y, end.y, 1.5);

    for (const auto& p : path)
    {
        CHECK(OnSurface(p.z, DUNGEON_FLOOR_Z));
    }
}

TEST_CASE(Exporter_SkipExisting)
{
    const auto& f = Get();
    REQUIRE(f.report.setupOk);

    const auto anp = f.outputDir / "600.anp";
    REQUIRE(std::filesystem::exists(anp));
    const auto writeTime = std::filesystem::last_write_time(anp);

    ExportOptions options = BaseOptions(f);
    options.mapIds = {static_cast<int>(CONTINENT)};
    options.skipExisting = true;

    const ExportReport report = RunExport(options);
    REQUIRE(report.setupOk);
    REQUIRE(report.maps.size() == 1);
    CHECK(report.maps[0].skipped);
    CHECK(!report.maps[0].exported);
    CHECK(std::filesystem::last_write_time(anp) == writeTime);
}

TEST_CASE(Exporter_SingleTile)
{
    const auto& f = Get();
    REQUIRE(f.dataOk);

    ExportOptions options = BaseOptions(f);
    options.outputDir = f.outputDir.parent_path() / "single";
    options.mapIds = {static_cast<int>(CONTINENT), static_cast<int>(DUNGEON)};
    options.tileX = 32;
    options.tileY = 32;
    std::filesystem::remove_all(options.outputDir);

    const ExportReport report = RunExport(options);
    REQUIRE(report.setupOk);
    CHECK_EQ(report.failedMaps, 0);
    REQUIRE(report.maps.size() == 2);

    CHECK(report.maps[0].exported);
    CHECK_EQ(report.maps[0].adts, 1);
    CHECK_EQ(report.maps[0].tilesBuilt, 1);
    CHECK_EQ(report.maps[0].wmoPlacements, 1u);
    CHECK_EQ(report.maps[0].waterRects, 0u);

    // WMO-only maps have no ADT to select.
    CHECK(report.maps[1].skipped);

    const auto loaded = Anp::Load(options.outputDir / "600.anp");
    REQUIRE(loaded.navMesh);
    CHECK_EQ(loaded.tilesLoaded, 1);
}

TEST_CASE(Exporter_BadInputFailsCleanly)
{
    ExportOptions options;
    options.wowDir = std::filesystem::temp_directory_path() / "anav_tests" / "does_not_exist";
    options.outputDir = std::filesystem::temp_directory_path() / "anav_tests" / "bad_output";
    CHECK(!RunExport(options).setupOk);

    // A folder without MPQs.
    const auto emptyDir = std::filesystem::temp_directory_path() / "anav_tests" / "empty_wow";
    std::filesystem::create_directories(emptyDir);
    options.wowDir = emptyDir;
    CHECK(!RunExport(options).setupOk);
}

TEST_CASE(Wmo_CollisionFacesFollowTheClient)
{
    CHECK(IsWmoCollisionFace(WMO_FACE_RENDER));                                 // plain wall
    CHECK(IsWmoCollisionFace(WMO_FACE_COLLISION));                              // invisible collision
    CHECK(IsWmoCollisionFace(WMO_FACE_DETAIL | WMO_FACE_COLLISION));            // solid detail (railing)
    CHECK(IsWmoCollisionFace(WMO_FACE_DETAIL | WMO_FACE_COLLISION | WMO_FACE_RENDER));
    CHECK(!IsWmoCollisionFace(WMO_FACE_DETAIL | WMO_FACE_RENDER));              // decoration only
    CHECK(!IsWmoCollisionFace(0x00));                                           // neither: no wall
    CHECK(!IsWmoCollisionFace(0x10));                                           // hint
    CHECK(!IsWmoCollisionFace(0x02));                                           // no camera collision only
}

TEST_CASE(M2_CollisionOfEveryVersion)
{
    std::vector<Vector3> verts;
    std::vector<uint16_t> indices;
    WowData::AddBox(verts, indices, {-1.0f, -0.05f, 0.0f}, {1.0f, 0.05f, 1.5f}); // a fence panel

    for (const auto& [version, chunked] : {std::pair{256u, false}, std::pair{263u, false}, std::pair{264u, false},
                                           std::pair{274u, true}})
    {
        WowData::Bytes data = WowData::MakeM2(verts, indices, version, chunked);
        M2 m2{data.data(), static_cast<unsigned int>(data.size())};
        REQUIRE(m2.IsValid());
        CHECK(m2.IsCollideable());
        CHECK_EQ(m2.Version(), version);
        CHECK_EQ(m2.Collision().countBoundingVertices, static_cast<unsigned int>(verts.size()));
        CHECK_EQ(m2.Collision().countBoundingTriangles, static_cast<unsigned int>(indices.size()));
        CHECK(m2.Vertex(7) == verts[7]);
        CHECK_EQ(m2.Index(5), indices[5]);

        // Reading a classic file with the WotLK layout (or the other way round) would find nonsense.
        data.resize(data.size() - 1);
        M2 truncated{data.data(), static_cast<unsigned int>(data.size())};
        CHECK(!truncated.IsValid() || truncated.Collision().countBoundingNormals > 0);
    }

    // Not an M2, or too short for the header.
    WowData::Bytes junk(16, 0);
    std::memcpy(junk.data(), "MD21", 4);
    const M2 notMd20{junk.data(), static_cast<unsigned int>(junk.size())};
    const M2 tooShort{junk.data(), 6};
    CHECK(!notMd20.IsValid());
    CHECK(!tooShort.IsValid());
}

TEST_CASE(Exporter_GameObjectsBlockTheWay)
{
    REQUIRE(Get().dataOk);
    const auto root = std::filesystem::temp_directory_path() / "anav_tests" / "exporter_gameobjects";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);

    // Fence along WoW x by display id, a door (type 0, left out by default), the fence by model path turned by 90
    // degrees (along WoW y), a line with a bad number.
    const Vector3 fence{PLAIN.x, PLAIN.y, TerrainHeight(PLAIN.x, PLAIN.y)};
    const Vector3 door{PLAIN.x, PLAIN.y - 40.0f, TerrainHeight(PLAIN.x, PLAIN.y - 40.0f)};
    const Vector3 turned{PLAIN.x + 45.0f, PLAIN.y - 80.0f, TerrainHeight(PLAIN.x + 45.0f, PLAIN.y - 80.0f)};
    const auto csv = root / "gameobjects.csv";

    {
        std::ofstream out(csv);
        out << "map,displayId,model,x,y,z,orientation,rotation0,rotation1,rotation2,rotation3,scale,type\n";
        out << std::format("{},{},,{},{},{},0,0,0,0,0,1,5\n", CONTINENT, FENCE_DISPLAY_ID, fence.x, fence.y, fence.z);
        out << std::format("{},{},,{},{},{},0,0,0,0,0,1,0\n", CONTINENT, FENCE_DISPLAY_ID, door.x, door.y, door.z);
        out << std::format("{},,World\\Generic\\Test\\Fence.m2,{},{},{},1.5707964,0,0,0,0,1,5\n", CONTINENT,
                           turned.x, turned.y, turned.z);
        out << std::format("{},{},,abc,0,0,0,0,0,0,0,1,5\n", CONTINENT, FENCE_DISPLAY_ID);
    }

    ExportOptions options = BaseOptions(Get());
    options.outputDir = root / "meshes";
    options.mapIds = {static_cast<int>(CONTINENT)};
    options.gameObjects = csv;
    const ExportReport report = RunExport(options);
    REQUIRE(report.maps.size() == 1);
    CHECK(report.maps[0].exported);
    CHECK_EQ(report.maps[0].gameObjects, size_t{2});
    CHECK_EQ(report.maps[0].gameObjectsMissing, size_t{0});

    AmeisenNavigationSettings settings;
    settings.meshFolder = options.outputDir;
    settings.useAnp = true;
    settings.maxPointPath = 256;
    AmeisenNavigation nav(settings);
    REQUIRE(nav.NewClient(1));

    const auto ground = [](float x, float y) { return Vector3{x, y, TerrainHeight(x, y)}; };
    const auto crossing = [&](const Vector3& a, const Vector3& b, float& length) {
        Path path(256);
        REQUIRE(nav.GetPath(1, static_cast<int>(CONTINENT), a, b, path));
        length = PathLength(path);
        return nav.CastMovementRay(1, static_cast<int>(CONTINENT), a, b);
    };

    float length = 0.0f;

    // Across the fence (WoW y): blocked, the path goes around one of its ends (15 yards away).
    const Vector3 a = ground(fence.x, fence.y + 8.0f);
    const Vector3 b = ground(fence.x, fence.y - 8.0f);
    CHECK(!crossing(a, b, length));
    CHECK(length > a.DistanceTo(b) + 15.0f);

    // The door isn't there: straight through.
    const Vector3 c = ground(door.x, door.y + 8.0f);
    const Vector3 d = ground(door.x, door.y - 8.0f);
    CHECK(crossing(c, d, length));
    CHECK(length < c.DistanceTo(d) + 1.0f);

    // The turned fence lies along WoW y: crossing it along x is blocked, along y next to it is free.
    const Vector3 e = ground(turned.x + 8.0f, turned.y);
    const Vector3 f = ground(turned.x - 8.0f, turned.y);
    CHECK(!crossing(e, f, length));
    CHECK(length > e.DistanceTo(f) + 15.0f);
    CHECK(crossing(ground(turned.x + 3.0f, turned.y + 10.0f), ground(turned.x + 3.0f, turned.y - 10.0f), length));
}

TEST_CASE(GameObjects_SpawnFileFormats)
{
    const auto dir = std::filesystem::temp_directory_path() / "anav_tests" / "gameobject_files";
    std::filesystem::create_directories(dir);

    // Positional columns (no header), tab separated with a header (mysql -B output), comments, bad lines.
    {
        std::ofstream out(dir / "positional.csv");
        out << "# comment\n0,123,1.5,2.5,3.5\n1,7,10,20,30,3.14,0,0,0.7071,0.7071,2,5\n0,1,nan,0,0\n";
    }
    {
        std::ofstream out(dir / "tabs.tsv");
        out << "map\tdisplayId\tposition_x\tposition_y\tposition_z\torientation\tsize\ttype\n";
        out << "530\t42\t-100\t200\t30\t1\t1.5\t0\n";
        out << "530\t\t-100\t200\t30\t1\t1.5\t0\n"; // no display id and no model
    }

    size_t invalid = 0;
    const auto positional = GameObjects::ReadSpawns(dir / "positional.csv", &invalid);
    REQUIRE(positional.size() == 2);
    CHECK_EQ(invalid, size_t{1});
    CHECK_EQ(positional[0].displayId, 123u);
    CHECK_NEAR(positional[0].position.z, 3.5f, 1e-6);
    CHECK_NEAR(positional[0].scale, 1.0f, 1e-6);
    CHECK_EQ(positional[1].type, 5);
    CHECK_NEAR(positional[1].rotation[3], 0.7071f, 1e-6);

    const auto tabs = GameObjects::ReadSpawns(dir / "tabs.tsv", &invalid);
    REQUIRE(tabs.size() == 1);
    CHECK_EQ(invalid, size_t{1});
    CHECK_EQ(tabs[0].mapId, 530);
    CHECK_EQ(tabs[0].displayId, 42u);
    CHECK_NEAR(tabs[0].scale, 1.5f, 1e-6);
    CHECK_EQ(tabs[0].type, 0);

    // Orientation and quaternion give the same transform.
    GameObjectSpawn byYaw;
    byYaw.orientation = 1.0f;
    GameObjectSpawn byQuat;
    byQuat.rotation[2] = std::sin(0.5f);
    byQuat.rotation[3] = std::cos(0.5f);
    const Vector3 local{3.0f, 1.0f, 2.0f};
    const Vector3 p = GameObjects::SpawnTransform(byYaw).Transform(local);
    const Vector3 q = GameObjects::SpawnTransform(byQuat).Transform(local);
    CHECK(p.DistanceTo(q) < 1e-4f);

    // Turning by 90 degrees maps the model's x axis onto WoW y.
    GameObjectSpawn quarter;
    quarter.orientation = 1.5707964f;
    const Vector3 turned = GameObjects::SpawnTransform(quarter).Transform({1.0f, 0.0f, 0.0f});
    CHECK_NEAR(turned.x, 0.0f, 1e-5);
    CHECK_NEAR(turned.y, 1.0f, 1e-5);
}

