#include "TestFramework.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <mutex>

#include "AmeisenNavigation.hpp"
#include "Dbc/Db2Header.hpp"
#include "Exporter.hpp"
#include "TcDataBuilder.hpp"
#include "TestWorld.hpp"
#include "Wow/TcData.hpp"
#include "WowDataBuilder.hpp"

// Exporter input from TrinityCore's extractors (recent clients): synthetic maps/*.map and vmaps/ files written like
// TrinityCore master's mapextractor and vmap4assembler do, exported and queried.
//
// Map 700: .map tiles for ADT (32,32) (float heights, sloped, a hole, city and faction areas) and ADT (33,32)
// (heights packed as uint8, a water pool, a deep ocean cell). vmaps: a WMO fence turned by 90 degrees, a WMO with a
// liquid surface, an M2 platform. A game object fence from GameObjectModels.dtree.
namespace {
using namespace WowData;
using namespace TcData;
using namespace TcDataBuilder;

constexpr int MAP = 700;
constexpr float T = TILESIZE;

float SlopedHeight(float wowX, float wowY) { return 2.0f + 0.01f * wowX + 0.005f * wowY; }
float PackedHeight(float wowX, float wowY)
{
    return 5.0f + 0.75f * std::sin(wowX * 0.01f) + 0.75f * std::cos(wowY * 0.01f);
}

Vector3 CellCenter(int adtX, int adtY, int cellRow, int cellColumn)
{
    return {RowX(adtY, cellRow * 8.0f + 4.0f), ColumnY(adtX, cellColumn * 8.0f + 4.0f), 0.0f};
}

const Vector3 HOLE = CellCenter(32, 32, 4, 10);
const Vector3 DISTRICT = CellCenter(32, 32, 8, 4); // Alliance capital's sub area
const Vector3 HORDE = CellCenter(32, 32, 12, 8);
const Vector3 WATER = CellCenter(33, 32, 5, 5);                                      // pool over cells 4..6 x 4..6
const Vector3 DEEP = CellCenter(33, 32, 10, 10);                                     // deep ocean: nothing
const Vector3 FENCE{-150.0f, -300.0f, SlopedHeight(-150.0f, -300.0f)};               // vmap WMO, along WoW y
const Vector3 OBJECT{-400.0f, -100.0f, SlopedHeight(-400.0f, -100.0f)};              // game object, along WoW x
const Vector3 PLATFORM{-150.0f, -T - 150.0f, PackedHeight(-150.0f, -T - 150.0f)};    // M2, 4 yards high
const Vector3 POOL{-450.0f, -T - 400.0f, PackedHeight(-450.0f, -T - 400.0f) + 3.0f}; // WMO liquid surface

constexpr uint32_t OBJECT_DISPLAY_ID = 900;
constexpr uint32_t POOL_LIQUID_TYPE = 77; // slime in the LiquidType CSV

bool WriteBytes(const std::filesystem::path& path, const Bytes& bytes)
{
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    return static_cast<bool>(out);
}

struct Fixture
{
    std::filesystem::path dataDir;
    std::filesystem::path outputDir;
    ExportReport report;
};

constexpr float FENCE_HALF_LENGTH = 15.0f;
constexpr float FENCE_HEIGHT = 1.4f;

const Fixture& Get()
{
    static std::once_flag once;
    static Fixture f;

    std::call_once(once, [] {
        const auto root = std::filesystem::temp_directory_path() / "anav_tests" / "tcdata";
        std::filesystem::remove_all(root);
        f.dataDir = root / "data";
        f.outputDir = root / "meshes";

        // Holes in cell (4,10), areas 2 (district of the Alliance capital) and 4 (Horde).
        REQUIRE(WriteBytes(TileFile(f.dataDir, MAP, 32, 32, false),
                           MakeMapFile(32, 32, SlopedHeight, false, {}, {0, 0}, {{8, 4, 2}, {12, 8, 4}}, {4, 10})));
        std::vector<std::pair<int, int>> pool;

        for (int r = 4; r <= 6; ++r)
            for (int c = 4; c <= 6; ++c)
                pool.emplace_back(r, c);

        // The hole cell of this tile is outside everything the tests look at.
        REQUIRE(WriteBytes(TileFile(f.dataDir, MAP, 33, 32, false),
                           MakeMapFile(33, 32, PackedHeight, true, pool, {10, 10}, {}, {15, 15})));

        REQUIRE(WriteBytes(f.dataDir / "vmaps" / "fence.vmo",
                           MakeVmo(false, {Box({-FENCE_HALF_LENGTH, -0.15f, -0.3f},
                                               {FENCE_HALF_LENGTH, 0.15f, FENCE_HEIGHT}, false)})));
        REQUIRE(WriteBytes(f.dataDir / "vmaps" / "platform.vmo",
                           MakeVmo(true, {Box({-6.0f, -6.0f, -1.0f}, {6.0f, 6.0f, 4.0f}, true)})));

        WorldModel::Liquid liquid;
        liquid.tilesX = 8;
        liquid.tilesY = 8;
        liquid.corner[0] = liquid.corner[1] = -4.0f * UNITSIZE;
        liquid.type = POOL_LIQUID_TYPE;
        liquid.heights.assign(81, 0.0f);
        liquid.flags.assign(64, 0);
        REQUIRE(WriteBytes(f.dataDir / "vmaps" / "pool.vmo",
                           MakeVmo(false, {VmoGroup{{}, {}, std::nullopt},
                                           VmoGroup{{{0, 0, -50}, {1, 0, -50}, {0, 1, -50}}, {0, 1, 2}, liquid}})));

        // The fence is listed in both tiles (it'd overlap them), it must be added once.
        const Bytes fence = MakeSpawn(1, FENCE, {0.0f, 90.0f, 0.0f}, "fence.vmo");
        REQUIRE(WriteBytes(TileFile(f.dataDir, MAP, 32, 32, true),
                           MakeVmtile({fence, MakeSpawn(2, POOL, {}, "pool.vmo")})));
        REQUIRE(WriteBytes(TileFile(f.dataDir, MAP, 33, 32, true),
                           MakeVmtile({fence, MakeSpawn(3, PLATFORM, {}, "platform.vmo"),
                                       MakeSpawn(4, PLATFORM, {}, "missing.vmo")})));

        REQUIRE(WriteBytes(f.dataDir / "vmaps" / "GameObjectModels.dtree",
                           MakeGameObjectModels(OBJECT_DISPLAY_ID, "fence.vmo")));

        {
            std::ofstream out(root / "gameobjects.csv");
            out << "map,displayId,x,y,z,orientation\n";
            out << std::format("{},{},{},{},{},0\n", MAP, OBJECT_DISPLAY_ID, OBJECT.x, OBJECT.y, OBJECT.z);
            out << std::format("{},{},{},{},{},0\n", MAP, 12345, OBJECT.x, OBJECT.y - 60.0f, OBJECT.z);
        }

        {
            // wago.tools style exports: quoted names with separators, more columns than needed.
            std::ofstream out(root / "AreaTable.csv");
            out << "ID,ZoneName,AreaName_lang,ContinentID,ParentAreaID,Flags[0],Flags[1],FactionGroupMask\n";
            out << "1,Capital,\"Capital, the \"\"Big\"\" One\",700,0,8,0,2\n";
            out << "2,District,\"Trade District\",700,1,0,0,0\n";
            out << "4,HordeZone,\"Horde Land\",700,0,0,0,4\n";
        }

        {
            std::ofstream out(root / "LiquidType.csv");
            out << "ID,Name,SoundBank,MaterialID\n";
            out << std::format("{},\"Green, Bubbly\",3,1\n", POOL_LIQUID_TYPE);
        }

        ExportOptions options;
        options.tcDataDir = f.dataDir;
        options.outputDir = f.outputDir;
        options.areaTableCsv = root / "AreaTable.csv";
        options.liquidTypeCsv = root / "LiquidType.csv";
        options.gameObjects = root / "gameobjects.csv";
        options.buildConfig = TestWorld::FastBuildConfig();
        options.buildConfig.meshResolution = 1024;
        f.report = RunExport(options);
    });

    return f;
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
    static const bool client = nav.NewClient(1);
    (void)client;
    return nav;
}

/// Area of the polygon closest to a WoW position, 255 if there is none.
unsigned char AreaAt(const Vector3& wow, float verticalExtent = 1.5f)
{
    static const Anp::LoadResult mesh = Anp::Load(Get().outputDir / Anp::FileName(MAP));
    REQUIRE(mesh.navMesh);
    NavMeshQueryPtr query(dtAllocNavMeshQuery());
    REQUIRE(dtStatusSucceed(query->init(mesh.navMesh.get(), 2048)));

    dtQueryFilter filter;
    const float pos[3]{wow.y, wow.z, wow.x};
    const float extents[3]{0.5f, verticalExtent, 0.5f};
    dtPolyRef ref = 0;
    float nearest[3]{};

    if (!dtStatusSucceed(query->findNearestPoly(pos, extents, &filter, &ref, nearest)) || !ref)
    {
        return 255;
    }

    const dtMeshTile* tile = nullptr;
    const dtPoly* poly = nullptr;
    REQUIRE(dtStatusSucceed(mesh.navMesh->getTileAndPolyByRef(ref, &tile, &poly)));
    return poly->getArea();
}

float HeightAt(float wowX, float wowY, float probeZ)
{
    Vector3 out;
    return Navigation().GetHeight(1, MAP, {wowX, wowY, probeZ}, out) ? out.z : NAN;
}

Vector3 Ground(float x, float y) { return {x, y, SlopedHeight(x, y)}; }

/// Path length from a to b and whether the straight line is clear.
bool Crossing(const Vector3& a, const Vector3& b, float& length)
{
    Path path(256);
    REQUIRE(Navigation().GetPath(1, MAP, a, b, path));
    length = 0.0f;

    for (int i = 1; i < path.pointCount; ++i)
    {
        length += path[i].DistanceTo(path[i - 1]);
    }

    return Navigation().CastMovementRay(1, MAP, a, b);
}
} // namespace

TEST_CASE(TcData_ExportsTheMap)
{
    const auto& report = Get().report;
    REQUIRE(report.setupOk);
    REQUIRE(report.maps.size() == 1);
    const MapExportStats& stats = report.maps[0];
    CHECK(stats.exported);
    CHECK(!stats.failed);
    CHECK_EQ(stats.adts, 2);
    CHECK_EQ(stats.wmoPlacements, size_t{2});    // fence once, pool
    CHECK_EQ(stats.doodadPlacements, size_t{1}); // platform, missing.vmo skipped
    CHECK_EQ(stats.gameObjects, size_t{1});
    CHECK_EQ(stats.gameObjectsMissing, size_t{1});
    CHECK(stats.waterRects > 0);
    CHECK(stats.cityRects > 0);
    CHECK(stats.factionRects > 0);

    const auto maps = ListTcMaps(Get().dataDir);
    REQUIRE(maps.size() == 1);
    CHECK_EQ(maps[0].first, static_cast<unsigned int>(MAP));
}

TEST_CASE(TcData_TerrainHeightsAndHoles)
{
    REQUIRE(Get().report.setupOk);
    const float ch = TILESIZE / 1024.0f;

    // Float heights.
    for (const auto& [x, y] : {std::pair{-100.0f, -200.0f}, {-300.0f, -450.0f}, {-480.0f, -20.0f}})
    {
        const float h = HeightAt(x, y, SlopedHeight(x, y) + 1.0f);
        CHECK(h >= SlopedHeight(x, y) - 0.25f && h <= SlopedHeight(x, y) + 2.0f * ch + 0.1f);
    }

    // uint8 packed heights (0.01 yard steps here).
    for (const auto& [x, y] : {std::pair{-100.0f, -T - 300.0f}, {-350.0f, -T - 60.0f}})
    {
        const float h = HeightAt(x, y, PackedHeight(x, y) + 1.0f);
        CHECK(h >= PackedHeight(x, y) - 0.25f && h <= PackedHeight(x, y) + 2.0f * ch + 0.1f);
    }

    // The hole cell has no navmesh in its middle.
    CHECK_EQ(AreaAt({HOLE.x, HOLE.y, SlopedHeight(HOLE.x, HOLE.y)}, 3.0f), 255);
    CHECK(AreaAt({HOLE.x + 20.0f, HOLE.y, SlopedHeight(HOLE.x + 20.0f, HOLE.y)}) != 255);
}

TEST_CASE(TcData_AreasFromTheCsvExports)
{
    REQUIRE(Get().report.setupOk);
    CHECK_EQ(AreaAt({DISTRICT.x, DISTRICT.y, SlopedHeight(DISTRICT.x, DISTRICT.y)}), ALLIANCE_TERRAIN_CITY);
    CHECK_EQ(AreaAt({HORDE.x, HORDE.y, SlopedHeight(HORDE.x, HORDE.y)}), HORDE_TERRAIN_GROUND);
}

TEST_CASE(TcData_LiquidsAndDeepOcean)
{
    REQUIRE(Get().report.setupOk);

    // The pool surface at 8 (ground ~5): water.
    CHECK_EQ(AreaAt({WATER.x, WATER.y, 8.0f}), LIQUID_WATER);

    // Deep ocean: no terrain and no surface.
    CHECK_EQ(AreaAt({DEEP.x, DEEP.y, PackedHeight(DEEP.x, DEEP.y)}, 3.0f), 255);

    // WMO liquid with a type from LiquidType.csv (slime) 3 yards above the ground.
    CHECK_EQ(AreaAt(POOL), LIQUID_SLIME);
}

TEST_CASE(TcData_ModelsAreTransformedLikeTrinityCore)
{
    REQUIRE(Get().report.setupOk);
    float length = 0.0f;

    // The WMO fence is turned by 90 degrees: along WoW y. Crossing it along x goes around an end.
    const Vector3 a = Ground(FENCE.x + 8.0f, FENCE.y);
    const Vector3 b = Ground(FENCE.x - 8.0f, FENCE.y);
    CHECK(!Crossing(a, b, length));
    CHECK(length > a.DistanceTo(b) + 15.0f);
    CHECK(Crossing(Ground(FENCE.x + 3.0f, FENCE.y + 10.0f), Ground(FENCE.x + 3.0f, FENCE.y - 10.0f), length));

    // The game object fence (orientation 0) lies along WoW x.
    const Vector3 c = Ground(OBJECT.x, OBJECT.y + 8.0f);
    const Vector3 d = Ground(OBJECT.x, OBJECT.y - 8.0f);
    CHECK(!Crossing(c, d, length));
    CHECK(length > c.DistanceTo(d) + 15.0f);

    // The M2 platform's top is walkable: TrinityCore's swapped M2 indices were turned back.
    const float top = PLATFORM.z + 4.0f;
    const float h = HeightAt(PLATFORM.x, PLATFORM.y, top + 1.0f);
    CHECK(h >= top - 0.25f && h <= top + 1.0f);
}

TEST_CASE(TcData_RejectsBrokenFiles)
{
    // Old version and truncated files are rejected, no prefix of a valid file crashes the parsers.
    const Bytes map = MakeMapFile(32, 32, PackedHeight, true, {{1, 1}}, {2, 2}, {{0, 0, 1}}, {3, 3});
    REQUIRE(ParseMapTile(map, 32, 32).has_value());

    Bytes old = map;
    old[4] = 9;
    CHECK(!ParseMapTile(old, 32, 32).has_value());

    for (size_t size = 0; size < map.size(); size += 97)
    {
        CHECK(!ParseMapTile(Bytes(map.begin(), map.begin() + static_cast<std::ptrdiff_t>(size)), 32, 32).has_value());
    }

    WorldModel::Liquid liquid;
    liquid.tilesX = 2;
    liquid.tilesY = 1;
    liquid.heights.assign(6, 1.0f);
    liquid.flags.assign(2, 0);
    const Bytes vmo = MakeVmo(true, {Box({0, 0, 0}, {1, 1, 1}, true), VmoGroup{{{0, 0, 0}}, {0, 0, 0}, liquid}});
    const auto model = ParseWorldModel(vmo);
    REQUIRE(model.has_value());
    CHECK(model->isM2);
    REQUIRE(model->groups.size() == 2);
    CHECK_EQ(model->groups[0].indices.size(), size_t{36});
    CHECK(model->groups[1].liquid.has_value());

    // (The group BIH at the end isn't read.)
    for (size_t size = 0; size + 36 < vmo.size(); ++size)
    {
        const Bytes prefix(vmo.begin(), vmo.begin() + static_cast<std::ptrdiff_t>(size));
        const auto parsed = ParseWorldModel(prefix);
        CHECK(!parsed || parsed->groups.size() < 2);
    }

    const Bytes tile = MakeVmtile({MakeSpawn(1, {}, {}, "a.vmo"), MakeSpawn(2, {}, {}, "b.vmo")});
    CHECK_EQ(ParseTileSpawns(tile).size(), size_t{2});

    for (size_t size = 0; size < tile.size(); ++size)
    {
        CHECK(ParseTileSpawns(Bytes(tile.begin(), tile.begin() + static_cast<std::ptrdiff_t>(size))).size() < 2);
    }

    // File names: map, then the tile's row (ADT y) and column (ADT x).
    const auto name = ParseTileFileName("0700_31_33.map", ".map");
    REQUIRE(name.has_value());
    CHECK_EQ((*name)[0], 700);
    CHECK_EQ((*name)[1], 33);
    CHECK_EQ((*name)[2], 31);
    CHECK(!ParseTileFileName("0700_31_33.mmtile", ".map").has_value());
    CHECK(!ParseTileFileName("0700_31_99.map", ".map").has_value());
}

TEST_CASE(TcData_LiquidTypeIds)
{
    const std::unordered_map<unsigned int, LiquidType> none;
    CHECK_EQ(LiquidTypeIdToArea(1, none), LIQUID_WATER);
    CHECK_EQ(LiquidTypeIdToArea(14, none), LIQUID_OCEAN); // WMO ocean
    CHECK_EQ(LiquidTypeIdToArea(19, none), LIQUID_LAVA);  // WMO magma
    CHECK_EQ(LiquidTypeIdToArea(20, none), LIQUID_SLIME); // WMO slime
    CHECK_EQ(LiquidTypeIdToArea(500, none), LIQUID_WATER);
    const std::unordered_map<unsigned int, LiquidType> table{{500, LiquidType::MAGMA}, {1, LiquidType::OCEAN}};
    CHECK_EQ(LiquidTypeIdToArea(500, table), LIQUID_LAVA);
    CHECK_EQ(LiquidTypeIdToArea(1, table), LIQUID_OCEAN);
}

TEST_CASE(Db2_HeaderAndExtractorLayouts)
{
    // WDC5: magic, version, schema string, then the WDC4 fields.
    Bytes wdc5{'W', 'D', 'C', '5'};
    PutRaw(wdc5, uint32_t{5});
    wdc5.resize(wdc5.size() + 128, 0);
    const uint32_t fields[9]{1013, 26, 120, 5000, 0xE0A6E2B1, 0xD43AFAC3, 0, 2900, 0};
    PutRaw(wdc5, fields);
    PutRaw(wdc5, uint16_t{4});
    PutRaw(wdc5, uint16_t{0});
    const uint32_t rest[7]{28, 0, 0, 0, 0, 0, 1};
    PutRaw(wdc5, rest);

    const auto header = Db2::ParseHeader(wdc5.data(), wdc5.size());
    REQUIRE(header.has_value());
    CHECK(header->format == "WDC5");
    CHECK_EQ(header->recordCount, 1013u);
    CHECK_EQ(header->fieldCount, 26u);
    CHECK_EQ(header->layoutHash, 0xD43AFAC3u);
    CHECK_EQ(header->totalFieldCount, 28u);
    CHECK_EQ(header->sectionCount, 1u);
    CHECK(!Db2::ParseHeader(wdc5.data(), wdc5.size() - 1).has_value());

    // WDC3: no version or schema.
    Bytes wdc3{'W', 'D', 'C', '3'};
    PutBytes(wdc3, Bytes(wdc5.begin() + 4 + 4 + 128, wdc5.end()));
    const auto old = Db2::ParseHeader(wdc3.data(), wdc3.size());
    REQUIRE(old.has_value());
    CHECK_EQ(old->layoutHash, 0xD43AFAC3u);
    const Bytes wdbc{'W', 'D', 'B', 'C', 0, 0, 0, 0};
    CHECK(!Db2::ParseHeader(wdbc.data(), wdbc.size()).has_value());

    // TrinityCore master's designated initializers, an older positional one.
    const std::string loadInfo = R"(
struct MapLoadInfo
{
    static constexpr DB2MetaField MetaFields[25] = { { .Type = FT_STRING, .ArraySize = 1, .IsSigned = true } };
    static constexpr DB2Meta MetaInstance =
    {
        .FileDataId         = 1349477,
        .IndexField         = -1,
        .ParentIndexField   = -1,
        .FieldCount         = 25,
        .FileFieldCount     = 25,
        .LayoutHash         = 0x75863E23,
        .Fields             = MetaFields
    };
};

struct LiquidTypeLoadInfo
{
    static DB2Meta const* Instance() { static DB2Meta instance(1371380, -1, 18, 18, 0xCD5D7F2B, fields, -1); }
};
)";
    const auto layouts = Db2::ParseLoadInfo(loadInfo);
    REQUIRE(layouts.size() == 2);
    const auto& map = layouts.at(Db2::TableKey("Map.db2"));
    CHECK_EQ(map.layoutHash, 0x75863E23u);
    CHECK_EQ(map.fileFieldCount, 25);
    CHECK_EQ(layouts.at(Db2::TableKey("DBFilesClient/LiquidType.db2")).layoutHash, 0xCD5D7F2Bu);
    CHECK(Db2::TableKey("GameobjectDisplayInfoLoadInfo") == Db2::TableKey("GameObjectDisplayInfo.db2"));
}
