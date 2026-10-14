#include "TestFramework.hpp"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>

#include "AmeisenNavigation.hpp"
#include "NavMeshPly.hpp"
#include "NavSources/OffMesh/TileRebuilder.hpp"
#include "TestWorld.hpp"

namespace {
using TestWorld::Wow;

/// The test world's tiles as a TrinityCore master MMAP folder (MMAP v16: .mmap with a header, maxPolys written as
/// 1u << DT_POLY_BITS like the real generator).
std::filesystem::path WriteTcMasterMmaps(const char* name, int mapId)
{
    const auto dir = std::filesystem::temp_directory_path() / "anav_tests" / name;
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);

    const auto loaded = Anp::Load(TestWorld::Get().meshDir / Anp::FileName(TestWorld::MAP_ID));
    REQUIRE(loaded.navMesh);

    MmapNavMeshHeader header{};
    header.mmapMagic = MMAP_MAGIC;
    header.mmapVersion = MMAP_VERSION_NAVMESH_HEADER;
    header.params = *loaded.navMesh->getParams();
    header.params.maxPolys = static_cast<int>(1u << DT_POLY_BITS);
    header.offmeshConnectionCount = 1;

    MmapOffMeshData offMesh{};
    offMesh.mapId = static_cast<uint32_t>(mapId);
    offMesh.connectionFlags = 1;

    {
        std::ofstream out(dir / std::format("{:04}.mmap", mapId), std::ios::binary);
        out.write(reinterpret_cast<const char*>(&header), sizeof(header));
        out.write(reinterpret_cast<const char*>(&offMesh), sizeof(offMesh));
    }

    const dtNavMesh& mesh = *loaded.navMesh;

    for (int i = 0; i < mesh.getMaxTiles(); ++i)
    {
        const dtMeshTile* tile = mesh.getTile(i);

        if (!tile || !tile->header)
        {
            continue;
        }

        MmapTileHeader tileHeader{};
        tileHeader.mmapMagic = MMAP_MAGIC;
        tileHeader.dtVersion = DT_NAVMESH_VERSION;
        tileHeader.mmapVersion = MMAP_VERSION_NAVMESH_HEADER;
        tileHeader.size = static_cast<uint32_t>(tile->dataSize);
        tileHeader.usesLiquids = 1;

        std::ofstream out(dir / std::format("{:04}_{:02}_{:02}.mmtile", mapId, tile->header->x, tile->header->y),
                          std::ios::binary);
        out.write(reinterpret_cast<const char*>(&tileHeader), sizeof(tileHeader));
        out.write(reinterpret_cast<const char*>(tile->data), tile->dataSize);
    }

    return dir;
}
} // namespace

TEST_CASE(Mmap_TrinityCoreMasterIsDetectedAndLoaded)
{
    const auto dir = WriteTcMasterMmaps("mmap_tc_master", TestWorld::MAP_ID);
    CHECK(MmapNavSource::DetectFormat(dir) == MmapFormat::TC_MASTER);

    AmeisenNavigationSettings settings;
    settings.meshFolder = dir;
    AmeisenNavigation nav(settings);
    CHECK(nav.GetMmapFormat() == MmapFormat::TC_MASTER);
    REQUIRE(nav.NewClient(1));

    // Same path as on the ANP world: across both tiles and around the wall.
    Path path(256);
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, Wow(-700.0f, 0.0f, -300.0f), Wow(-200.0f, 0.0f, -300.0f), path));
    CHECK(path.pointCount >= 3);
}

TEST_CASE(Mmap_ChildMapsUseTheParentsFiles)
{
    const auto dir = WriteTcMasterMmaps("mmap_parent", TestWorld::MAP_ID);

    AmeisenNavigationSettings settings;
    settings.meshFolder = dir;
    settings.mmapFormat = MmapFormat::TC_MASTER;
    settings.parentMaps = {{7, 5}, {5, TestWorld::MAP_ID}};
    AmeisenNavigation nav(settings);
    REQUIRE(nav.NewClient(1));

    Path path(256);
    CHECK(nav.GetPath(1, 7, Wow(-700.0f, 0.0f, -300.0f), Wow(-200.0f, 0.0f, -300.0f), path));
    CHECK(!nav.GetPath(1, 8, Wow(-700.0f, 0.0f, -300.0f), Wow(-200.0f, 0.0f, -300.0f), path));
}

TEST_CASE(Mmap_ReadsBothParamsLayouts)
{
    const auto dir = std::filesystem::temp_directory_path() / "anav_tests" / "mmap_params";
    std::filesystem::create_directories(dir);

    dtNavMeshParams params{};
    params.orig[0] = -1.0f;
    params.tileWidth = params.tileHeight = 533.33333f;
    params.maxTiles = 4096;
    params.maxPolys = 1 << 20;

    {
        std::ofstream out(dir / "raw.mmap", std::ios::binary);
        out.write(reinterpret_cast<const char*>(&params), sizeof(params));
    }

    MmapNavMeshHeader header{MMAP_MAGIC, 16, params, 0};
    {
        std::ofstream out(dir / "v16.mmap", std::ios::binary);
        out.write(reinterpret_cast<const char*>(&header), sizeof(header));
    }

    header.mmapVersion = 15;
    {
        std::ofstream out(dir / "old.mmap", std::ios::binary);
        out.write(reinterpret_cast<const char*>(&header), sizeof(header));
    }

    {
        std::ofstream out(dir / "short.mmap", std::ios::binary);
        out.write(reinterpret_cast<const char*>(&header), 12);
    }

    dtNavMeshParams read{};
    CHECK(MmapNavSource::ReadNavMeshParams(dir / "raw.mmap", read));
    CHECK(std::memcmp(&read, &params, sizeof(params)) == 0);

    read = {};
    CHECK(MmapNavSource::ReadNavMeshParams(dir / "v16.mmap", read));
    CHECK(std::memcmp(&read, &params, sizeof(params)) == 0);

    CHECK(!MmapNavSource::ReadNavMeshParams(dir / "old.mmap", read));
    CHECK(!MmapNavSource::ReadNavMeshParams(dir / "short.mmap", read));
    CHECK(!MmapNavSource::ReadNavMeshParams(dir / "missing.mmap", read));

    // maxPolys is unused with 64 bit poly refs, TrinityCore master writes a negative int.
    params.maxPolys = static_cast<int>(1u << 31);
    CHECK(ValidateNavMeshParams(params));
    params.maxPolys = 0;
    CHECK(!ValidateNavMeshParams(params));
}

namespace {
NavMeshPtr LoadWorld()
{
    auto loaded = Anp::Load(TestWorld::Get().meshDir / Anp::FileName(TestWorld::MAP_ID));
    REQUIRE(loaded.navMesh);
    return std::move(loaded.navMesh);
}

/// Path length between two RD positions with a plain filter, -1 if there is no complete path.
float RdPathLength(const dtNavMesh& mesh, const Vector3& start, const Vector3& end, int* offMeshCorners = nullptr)
{
    NavMeshQueryPtr query(dtAllocNavMeshQuery());
    REQUIRE(dtStatusSucceed(query->init(&mesh, 4096)));
    dtQueryFilter filter;
    const float extents[3]{2.0f, 4.0f, 2.0f};
    dtPolyRef startRef = 0;
    dtPolyRef endRef = 0;
    Vector3 s;
    Vector3 e;
    REQUIRE(dtStatusSucceed(query->findNearestPoly(start, extents, &filter, &startRef, s)) && startRef);
    REQUIRE(dtStatusSucceed(query->findNearestPoly(end, extents, &filter, &endRef, e)) && endRef);

    dtPolyRef polys[1024];
    int polyCount = 0;
    const dtStatus status = query->findPath(startRef, endRef, s, e, &filter, polys, &polyCount, 1024);

    if (dtStatusFailed(status) || polyCount == 0 || polys[polyCount - 1] != endRef)
    {
        return -1.0f;
    }

    float points[256 * 3];
    unsigned char flags[256];
    int pointCount = 0;
    REQUIRE(dtStatusSucceed(query->findStraightPath(s, e, polys, polyCount, points, flags, nullptr, &pointCount, 256)));

    float length = 0.0f;

    for (int i = 1; i < pointCount; ++i)
    {
        length += dtVdist(&points[(i - 1) * 3], &points[i * 3]);
    }

    if (offMeshCorners)
    {
        *offMeshCorners = 0;

        for (int i = 0; i < pointCount; ++i)
        {
            *offMeshCorners += (flags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? 1 : 0;
        }
    }

    return length;
}

int GroundPolyCount(const dtNavMesh& mesh)
{
    int count = 0;

    for (int i = 0; i < mesh.getMaxTiles(); ++i)
    {
        const dtMeshTile* tile = mesh.getTile(i);
        count += tile && tile->header ? tile->header->offMeshBase : 0;
    }

    return count;
}
} // namespace

TEST_CASE(OffMesh_RebuildWithoutLinksKeepsTheTile)
{
    NavMeshPtr mesh = LoadWorld();
    const int polys = GroundPolyCount(*mesh);
    const Vector3 start{-700.0f, 0.0f, -300.0f};
    const Vector3 end{-200.0f, 0.0f, -300.0f};
    const float before = RdPathLength(*mesh, start, end);
    REQUIRE(before > 0.0f);

    for (int i = 0; i < mesh->getMaxTiles(); ++i)
    {
        const dtMeshTile* tile = mesh->getTile(i);

        if (tile && tile->header)
        {
            const dtTileRef ref = mesh->getTileRef(tile);
            REQUIRE(TileRebuilder::Rebuild(*mesh, tile, {}));
            CHECK_EQ(mesh->getTileRef(mesh->getTile(i)), ref); // same index and salt
            CHECK(ValidateTileData(mesh->getTile(i)->data, static_cast<size_t>(mesh->getTile(i)->dataSize)));
        }
    }

    CHECK_EQ(GroundPolyCount(*mesh), polys);
    CHECK_NEAR(RdPathLength(*mesh, start, end), before, 0.05);

    // Heights survive the requantization.
    NavMeshQueryPtr query(dtAllocNavMeshQuery());
    REQUIRE(dtStatusSucceed(query->init(mesh.get(), 256)));
    dtQueryFilter filter;
    const float platform[3]{-425.0f, TestWorld::PLATFORM_Y, -425.0f};
    const float extents[3]{1.0f, 1.0f, 1.0f};
    dtPolyRef ref = 0;
    float nearest[3]{};
    REQUIRE(dtStatusSucceed(query->findNearestPoly(platform, extents, &filter, &ref, nearest)) && ref);
    CHECK(nearest[1] >= TestWorld::PLATFORM_Y - 0.01f && nearest[1] < TestWorld::PLATFORM_Y + 1.0f);
}

TEST_CASE(OffMesh_LinkShortcutsTheWall)
{
    NavMeshPtr mesh = LoadWorld();
    const Vector3 start{-700.0f, 0.0f, -300.0f};
    const Vector3 end{-200.0f, 0.0f, -300.0f};
    const float detour = RdPathLength(*mesh, start, end);
    REQUIRE(detour > 550.0f);

    OffMeshLink link;
    link.start = {TestWorld::WALL_X - 3.0f, 0.0f, -300.0f};
    link.end = {TestWorld::WALL_X + 4.0f, 0.0f, -300.0f};
    link.radius = 1.5f;
    link.area = TERRAIN_GROUND;
    link.flags = NAV_GROUND;
    link.userId = static_cast<unsigned int>(OffMeshLinkKind::JUMP);

    CHECK_EQ(TileRebuilder::AddOffMeshLinks(*mesh, std::span(&link, 1)), 1);

    int offMeshCorners = 0;
    const float shortcut = RdPathLength(*mesh, start, end, &offMeshCorners);
    CHECK(shortcut > 0.0f && shortcut < 510.0f);
    CHECK_EQ(offMeshCorners, 1);

    // One way: the way back still goes around.
    CHECK(RdPathLength(*mesh, end, start) > 550.0f);
}

TEST_CASE(OffMesh_LinkAcrossTilesIsConnectedOnBothEnds)
{
    NavMeshPtr mesh = LoadWorld();

    // Tile border at x = -TILESIZE: start in the western tile, end in the eastern one.
    OffMeshLink link;
    link.start = {-TILESIZE - 3.0f, 0.0f, -200.0f};
    link.end = {-TILESIZE + 3.0f, 0.0f, -200.0f};
    link.radius = 1.0f;
    link.bidirectional = true;
    link.area = TERRAIN_GROUND;
    link.flags = NAV_GROUND;
    REQUIRE(TileRebuilder::AddOffMeshLinks(*mesh, std::span(&link, 1)) == 2);

    const dtMeshTile* startTile = nullptr;

    for (int i = 0; i < mesh->getMaxTiles(); ++i)
    {
        const dtMeshTile* tile = mesh->getTile(i);

        if (tile && tile->header && tile->header->offMeshConCount == 1)
        {
            startTile = tile;
        }
    }

    REQUIRE(startTile != nullptr);
    const dtPoly& poly = startTile->polys[startTile->offMeshCons[0].poly];
    int links = 0;

    for (unsigned int l = poly.firstLink; l != DT_NULL_LINK; l = startTile->links[l].next)
    {
        links++;
    }

    CHECK_EQ(links, 2); // start and end landing polygons

    // Links outside the navmesh are ignored.
    OffMeshLink outside = link;
    outside.start = {5000.0f, 0.0f, 5000.0f};
    CHECK_EQ(TileRebuilder::AddOffMeshLinks(*mesh, std::span(&outside, 1)), 0);
}

namespace {
AmeisenNavigationSettings AnpWorldSettings()
{
    AmeisenNavigationSettings settings;
    settings.meshFolder = TestWorld::Get().meshDir;
    settings.useAnp = true;
    settings.maxPointPath = 256;
    return settings;
}

int CountFlag(const unsigned char* flags, int count, unsigned char flag)
{
    int n = 0;

    for (int i = 0; i < count; ++i)
    {
        n += (flags[i] & flag) ? 1 : 0;
    }

    return n;
}
} // namespace

TEST_CASE(OffMesh_AutoDropLinksLeadOffThePlatform)
{
    // The platform deck (y = 5) can't be reached or left on foot.
    const Vector3 deck = Wow(-425.0f, TestWorld::PLATFORM_Y, -425.0f);
    const Vector3 ground = Wow(-600.0f, 0.0f, -425.0f);

    {
        AmeisenNavigation nav(AnpWorldSettings());
        REQUIRE(nav.NewClient(1));
        Path path(256);
        bool partial = false;
        REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, deck, ground, path, &partial));
        CHECK(partial);
    }

    AmeisenNavigationSettings settings = AnpWorldSettings();
    settings.autoDropHeight = 8.0f;
    AmeisenNavigation nav(settings);
    REQUIRE(nav.NewClient(1));
    Path path(256);
    std::vector<unsigned char> flags(256);
    bool partial = true;
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, deck, ground, path, &partial, nullptr, flags.data()));
    CHECK(!partial);
    CHECK(nav.GetAddedOffMeshLinks(TestWorld::MAP_ID) > 0);
    CHECK_EQ(CountFlag(flags.data(), path.pointCount, PATH_POINT_DROP), 1);
    CHECK((flags[0] & PATH_POINT_START) != 0);
    CHECK((flags[static_cast<size_t>(path.pointCount - 1)] & PATH_POINT_END) != 0);

    // The drop starts on the deck and lands on the ground next to it.
    for (int i = 0; i + 1 < path.pointCount; ++i)
    {
        if (flags[static_cast<size_t>(i)] & PATH_POINT_DROP)
        {
            CHECK_NEAR(path[i].z, TestWorld::PLATFORM_Y, 1.0);
            CHECK_NEAR(path[i + 1].z, 0.0f, 1.0);
            CHECK(path[i].DistanceTo(path[i + 1]) < 8.0f);
        }
    }

    // One way: still no way up.
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, ground, deck, path, &partial));
    CHECK(partial);

    // No drops along the open ground (no ledges there): the ground doesn't drop anywhere else.
    CHECK(nav.GetAddedOffMeshLinks(TestWorld::MAP_ID) < 64);
}

TEST_CASE(OffMesh_FileLinksAndJumpsAreReported)
{
    const auto file = std::filesystem::temp_directory_path() / "anav_tests" / "offmesh.txt";
    std::filesystem::create_directories(file.parent_path());

    // Across the wall (WoW: x = RD z, y = RD x).
    {
        std::ofstream out(file);
        out << "// comment line\n";
        out << std::format("{} 32,32 (-300 {} 0) (-300 {} 0) 1.5  // over the wall\n", TestWorld::MAP_ID,
                           TestWorld::WALL_X - 3.0f, TestWorld::WALL_X + 4.0f);
        out << "999 1,1 (0 0 0) (1 1 1) 1.0\n";          // other map
        out << "1 1,1 (nan 0 0) (1 1 1) 1.0\n";          // invalid
    }

    AmeisenNavigationSettings settings = AnpWorldSettings();
    settings.offMeshFile = file;
    AmeisenNavigation nav(settings);
    REQUIRE(nav.NewClient(1));

    const Vector3 start = Wow(-700.0f, 0.0f, -300.0f);
    const Vector3 end = Wow(-200.0f, 0.0f, -300.0f);
    Path path(256);
    std::vector<unsigned char> flags(256);
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, start, end, path, nullptr, nullptr, flags.data()));
    CHECK_EQ(nav.GetAddedOffMeshLinks(TestWorld::MAP_ID), 1);
    CHECK_EQ(CountFlag(flags.data(), path.pointCount, PATH_POINT_LINK), 1);

    // Refinement keeps both ends of the link.
    PathOptions all;
    all.shortcut = all.avoidWalls = all.roundCorners = true;
    REQUIRE(nav.GetPath(1, TestWorld::MAP_ID, start, end, path, nullptr, &all, flags.data()));
    bool kept = false;

    for (int i = 0; i + 1 < path.pointCount; ++i)
    {
        if (flags[static_cast<size_t>(i)] & PATH_POINT_LINK)
        {
            kept = std::fabs(path[i].y - (TestWorld::WALL_X - 3.0f)) < 1.6f
                   && std::fabs(path[i + 1].y - (TestWorld::WALL_X + 4.0f)) < 1.6f;
        }
    }

    CHECK(kept);

    // Jump links are excluded by default and enabled with a cost.
    NavMeshPtr mesh = LoadWorld();
    OffMeshLink jump;
    jump.start = {TestWorld::WALL_X - 3.0f, 0.0f, -250.0f};
    jump.end = {TestWorld::WALL_X + 4.0f, 0.0f, -250.0f};
    jump.radius = 1.5f;
    jump.bidirectional = true;
    jump.area = AREA_OFFMESH_JUMP;
    jump.flags = NAV_JUMP;
    jump.userId = static_cast<unsigned int>(OffMeshLinkKind::JUMP);
    REQUIRE(TileRebuilder::AddOffMeshLinks(*mesh, std::span(&jump, 1)) == 1);

    AnpQueryFilterProvider provider;
    NavMeshQueryPtr query(dtAllocNavMeshQuery());
    REQUIRE(dtStatusSucceed(query->init(mesh.get(), 4096)));
    AmeisenNavClient client(1, &provider, 2048, 256);
    const auto jumpCorners = [&]() {
        const float extents[3]{2.0f, 4.0f, 2.0f};
        const Vector3 s{-700.0f, 0.0f, -250.0f};
        const Vector3 e{-200.0f, 0.0f, -250.0f};
        dtPolyRef sRef = 0, eRef = 0;
        Vector3 sp, ep;
        query->findNearestPoly(s, extents, client.QueryFilter(), &sRef, sp);
        query->findNearestPoly(e, extents, client.QueryFilter(), &eRef, ep);
        dtPolyRef polys[2048];
        int count = 0;
        query->findPath(sRef, eRef, sp, ep, client.QueryFilter(), polys, &count, 2048);
        float points[256 * 3];
        unsigned char straightFlags[256];
        dtPolyRef refs[256];
        int n = 0;
        query->findStraightPath(sp, ep, polys, count, points, straightFlags, refs, &n, 256);
        unsigned char pointFlags[256];
        // Same conversion as the server.
        int jumps = 0;

        for (int i = 0; i < n; ++i)
        {
            const dtOffMeshConnection* con = (straightFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
                                                 ? mesh->getOffMeshConnectionByRef(refs[i])
                                                 : nullptr;
            pointFlags[i] = con && con->userId == static_cast<unsigned int>(OffMeshLinkKind::JUMP) ? 1 : 0;
            jumps += pointFlags[i];
        }

        return jumps;
    };

    CHECK_EQ(jumpCorners(), 0);
    const AreaCost enableJumps[]{{AREA_OFFMESH_JUMP, 2.0f}};
    REQUIRE(client.ConfigureQueryFilter(ClientState::NORMAL, enableJumps));
    CHECK_EQ(jumpCorners(), 1);
}

TEST_CASE(NavMeshPly_WritesEveryTriangleAndLink)
{
    NavMeshPtr mesh = LoadWorld();
    OffMeshLink link;
    link.start = {TestWorld::WALL_X - 3.0f, 0.0f, -300.0f};
    link.end = {TestWorld::WALL_X + 4.0f, 0.0f, -300.0f};
    link.radius = 1.5f;
    link.area = AREA_OFFMESH_JUMP;
    link.flags = NAV_JUMP;
    REQUIRE(TileRebuilder::AddOffMeshLinks(*mesh, std::span(&link, 1)) == 1);

    const auto path = std::filesystem::temp_directory_path() / "anav_tests" / "navmesh.ply";
    NavMeshPly::Stats stats;
    REQUIRE(NavMeshPly::Write(*mesh, path.string(), {}, &stats));
    CHECK(stats.triangles > 100);
    CHECK_EQ(stats.offMeshConnections, size_t{1});

    // Header counts match the content: 3 vertices (15 bytes) and one face (13 bytes) per triangle.
    std::ifstream in(path, std::ios::binary);
    std::string line;
    size_t vertices = 0;
    size_t faces = 0;

    while (std::getline(in, line) && line != "end_header")
    {
        std::sscanf(line.c_str(), "element vertex %zu", &vertices);
        std::sscanf(line.c_str(), "element face %zu", &faces);
    }

    const size_t triangles = stats.triangles + 4 * stats.offMeshConnections;
    CHECK_EQ(faces, triangles);
    CHECK_EQ(vertices, triangles * 3);
    const auto headerEnd = static_cast<size_t>(in.tellg());
    CHECK_EQ(std::filesystem::file_size(path) - headerEnd, triangles * (3 * 15 + 13));

    // A box around nothing writes an empty but valid file.
    NavMeshPly::Options empty;
    empty.box = true;
    empty.boxMin[0] = empty.boxMin[1] = 15000.0f;
    empty.boxMax[0] = empty.boxMax[1] = 15001.0f;
    REQUIRE(NavMeshPly::Write(*mesh, path.string(), empty, &stats));
    CHECK_EQ(stats.triangles, size_t{0});
}
