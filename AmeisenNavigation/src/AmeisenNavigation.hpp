#pragma once

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <mutex>
#include <random>
#include <shared_mutex>
#include <span>
#include <string>
#include <unordered_map>

#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>

#include "Clients/AmeisenNavClient.hpp"
#include "Helpers/Polygon.hpp"
#include "Helpers/Tour.hpp"
#include "NavSources/Anp/AnpNavSource.hpp"
#include "NavSources/Anp/AnpQueryFilterProvider.hpp"
#include "NavSources/AvoidZoneStore.hpp"
#include "NavSources/INavSource.hpp"
#include "NavSources/Mmap/MmapNavSource.hpp"
#include "NavSources/Mmap/MmapQueryFilterProvider.hpp"
#include "NavSources/NavMeshQueryPool.hpp"
#include "NavSources/OffMesh/OffMeshLinks.hpp"
#include "Smoothing/BezierCurve.hpp"
#include "Smoothing/CatmullRomSpline.hpp"
#include "Smoothing/ChaikinCurve.hpp"
#include "Utils/Logger.hpp"
#include "Utils/Path.hpp"
#include "Utils/PolyPosition.hpp"
#include "Utils/Vector3.hpp"
#include "Utils/VectorUtils.hpp"

/// Random float in [0, 1) for Detour's random point functions. One generator per thread.
inline float GetRandomFloat() noexcept
{
    thread_local std::mt19937 rng{std::random_device{}()};
    thread_local std::uniform_real_distribution<float> dis{0.0f, 1.0f};
    return dis(rng);
}

/// Search neighborhood half-extents for dtNavMeshQuery::findNearestPoly (in RD coordinates).
constexpr float NEAREST_POLY_EXTENTS[3] = {6.0f, 6.0f, 6.0f};

/// Second search for start/end positions with nothing within NEAREST_POLY_EXTENTS: a character in the air (jumping,
/// falling, flying, on a mount above the ground) gets the navmesh below it.
constexpr float NEAREST_POLY_FALLBACK_EXTENTS[3] = {6.0f, 50.0f, 6.0f};

/// A raycast that stops this close to its end counts as clear: an end exactly on a polygon edge or corner (path
/// corners are) is otherwise reported as a wall hit at t = 1.
constexpr float RAYCAST_END_TOLERANCE = 0.05f;

/// Larger vertical search extents for height queries where the input Z is unknown/unreliable.
constexpr float HEIGHT_QUERY_EXTENTS[3] = {6.0f, 2500.0f, 6.0f};

/// Maximum distance per step in PostProcessMoveAlongSurface before subdividing.
constexpr float MOVE_ALONG_SURFACE_MAX_CHUNK = 25.0f;

/// Size of the visited polygon buffer for moveAlongSurface calls. Detour's search runs in its 64 node tiny pool, so
/// the visited path has at most 64 polygons: with room for one more the list is never cut (Detour flags a full
/// buffer as too small) and its last entry is always the polygon holding the result.
constexpr int MOVE_ALONG_SURFACE_VISITED_SIZE = 64 + 1;

/// ExplorePolygon limits: outline vertices, waypoints (the spacing grows to stay below it), minimum spacing,
/// size of the polygon's bounding box and how far a waypoint may move when it is snapped to the navmesh.
constexpr int MAX_EXPLORE_POLYGON_POINTS = 256;
constexpr int MAX_EXPLORE_WAYPOINTS = 1024;
constexpr float MIN_EXPLORE_SPACING = 2.0f;
constexpr float MAX_EXPLORE_EXTENT = 10000.0f;
constexpr float MAX_EXPLORE_SNAP_DISTANCE = 32.0f;

/// Request limits. The WoW world spans +-17067 yards; bigger or non-finite coordinates are rejected before they
/// reach Detour, whose float -> int tile conversions are undefined for them (and loop for ages on ARM64).
/// Map ids are bounded by NavMeshCache::MAX_MAP_ID.
constexpr float MAX_COORDINATE = 100000.0f;
constexpr float MAX_QUERY_RADIUS = 5000.0f;

/// True for finite positions inside the accepted coordinate range.
inline bool IsValidPosition(const Vector3& v) noexcept
{
    return v.IsFinite() && std::fabs(v.x) <= MAX_COORDINATE && std::fabs(v.y) <= MAX_COORDINATE
           && std::fabs(v.z) <= MAX_COORDINATE;
}

/// WoW position -> RD coordinates, false (and rd untouched) if the position isn't valid.
inline bool ToValidRdCoords(const Vector3& wow, Vector3& rd) noexcept
{
    if (!IsValidPosition(wow))
    {
        return false;
    }

    wow.CopyToRDCoords(rd);
    return true;
}

struct AmeisenNavigationSettings
{
    std::filesystem::path meshFolder;
    bool useAnp = false;
    MmapFormat mmapFormat = MmapFormat::UNKNOWN;
    MmapNavSource::Patterns customMmapPatterns{};
    std::unordered_map<int, int> parentMaps; // MMAPs: child map id -> parent map id (files fall back to the parent)
    int maxPolyPath = 2048;
    int maxPointPath = 512;
    int maxSearchNodes = 65535;
    float factionDangerCost = 3.0f;
    float waterCost = 1.6f;     // water and ocean
    float badLiquidCost = 4.0f; // lava and slime
    float roadCost = 0.75f;     // ANP roads (< 1 prefers roads)
    size_t maxIdleQueriesPerMap = 0; // dtNavMeshQuery objects kept per map between requests, 0 = automatic
    std::filesystem::path avoidZonesFile; // REPORT_STUCK zones are loaded from / appended to this file (optional)
    float stuckZoneRadius = 3.0f;        // radius of a stuck report that doesn't give one
    float stuckZoneCost = 8.0f;          // cost multiplier of a stuck zone (raised by repeated reports)
    std::filesystem::path offMeshFile;   // off-mesh connections in TrinityCore's offmesh.txt format (optional)
    float autoDropHeight = 0.0f;         // one way drop-down links from ledges up to this height (0 = off)
    float autoDropReach = 2.0f;          // how far beyond a ledge the landing point may be
};

/// A unit near a steering client (WoW coordinates).
struct SteerUnit
{
    Vector3 position;
    Vector3 velocity;
    float radius = 0.5f;
};

/// Result of Steer (WoW coordinates).
struct SteerResult
{
    Vector3 velocity; // velocity for the next tick (horizontal)
    Vector3 position; // where that velocity leads on the navmesh after deltaTime
};

constexpr int MAX_STEER_UNITS = 32;
constexpr float MAX_STEER_SPEED = 100.0f;
constexpr float MAX_STEER_RADIUS = 10.0f;
constexpr float MAX_STEER_DELTA_TIME = 5.0f;

/// Per request path tuning (PATH, RANDOM_PATH, EXPLORE_POLY), see PathOptions.
struct PathTuning
{
    float wallMargin = 1.0f;   // how far corners are moved away from the walls they bend around
    float cornerRadius = 2.0f; // radius of the arcs replacing corners
    int cornerPoints = 4;      // points per arc
    float stopDistance = 0.0f; // end the path this far before the target (0 = at the target)
};

/// Path post-processing on the navmesh. Every changed segment is checked with a navmesh raycast and a step that
/// fails is skipped, so the result is never worse than the plain path.
struct PathOptions
{
    bool shortcut = false;     // skip corners wherever a straight line stays on the navmesh
    bool avoidWalls = false;   // keep corners tuning.wallMargin away from the walls they bend around
    bool roundCorners = false; // replace corners by arcs
    bool follow = false;       // reuse the corridor of the client's previous path on this map (GetPath only)
    PathTuning tuning{};
};

/// Limits for PathTuning values (clamped).
constexpr float MAX_WALL_MARGIN = 10.0f;
constexpr float MAX_CORNER_RADIUS = 25.0f;
constexpr int MAX_CORNER_POINTS = 16;
constexpr float MAX_STOP_DISTANCE = 200.0f;

/// Follow requests reuse the previous corridor while the start stays this close to it and the target this close to
/// the previous target, otherwise the path is searched again.
constexpr float FOLLOW_MAX_START_DEVIATION = 3.0f;
constexpr float FOLLOW_MAX_TARGET_MOVE = 10.0f;

/// What a path point is (PATH_EX reports them).
enum PathPointFlags : unsigned char
{
    PATH_POINT_START = 1 << 0,
    PATH_POINT_END = 1 << 1,
    PATH_POINT_OFFMESH = 1 << 2, // leave this point by an off-mesh connection, the next point is where it ends
    PATH_POINT_JUMP = 1 << 3,    // ... by jumping (over a low obstacle or a gap)
    PATH_POINT_DROP = 1 << 4,    // ... by walking off a ledge
    PATH_POINT_LINK = 1 << 5,    // ... by a custom connection (sOffMeshFile, connections of the navmesh file)
};

/// Details of an ExplorePolygon run.
struct ExploreResult
{
    float spacing = 0.0f;  // spacing actually used (grown if the polygon needed too many waypoints)
    int waypoints = 0;     // waypoints on the navmesh inside the polygon
    int reached = 0;       // waypoints included in the route (the rest was unreachable)
    bool truncated = false; // the route was cut at the path buffer size
};

/// Result of CastMovementRay (positions in WoW coordinates).
struct RaycastResult
{
    bool hit = false;           // true if a wall was hit before reaching the end position
    float t = 0.0f;             // hit parameter along start->end (1 if no hit)
    Vector3 hitPosition{};      // position where the wall was hit (end position if no hit)
    Vector3 hitNormal{};        // wall normal (zero if no hit)
};

/// Thread-safe navigation engine. Any number of clients can issue queries concurrently, but each
/// individual client id must only be used by one thread at a time.
class AmeisenNavigation
{
    AmeisenNavigationSettings Settings;
    std::unique_ptr<INavSource> NavSource;
    std::unique_ptr<IQueryFilterProvider> FilterProvider;

    mutable std::shared_mutex ClientsMutex;
    std::unordered_map<size_t, std::shared_ptr<AmeisenNavClient>> Clients;

    AvoidZoneStore GlobalZones;

    mutable std::mutex OffMeshStatsMutex;
    std::unordered_map<int, int> AddedOffMeshLinks;

    /// Add the configured off-mesh connections to a freshly loaded navmesh (INavSource post-load hook).
    void AddOffMeshLinks(int mapId, dtNavMesh& navMesh);

    // Query pools per map, created with the first request for a map and never removed (navmeshes aren't either).
    mutable std::shared_mutex QueryPoolsMutex;
    std::unordered_map<int, std::unique_ptr<NavMeshQueryPool>> QueryPools;

public:
    explicit AmeisenNavigation(const AmeisenNavigationSettings& settings);

    AmeisenNavigation(const AmeisenNavigation&) = delete;
    AmeisenNavigation& operator=(const AmeisenNavigation&) = delete;


    /// MMAP format in use (UNKNOWN for ANP navmeshes or when detection failed).
    MmapFormat GetMmapFormat() const noexcept;

    /// Off-mesh connections added to a map's navmesh when it was loaded (file + automatic ones).
    int GetAddedOffMeshLinks(int mapId) const;

    bool NewClient(size_t clientId);
    void FreeClient(size_t clientId);
    std::shared_ptr<AmeisenNavClient> GetClient(size_t clientId) const;
    size_t GetClientCount() const;

    /// Load a map's navmesh now instead of on the first request. Returns false if it doesn't exist.
    bool PreloadMap(int mapId) noexcept;

    /// dtNavMeshQuery objects created for a map so far (follows the peak number of concurrent requests).
    size_t GetQueryCount(int mapId) const;

    /// Find a path from start to end. Returns true on success, populates path (WoW coordinates).
    /// If the end isn't reachable the path leads as close as possible and *partial is set to true.
    /// pointFlags (optional, path.maxSize entries) receives the PathPointFlags of every point.
    bool GetPath(size_t clientId, int mapId, const Vector3& startPosition, const Vector3& endPosition, Path& path,
                 bool* partial = nullptr, const PathOptions* options = nullptr, unsigned char* pointFlags = nullptr);

    /// Find a path with randomized intermediate waypoints (within maxRandomDistance).
    bool GetRandomPath(size_t clientId, int mapId, const Vector3& startPosition, const Vector3& endPosition,
                       Path& path, float maxRandomDistance, bool* partial = nullptr,
                       const PathOptions* options = nullptr, unsigned char* pointFlags = nullptr);

    /// Route that explores an area: waypoints on a hexagonal grid (`spacing` apart) inside the polygon
    /// (WoW x/y outline, the outline's z picks the floor), snapped to the navmesh, visited in a short tour
    /// from the start and connected by navmesh paths. Unreachable waypoints are skipped, the route is cut
    /// at the path's capacity. Returns false if no waypoint could be reached.
    /// Options: shortcut, stopDistance and follow don't apply (the route has to visit its waypoints).
    bool ExplorePolygon(size_t clientId, int mapId, const Vector3& startPosition, std::span<const Vector3> polygon,
                        float spacing, Path& path, ExploreResult* result = nullptr,
                        const PathOptions* options = nullptr);

    /// Move from start towards end along the navmesh surface (small deltas), result is snapped to the surface.
    bool MoveAlongSurface(size_t clientId, int mapId, const Vector3& startPosition, const Vector3& endPosition,
                          Vector3& positionToGoTo);

    bool GetRandomPoint(size_t clientId, int mapId, Vector3& position);

    bool GetRandomPointAround(size_t clientId, int mapId, const Vector3& startPosition, float radius,
                              Vector3& position);

    /// Cast a ray along the navmesh surface. Returns true if the path is clear (no wall hit).
    /// Returns false on hits and on errors, check result->hit to tell them apart.
    bool CastMovementRay(size_t clientId, int mapId, const Vector3& startPosition, const Vector3& endPosition,
                         RaycastResult* result = nullptr);

    /// Get the navmesh surface position at (x, y). Z is only used to pick between stacked floors.
    bool GetHeight(size_t clientId, int mapId, const Vector3& position, Vector3& out);

    /// Snap each path point to the nearest poly surface (validates smoothed paths).
    bool PostProcessClosestPointOnPoly(size_t clientId, int mapId, const Path& input, Path& output);

    /// Walk the path along the navmesh surface (validates smoothed paths with chunking).
    bool PostProcessMoveAlongSurface(size_t clientId, int mapId, const Path& input, Path& output);

    /// Replace a client's avoid zones on a map (centers in WoW coordinates, empty = remove). The search prefers
    /// other ways and paths are led around them. False for invalid zones or too many.
    bool SetAvoidZones(size_t clientId, int mapId, std::span<const AvoidZone> zones);

    /// A client got stuck at a position: add a zone every client avoids from now on (persisted to avoidZonesFile).
    /// radius <= 0 uses the configured default.
    bool ReportStuck(int mapId, const Vector3& position, float radius);

    /// Global (stuck) zones of a map.
    size_t GetGlobalZoneCount(int mapId) const { return GlobalZones.Count(mapId); }

    /// Local steering for one tick: the velocity towards target (along the navmesh path) that avoids the units, the
    /// avoid zones and the navmesh borders (DetourCrowd obstacle avoidance), and the position it leads to.
    bool Steer(size_t clientId, int mapId, const Vector3& position, const Vector3& velocity, const Vector3& target,
               float maxSpeed, float radius, float deltaTime, std::span<const SteerUnit> units, SteerResult& result);

    void SmoothPathChaikinCurve(const Path& input, Path& output) const noexcept;
    void SmoothPathCatmullRom(const Path& input, Path& output, int points, float alpha) const noexcept;
    void SmoothPathBezier(const Path& input, Path& output, int points) const noexcept;

private:
    /// Everything a request needs. Holds the leased query, keep it alive while the query is used.
    struct QueryContext
    {
        std::shared_ptr<AmeisenNavClient> client;
        NavMeshQueryPool::Lease lease;
        dtNavMeshQuery* query = nullptr;
        const dtQueryFilter* filter = nullptr;

        explicit operator bool() const noexcept { return client && query && filter; }
    };

    QueryContext GetQueryContext(size_t clientId, int mapId);

    /// The map's query pool, nullptr if the map has no navmesh.
    NavMeshQueryPool* GetQueryPool(int mapId);

    /// Every position that reaches Detour goes through here or ToValidRdCoords first.
    static bool FindNearestPoly(const dtNavMeshQuery* query, const dtQueryFilter* filter, const Vector3& rdPosition,
                                PolyPosition& result, const float* extents = NEAREST_POLY_EXTENTS) noexcept
    {
        result.poly = 0;

        if (!IsValidPosition(rdPosition))
        {
            return false;
        }

        const dtStatus status = query->findNearestPoly(rdPosition, extents, filter, &result.poly, result.pos);
        return dtStatusSucceed(status) && result.poly != 0;
    }

    /// Nearest polygon to a position given by a client (start/end of a request): within NEAREST_POLY_EXTENTS, else
    /// within NEAREST_POLY_FALLBACK_EXTENTS.
    static bool FindNearestPolyWow(const dtNavMeshQuery* query, const dtQueryFilter* filter,
                                   const Vector3& wowPosition, PolyPosition& result) noexcept
    {
        Vector3 rd;
        wowPosition.CopyToRDCoords(rd);
        return FindNearestPoly(query, filter, rd, result)
               || FindNearestPoly(query, filter, rd, result, NEAREST_POLY_FALLBACK_EXTENTS);
    }

    /// Raycast along the navmesh from start to rdEnd. A start on a polygon edge or corner touches several polygons,
    /// Detour casts from one of them only and may leave it through a border edge at t = 0: the ray is cast from
    /// every polygon touching the start and the furthest result wins. Returns false if no ray could be cast.
    /// visited (optional) receives the polygons of the winning ray if it is clear (hit.path/pathCount).
    static bool Raycast(const dtNavMeshQuery* query, const dtQueryFilter* filter, const PolyPosition& start,
                        const Vector3& rdEnd, dtRaycastHit& hit, dtPolyRef* visited = nullptr,
                        int maxVisited = 0) noexcept;

    /// moveAlongSurface from `position` towards rdTarget. On success `position` is the result, projected onto the
    /// surface (moveAlongSurface doesn't fix the height), and the polygon holding it.
    static dtStatus MoveAlong(const dtNavMeshQuery* query, const dtQueryFilter* filter, PolyPosition& position,
                              const Vector3& rdTarget) noexcept
    {
        dtPolyRef visited[MOVE_ALONG_SURFACE_VISITED_SIZE];
        int visitedCount = 0;
        Vector3 result;

        const dtStatus status = query->moveAlongSurface(position.poly, position.pos, rdTarget, filter, result, visited,
                                                        &visitedCount, MOVE_ALONG_SURFACE_VISITED_SIZE);

        if (dtStatusFailed(status) || visitedCount <= 0)
        {
            return status | DT_FAILURE;
        }

        position.poly = visited[visitedCount - 1];

        if (float height = 0.0f; dtStatusSucceed(query->getPolyHeight(position.poly, result, &height)))
        {
            result.y = height;
        }

        position.pos = result;
        return status;
    }

    /// Straight path in RD coordinates, optionally storing the poly ref of each corner. *partial is set when
    /// the end isn't reachable (the path then leads to the closest reachable point).
    bool CalculateNormalPath(dtNavMeshQuery* query, const dtQueryFilter* filter, AmeisenNavClient& client,
                             const Vector3& startPosition, const Vector3& endPosition, Path& path,
                             dtPolyRef* straightPathRefs = nullptr, bool* partial = nullptr,
                             unsigned char* straightPathFlags = nullptr) noexcept;

    /// Same for positions already on the navmesh.
    bool CalculateNormalPath(dtNavMeshQuery* query, const dtQueryFilter* filter, AmeisenNavClient& client,
                             const PolyPosition& start, const PolyPosition& end, Path& path,
                             dtPolyRef* straightPathRefs = nullptr, bool* partial = nullptr,
                             unsigned char* straightPathFlags = nullptr) noexcept;

    /// Detour straight path flags + corner refs -> PathPointFlags (in place allowed: out may be detourFlags).
    static void ToPointFlags(const dtNavMeshQuery* query, const unsigned char* detourFlags, const dtPolyRef* refs,
                             int count, unsigned char* out) noexcept;

    /// Zone detours and options for a path (RD) with PathPointFlags, in pieces between off-mesh connections (their
    /// ends never move). flags is updated with the path.
    void RefineAll(const dtNavMeshQuery* query, const dtQueryFilter* filter, Path& path, unsigned char* flags,
                   const Vector3& target, const PathOptions* options, bool allowShortcut) const noexcept;

    // ── Path refinement (PathRefinement.cpp), paths in RD coordinates ──

    /// Path along the client's follow corridor (reused if the start and target moved little, searched otherwise).
    bool CalculateFollowPath(dtNavMeshQuery* query, const dtQueryFilter* filter, AmeisenNavClient& client, int mapId,
                             const Vector3& startPosition, const Vector3& endPosition, Path& path, bool* partial,
                             dtPolyRef* straightPathRefs, unsigned char* straightPathFlags) noexcept;

    /// Apply the options to a path without off-mesh connections. target (RD) is the requested end (stop distance).
    void RefinePath(dtNavMeshQuery* query, const dtQueryFilter* filter, Path& path, const Vector3& target,
                    const PathOptions& options, bool allowShortcut, Path& scratch) const noexcept;

    /// True if the navmesh surface leads straight from a to b (raycast along it reaching b's height).
    static bool SegmentClear(const dtNavMeshQuery* query, const dtQueryFilter* filter, const Vector3& a,
                             const Vector3& b) noexcept;

    static void ShortcutPath(const dtNavMeshQuery* query, const dtQueryFilter* filter, Path& path) noexcept;
    static void CutAtStopDistance(const dtNavMeshQuery* query, const dtQueryFilter* filter, Path& path,
                                  const Vector3& target, float stopDistance) noexcept;
    static void PushCornersFromWalls(const dtNavMeshQuery* query, const dtQueryFilter* filter, Path& path,
                                     float margin) noexcept;
    static void RoundCorners(const dtNavMeshQuery* query, const dtQueryFilter* filter, Path& path, float radius,
                             int points, Path& scratch) noexcept;

    static bool HasZones(const dtQueryFilter* filter) noexcept
    {
        const auto* navFilter = dynamic_cast<const NavQueryFilter*>(filter);
        return navFilter && !navFilter->GetZones().empty();
    }

    /// Lead path segments crossing an avoid zone of the filter around it (Avoidance.cpp). The search already prefers
    /// other ways, but string pulling cuts across zones the corridor touches.
    static void LeadAroundZones(const dtNavMeshQuery* query, const dtQueryFilter* filter, Path& path,
                                Path& scratch) noexcept;
};
