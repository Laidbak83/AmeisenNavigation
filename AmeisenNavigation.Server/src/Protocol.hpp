#pragma once

#include <cstddef>
#include <cstdint>

#include "Clients/ClientState.hpp"
#include "Utils/Vector3.hpp"

/// Wire-protocol enums and structs shared between the C++ server and the C# client.
/// These must stay in sync with AmeisenNavigation.Client/WireFormat.cs (little endian, natural alignment).
///
/// Every request gets exactly one response with the same message type:
///   PATH / RANDOM_PATH      -> Vector3[] (a single zero vector on failure), request: PathRequestData
///                              [+ PathTuningData]
///   MOVE_ALONG_SURFACE      -> Vector3   (zero on failure)
///   RANDOM_POINT(_AROUND)   -> Vector3   (zero on failure)
///   CAST_RAY                -> Vector3   (end position if the ray is clear, zero if blocked/failed)
///   CONFIGURE_FILTER        -> bool
///   GET_HEIGHT              -> Vector3   (zero on failure)
///   GET_CONFIG              -> GetConfigResponseHeader + meshes path bytes
///                              + GetConfigResponseTrailer + server version bytes (protocol 2+)
///   CAST_RAY_EX             -> CastRayExResponse
///   EXPLORE_POLY            -> Vector3[] (a single zero vector on failure)
///   SET_AVOID_ZONES         -> bool, request: AvoidZonesHeader + zoneCount AvoidZoneData
///   STEER                   -> SteerResponse, request: SteerRequestHeader + unitCount SteerUnitData
///   REPORT_STUCK            -> bool, request: ReportStuckData
///   PATH_EX                 -> PathExResponseHeader + pointCount Vector3 + pointCount uint8 PathPointFlags
///                              (pointCount 0 on failure, resultFlags: PathExResultFlags), request: like PATH
///                              (RANDOMIZE flag for random offsets)
///                              + with PATH_RESULT_SURFACE_INFO: pointCount uint8 navmesh areas
///                              + pointCount uint8 PathPointInfo
///   LINE_OF_SIGHT           -> WorldHitResponse, request: LineOfSightData
///   SURFACES                -> SurfacesResponseHeader + count SurfaceData, request: SurfacesRequestData
///   SWEEP                   -> WorldHitResponse, request: SweepData
///   SET_OBJECTS             -> SetObjectsResponse, request: SetObjectsHeader + count ObjectData
///                              + removeCount uint64 guids
///   unknown types           -> empty payload

enum class MessageType : unsigned char
{
    PATH,                // Generate a simple straight path
    MOVE_ALONG_SURFACE,  // Move an entity by small deltas using pathfinding
    RANDOM_POINT,        // Get a random point on the mesh
    RANDOM_POINT_AROUND, // Get a random point on the mesh in a circle
    CAST_RAY,            // Cast a movement ray to test for obstacles
    RANDOM_PATH,         // Generate a straight path with random offsets
    EXPLORE_POLY,        // Generate a route that explores a polygon area
    CONFIGURE_FILTER,    // Configure the client's dtQueryFilter area costs
    GET_HEIGHT,          // Get the navmesh terrain height at a position
    GET_CONFIG,          // Get the server's configuration (meshes path, format, etc.)
    CAST_RAY_EX,         // Cast a movement ray and get hit position, normal and fraction
    SET_AVOID_ZONES,     // Replace the connection's avoid zones on a map
    STEER,               // Local steering: velocity for one tick avoiding units, zones and walls
    REPORT_STUCK,        // Report a position where the client got stuck (a zone every client avoids)
    PATH_EX,             // Path with a flag per point (start/end, where to jump or drop, partial result)
    LINE_OF_SIGHT,       // Line of sight against the collision geometry (static world)
    SURFACES,            // Every surface under a point with its provenance (static world)
    SWEEP,               // Move a standing body along a segment: what it runs into (static world)
    SET_OBJECTS,         // Add, move, open/close or remove game objects: their collision changes the navmesh
};

enum class PathType
{
    STRAIGHT, // Request a simple straight path
    RANDOM,   // Request a path with small random deltas per position
};

enum class PathRequestFlags : int
{
    NONE = 0,
    SMOOTH_CHAIKIN = 1 << 0,     // Smooth path using Chaikin Curve
    SMOOTH_CATMULLROM = 1 << 1,  // Smooth path using Catmull-Rom Spline
    SMOOTH_BEZIERCURVE = 1 << 2, // Smooth path using Bezier Curve
    VALIDATE_CPOP = 1 << 3,      // Validate smoothed path using closestPointOnPoly
    VALIDATE_MAS = 1 << 4,       // Validate smoothed path using moveAlongSurface
    REQUIRE_COMPLETE = 1 << 5,   // Fail instead of returning a partial path when the end is unreachable
    AVOID_WALLS = 1 << 6,        // Keep corners fWallMargin away from the walls they bend around
    SHORTCUT = 1 << 7,           // Skip corners wherever the navmesh allows a straight line
    ROUND_CORNERS = 1 << 8,      // Replace corners by arcs (fCornerRadius, iCornerPoints) around them
    FOLLOW = 1 << 9,             // PATH only: reuse the corridor of the connection's previous path
    RANDOMIZE = 1 << 10,         // PATH_EX only: random offsets like RANDOM_PATH
    CLEAR_BODY = 1 << 11,        // sweep a body along the path (static world), go around what blocks it
    DETAIL = 1 << 12,            // dense waypoints on the surface: area changes, height bends, max spacing
    SURFACE_INFO = 1 << 13,      // PATH_EX only: navmesh area and PathPointInfo of every point
};

/// Bumped whenever requests/responses are added or extended (reported by GET_CONFIG).
///   1: PATH ... CAST_RAY_EX
///   2: EXPLORE_POLY, PathRequestFlags::REQUIRE_COMPLETE, GET_CONFIG trailer
///   3: AVOID_WALLS, SHORTCUT, ROUND_CORNERS, FOLLOW, PathTuningData trailer, SET_AVOID_ZONES, STEER,
///      REPORT_STUCK, PATH_EX, CONFIGURE_FILTER cost 0 (forbid) / > 0 (allow areas excluded by default)
///   4: LINE_OF_SIGHT, SURFACES, SWEEP (static world), SET_OBJECTS (game objects at runtime), CLEAR_BODY and the
///      PATH_EX point flags BYPASS / BLOCKED, DETAIL and the PathDetailData trailer,
///      SURFACE_INFO (PATH_EX areas and PathPointInfo), PATH_EX result flags UNREACHABLE / SEARCH_LIMIT
constexpr int PROTOCOL_VERSION = 4;

constexpr bool HasFlag(int flags, PathRequestFlags flag) noexcept { return (flags & static_cast<int>(flag)) != 0; }

struct PathRequestData
{
    int mapId;
    int flags;
    Vector3 start;
    Vector3 end;
};

/// Optional trailer of PATH / RANDOM_PATH (after PathRequestData) and EXPLORE_POLY (after the polygon points).
/// Negative or non-finite values use the server's defaults (fWallMargin, fCornerRadius, iCornerPoints).
struct PathTuningData
{
    float wallMargin;
    float cornerRadius;
    int cornerPoints;
    float stopDistance; // end this far before the target, at a point that sees it (0 = at the target)
};

/// Optional second trailer of PATH / RANDOM_PATH / PATH_EX (after PathTuningData) for DETAIL. Values <= 0 or
/// non-finite use the server's defaults (fWaypointSpacing, fWaypointHeightError).
struct PathDetailData
{
    float spacing;     // longest distance between two waypoints
    float heightError; // how far the surface may leave the line between two waypoints
};

/// PATH_EX response header.
struct PathExResponseHeader
{
    int pointCount;
    int resultFlags; // PathExResultFlags
};

enum PathExResultFlags : int
{
    PATH_RESULT_PARTIAL = 1 << 0,      // the end isn't reachable, the path leads as close as possible
    PATH_RESULT_SURFACE_INFO = 1 << 1, // areas and PathPointInfo follow the flags (not after legacy smoothing)
    PATH_RESULT_UNREACHABLE = 1 << 2,  // partial: no path exists (another part of the navmesh, or the filter forbids
                                       // the way), asking again won't help
    PATH_RESULT_SEARCH_LIMIT = 1 << 3, // partial: the search ran out of nodes (too far or too winding), the path
                                       // leads towards the target, ask again from further on
};

struct MoveRequestData
{
    int mapId;
    Vector3 start;
    Vector3 end;
};

struct CastRayData
{
    int mapId;
    Vector3 start;
    Vector3 end;
};

struct CastRayExResponse
{
    int hit;          // 1 = wall hit, 0 = clear, -1 = error (no mesh / start off-mesh)
    float t;          // fraction of start->end travelled before the hit (1 if clear)
    Vector3 position; // hit position (end if clear)
    Vector3 normal;   // wall normal (zero if clear)
};

struct RandomPointAroundData
{
    int mapId;
    Vector3 start;
    float radius;
};

struct GetHeightData
{
    int mapId;
    Vector3 position;
};

struct GetConfigResponseHeader
{
    int mmapFormat;
    int useAnpFileFormat;
    int pathLength;
};

/// Follows the meshes path of a GET_CONFIG response (older clients ignore it).
struct GetConfigResponseTrailer
{
    int protocolVersion;
    int maxPointPath;
    int versionLength; // server version string bytes following the trailer
};

/// Variable length: header followed by pointCount Vector3 polygon vertices (WoW coordinates).
struct ExplorePolyRequestHeader
{
    int mapId;
    int flags;     // PathRequestFlags applied to the route (smoothing/validation)
    Vector3 start;
    float spacing; // distance between waypoints, e.g. twice the sight range
    int pointCount;
};

/// Variable length: header followed by zoneCount AvoidZoneData (0 = remove the zones of the map).
struct AvoidZonesHeader
{
    int mapId;
    int zoneCount;
};

struct AvoidZoneData
{
    Vector3 center; // WoW coordinates
    float radius;
    float cost;     // travel cost multiplier inside (>= 1)
};

/// Variable length: header followed by unitCount SteerUnitData (nearby units to avoid).
struct SteerRequestHeader
{
    int mapId;
    Vector3 position;
    Vector3 velocity; // current velocity
    Vector3 target;
    float maxSpeed;
    float radius;     // of the steering unit
    float deltaTime;  // tick length, the response position is where the velocity leads after it
    int unitCount;
};

struct SteerUnitData
{
    Vector3 position;
    Vector3 velocity;
    float radius;
};

struct SteerResponse
{
    int ok;           // 1 = success, 0 = failure (no mesh, invalid input)
    Vector3 velocity;
    Vector3 position;
};

struct ReportStuckData
{
    int mapId;
    Vector3 position;
    float radius;     // <= 0: server default (fStuckZoneRadius)
};

/// Static world queries. flags: World::QueryFlags, kinds of geometry ignored (1 terrain, 2 WMOs, 4 doodads,
/// 8 liquids, 16 game objects).
struct LineOfSightData
{
    int mapId;
    int flags;
    Vector3 start;
    Vector3 end;
};

struct SweepData
{
    int mapId;
    int flags;
    Vector3 start; // feet positions
    Vector3 end;
    float radius;
    float height;
    float stepHeight; // geometry below this height above the feet is walked over
};

struct WorldHitResponse
{
    int result;       // 1 = hit, 0 = clear, -1 = no static world there / invalid request
    float t;          // fraction of start -> end before the hit (1 if clear)
    Vector3 position; // hit position (LINE_OF_SIGHT) or where the body stops (SWEEP), end if clear
    Vector3 normal;   // surface normal facing the ray (zero if clear)
    unsigned int source; // kind << 28 | id: 0 terrain, 1 WMO, 2 doodad (M2), 3 liquid, 4 game object (display id)
    int area;            // navmesh area id of the triangle (0 = too steep to stand on)
};

struct SurfacesRequestData
{
    int mapId;
    int flags;
    float x;
    float y;
    float zTop;
    float zBottom;
};

struct SurfacesResponseHeader
{
    int count;           // surfaces following, -1 = no static world there / invalid request
    unsigned int areaId; // AreaTable id of the chunk (0 = unknown)
};

enum SurfaceFlags : unsigned char
{
    SURFACE_STANDABLE = 1 << 0, // walkable ground (not too steep, not liquid)
    SURFACE_LIQUID = 1 << 1,
};

struct SurfaceData
{
    float z;
    Vector3 normal;
    unsigned int source;
    unsigned char area;
    unsigned char flags; // SurfaceFlags
    unsigned short reserved;
};

/// Variable length: header, count ObjectData, removeCount uint64 guids of objects to remove.
struct SetObjectsHeader
{
    int mapId;
    int count;
    int removeCount;
    int reserved;
};

enum ObjectFlags : unsigned int
{
    OBJECT_BLOCKING = 1 << 0, // its collision is in the way (closed door); without it the object is kept but open
};

struct ObjectData
{
    unsigned long long guid;
    unsigned int displayId; // GameObjectDisplayInfo id (models exported to gameobjects.ano)
    unsigned int flags;     // ObjectFlags
    Vector3 position;       // WoW coordinates
    float rotation[4];      // quaternion x, y, z, w (all 0 = none)
    float scale;
};

struct SetObjectsResponse
{
    int accepted;     // objects with a known model, -1 = the map can't change (no static world with build settings)
    int pendingTiles; // navmesh tiles waiting to be rebuilt (paths see the objects once they are)
};

/// Most objects (added + removed) in one SET_OBJECTS request.
constexpr int MAX_OBJECTS_PER_REQUEST = 512;

/// Most surfaces in one SURFACES response.
constexpr int MAX_SURFACES = 64;

struct FilterConfig
{
    unsigned char areaId;
    float cost;
};

/// Variable length: header followed by filterConfigCount FilterConfig entries.
struct ConfigureFilterHeader
{
    ClientState state;
    int filterConfigCount;
};

// Layout checks, the C# client relies on these exact sizes/offsets.
static_assert(sizeof(PathRequestData) == 32);
static_assert(sizeof(PathTuningData) == 16 && offsetof(PathTuningData, stopDistance) == 12);
static_assert(sizeof(PathDetailData) == 8);
static_assert(sizeof(MoveRequestData) == 28);
static_assert(sizeof(CastRayData) == 28);
static_assert(sizeof(CastRayExResponse) == 32);
static_assert(sizeof(RandomPointAroundData) == 20);
static_assert(sizeof(GetHeightData) == 16);
static_assert(sizeof(GetConfigResponseHeader) == 12);
static_assert(sizeof(GetConfigResponseTrailer) == 12);
static_assert(sizeof(ExplorePolyRequestHeader) == 28 && offsetof(ExplorePolyRequestHeader, pointCount) == 24);
static_assert(sizeof(FilterConfig) == 8 && offsetof(FilterConfig, cost) == 4);
static_assert(sizeof(AvoidZonesHeader) == 8 && sizeof(AvoidZoneData) == 20);
static_assert(sizeof(SteerRequestHeader) == 56 && offsetof(SteerRequestHeader, unitCount) == 52);
static_assert(sizeof(SteerUnitData) == 28 && sizeof(SteerResponse) == 28);
static_assert(sizeof(ReportStuckData) == 20);
static_assert(sizeof(PathExResponseHeader) == 8);
static_assert(sizeof(SetObjectsHeader) == 16 && sizeof(ObjectData) == 48 && offsetof(ObjectData, position) == 16);
static_assert(sizeof(SetObjectsResponse) == 8);
static_assert(sizeof(LineOfSightData) == 32 && sizeof(SweepData) == 44);
static_assert(sizeof(WorldHitResponse) == 40 && offsetof(WorldHitResponse, source) == 32);
static_assert(sizeof(SurfacesRequestData) == 24 && sizeof(SurfacesResponseHeader) == 8);
static_assert(sizeof(SurfaceData) == 24 && offsetof(SurfaceData, area) == 20);
static_assert(sizeof(ConfigureFilterHeader) == 8 && offsetof(ConfigureFilterHeader, filterConfigCount) == 4);

/// Maximum number of filter entries in one CONFIGURE_FILTER request (one per Detour area).
constexpr int MAX_FILTER_CONFIGS = 64;
