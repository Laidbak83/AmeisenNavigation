#include "AmeisenNavigation.hpp"

#include <cfloat>
#include <algorithm>
#include <cmath>
#include <vector>

namespace {
/// Shortcut search: how many corners ahead a straight line is tried.
constexpr int SHORTCUT_LOOKAHEAD = 24;

/// Height difference between a raycast's end polygon and the wanted end point that still counts as "the same
/// surface" (stacked floors are further apart than this).
constexpr float SEGMENT_END_HEIGHT_TOLERANCE = 1.5f;

/// Corners turning less than this (radians) are left alone by the wall margin and the rounding.
constexpr float MIN_CORNER_TURN = 0.17f; // ~10 degrees

/// Stop distance: step of the search for a point in range that sees the target.
constexpr float STOP_DISTANCE_STEP = 0.5f;
constexpr int STOP_DISTANCE_MAX_SAMPLES = 4096;

constexpr float RAYCAST_POLY_EXTENTS[3]{0.5f, 2.0f, 0.5f};

float Length2D(float x, float z) noexcept { return std::sqrt(x * x + z * z); }

/// Snap a point onto the navmesh surface below/above it (nearest polygon within a small box).
bool SnapToSurface(const dtNavMeshQuery* query, const dtQueryFilter* filter, Vector3& point) noexcept
{
    dtPolyRef ref = 0;
    Vector3 nearest;

    if (dtStatusFailed(query->findNearestPoly(point, RAYCAST_POLY_EXTENTS, filter, &ref, nearest)) || !ref)
    {
        return false;
    }

    point = nearest;
    return true;
}
} // namespace

bool AmeisenNavigation::SegmentClear(const dtNavMeshQuery* query, const dtQueryFilter* filter, const Vector3& a,
                                     const Vector3& b) noexcept
{
    PolyPosition start;

    if (dtStatusFailed(query->findNearestPoly(a, RAYCAST_POLY_EXTENTS, filter, &start.poly, start.pos))
        || !start.poly)
    {
        return false;
    }

    // Raycast with the visited polygons to check that the ray arrives on b's surface (not on a floor above or
    // below it with the same x/z).
    constexpr int MAX_VISITED = 256;
    dtPolyRef visited[MAX_VISITED];
    dtRaycastHit hit{};

    if (!Raycast(query, filter, start, b, hit, visited, MAX_VISITED) || hit.t != FLT_MAX || hit.pathCount <= 0)
    {
        return false;
    }

    float height = 0.0f;

    if (dtStatusFailed(query->getPolyHeight(visited[hit.pathCount - 1], b, &height)))
    {
        // b on the edge of the last polygon: compare with the closest point instead.
        Vector3 closest;

        if (dtStatusFailed(query->closestPointOnPoly(visited[hit.pathCount - 1], b, closest, nullptr)))
        {
            return false;
        }

        height = closest.y;
    }

    return std::fabs(height - b.y) < SEGMENT_END_HEIGHT_TOLERANCE;
}

void AmeisenNavigation::ShortcutPath(const dtNavMeshQuery* query, const dtQueryFilter* filter, Path& path) noexcept
{
    if (path.pointCount < 3)
    {
        return;
    }

    int write = 1;
    int i = 0;

    while (i < path.pointCount - 1)
    {
        int next = i + 1;

        for (int j = std::min(path.pointCount - 1, i + SHORTCUT_LOOKAHEAD); j > i + 1; --j)
        {
            if (SegmentClear(query, filter, path[i], path[j]))
            {
                next = j;
                break;
            }
        }

        path[write++] = path[next];
        i = next;
    }

    path.pointCount = write;
}

void AmeisenNavigation::CutAtStopDistance(const dtNavMeshQuery* query, const dtQueryFilter* filter, Path& path,
                                          const Vector3& target, float stopDistance) noexcept
{
    if (path.pointCount < 2 || stopDistance <= 0.0f)
    {
        return;
    }

    // Line of sight is checked towards the target's position on the navmesh (a target may stand off it).
    Vector3 onMesh = target;

    if (!SnapToSurface(query, filter, onMesh))
    {
        onMesh = path[path.pointCount - 1];
    }

    const auto sees = [&](const Vector3& p) {
        dtRaycastHit hit{};
        PolyPosition from;

        if (dtStatusFailed(query->findNearestPoly(p, RAYCAST_POLY_EXTENTS, filter, &from.poly, from.pos)) || !from.poly)
        {
            return false;
        }

        return Raycast(query, filter, from, onMesh, hit) && hit.t == FLT_MAX;
    };

    if (path[0].DistanceTo(target) <= stopDistance && sees(path[0]))
    {
        path.pointCount = 1;
        return;
    }

    int samples = 0;

    for (int i = 1; i < path.pointCount && samples < STOP_DISTANCE_MAX_SAMPLES; ++i)
    {
        const Vector3 a = path[i - 1];
        const Vector3 b = path[i];
        const Vector3 d = b - a;
        const float length = d.Length();

        if (length < 1e-4f)
        {
            continue;
        }

        // Where the segment enters the range: |a + t d - target| = stopDistance, smallest t in [0, 1].
        const Vector3 f = a - target;
        const float qa = d.LengthSquared();
        const float qb = 2.0f * (f.x * d.x + f.y * d.y + f.z * d.z);
        const float qc = f.LengthSquared() - stopDistance * stopDistance;
        const float disc = qb * qb - 4.0f * qa * qc;

        if (disc < 0.0f)
        {
            continue;
        }

        const float root = std::sqrt(disc);
        const float t0 = std::max(0.0f, (-qb - root) / (2.0f * qa));
        const float t1 = std::min(1.0f, (-qb + root) / (2.0f * qa));

        if (t0 > t1)
        {
            continue;
        }

        const float step = STOP_DISTANCE_STEP / length;

        for (float t = t0; t <= t1 + 1e-6f && samples < STOP_DISTANCE_MAX_SAMPLES; t += step, ++samples)
        {
            Vector3 p = a + d * std::min(t, 1.0f);

            if (!SnapToSurface(query, filter, p) || !sees(p))
            {
                continue;
            }

            path[i] = p;
            path.pointCount = i + 1;
            return;
        }
    }
}

namespace {
/// Geometry of the turn at a path corner (x/z plane): unit directions in and out, the turn angle and the unit
/// normals pointing to the outside of the turn (away from the wall the corner bends around).
struct Corner
{
    float d1x, d1z, d2x, d2z;
    float turn;
    float o1x, o1z, o2x, o2z;
};

bool MakeCorner(const Vector3& p, const Vector3& c, const Vector3& n, Corner& k) noexcept
{
    const float l1 = Length2D(c.x - p.x, c.z - p.z);
    const float l2 = Length2D(n.x - c.x, n.z - c.z);

    if (l1 < 1e-3f || l2 < 1e-3f)
    {
        return false;
    }

    k.d1x = (c.x - p.x) / l1;
    k.d1z = (c.z - p.z) / l1;
    k.d2x = (n.x - c.x) / l2;
    k.d2z = (n.z - c.z) / l2;
    k.turn = std::acos(std::clamp(k.d1x * k.d2x + k.d1z * k.d2z, -1.0f, 1.0f));

    // Turning towards the side of (-dz, dx) when the cross product is positive: the outside is the other side.
    const float sign = k.d1x * k.d2z - k.d1z * k.d2x > 0.0f ? 1.0f : -1.0f;
    k.o1x = k.d1z * sign;
    k.o1z = -k.d1x * sign;
    k.o2x = k.d2z * sign;
    k.o2z = -k.d2x * sign;
    return true;
}

/// Move a corner `distance` to the outside of its turn (along the bisector of the outside normals), on the navmesh.
bool PushCorner(const dtNavMeshQuery* query, const dtQueryFilter* filter, const Vector3& c, const Corner& k,
                float distance, Vector3& out) noexcept;
} // namespace

void AmeisenNavigation::PushCornersFromWalls(const dtNavMeshQuery* query, const dtQueryFilter* filter, Path& path,
                                             float margin) noexcept
{
    if (path.pointCount < 3 || margin <= 0.0f)
    {
        return;
    }

    // Directions come from the original corners: around the end of an obstacle the path has several close corners,
    // each one moves out along its own bisector (a moved neighbour would hide the turn).
    const std::vector<Vector3> original(path.begin(), path.end());

    for (int i = 1; i < path.pointCount - 1; ++i)
    {
        Corner k{};

        if (!MakeCorner(original[static_cast<size_t>(i - 1)], original[static_cast<size_t>(i)],
                        original[static_cast<size_t>(i + 1)], k)
            || k.turn < MIN_CORNER_TURN)
        {
            continue;
        }

        // Full margin first, then half of it (narrow passages).
        for (const float m : {margin, margin * 0.5f})
        {
            Vector3 moved;

            if (PushCorner(query, filter, original[static_cast<size_t>(i)], k, m, moved)
                && SegmentClear(query, filter, path[i - 1], moved) && SegmentClear(query, filter, moved, path[i + 1]))
            {
                path[i] = moved;
                break;
            }
        }
    }
}

namespace {
bool PushCorner(const dtNavMeshQuery* query, const dtQueryFilter* filter, const Vector3& c, const Corner& k,
                float distance, Vector3& out) noexcept
{
    const float bx = k.o1x + k.o2x;
    const float bz = k.o1z + k.o2z;
    const float bl = Length2D(bx, bz);

    if (bl < 1e-4f)
    {
        return false; // U-turn: no outside
    }

    out = Vector3{c.x + bx / bl * distance, c.y, c.z + bz / bl * distance};
    return SnapToSurface(query, filter, out) && out.DistanceTo(c) > 0.05f;
}
} // namespace

void AmeisenNavigation::RoundCorners(const dtNavMeshQuery* query, const dtQueryFilter* filter, Path& path,
                                     float radius, int points, Path& scratch) noexcept
{
    if (path.pointCount < 3 || radius <= 0.0f || points < 1)
    {
        return;
    }

    // Every corner is replaced by an arc of `radius` around it on the outside of the turn: the path keeps that
    // distance from the corner (and the wall the corner bends around). Consecutive arcs are joined by the corner
    // segments shifted outwards, a cluster of close corners around an obstacle becomes one smooth curve. If an arc
    // doesn't fit, half the radius is tried, then the corner stays.
    const std::vector<Vector3> original(path.begin(), path.end());
    scratch.Clear();
    scratch.TryAppend(original[0]);
    std::vector<Vector3> arc;
    arc.reserve(static_cast<size_t>(points) + 2);

    for (int i = 1; i < path.pointCount - 1; ++i)
    {
        const Vector3& c = original[static_cast<size_t>(i)];
        const Vector3& n = original[static_cast<size_t>(i + 1)];
        Corner k{};
        bool rounded = false;

        if (MakeCorner(original[static_cast<size_t>(i - 1)], c, n, k) && k.turn >= MIN_CORNER_TURN)
        {
            for (const float r : {radius, radius * 0.5f})
            {
                const Vector3 p = scratch[scratch.pointCount - 1];
                const float a1 = std::atan2(k.o1z, k.o1x);
                float a2 = std::atan2(k.o2z, k.o2x);

                // Sweep the short way round (the turn angle).
                while (a2 - a1 > 3.14159265f)
                {
                    a2 -= 6.2831853f;
                }

                while (a2 - a1 < -3.14159265f)
                {
                    a2 += 6.2831853f;
                }

                arc.clear();

                for (int j = 0; j <= points + 1; ++j)
                {
                    const float angle = a1 + (a2 - a1) * static_cast<float>(j) / static_cast<float>(points + 1);
                    arc.emplace_back(c.x + std::cos(angle) * r, c.y, c.z + std::sin(angle) * r);
                }

                bool valid = scratch.GetSpace() > static_cast<int>(arc.size()) + (path.pointCount - i);

                for (size_t j = 0; valid && j < arc.size(); ++j)
                {
                    valid = SnapToSurface(query, filter, arc[j])
                            && SegmentClear(query, filter, j == 0 ? p : arc[j - 1], arc[j]);
                }

                if (valid && SegmentClear(query, filter, arc.back(), n))
                {
                    for (const Vector3& v : arc)
                    {
                        scratch.TryAppendUnique(v);
                    }

                    rounded = true;
                    break;
                }
            }
        }

        if (!rounded && !scratch.TryAppendUnique(c))
        {
            return; // full, keep the unrounded path
        }
    }

    if (!scratch.TryAppendUnique(original.back()) || scratch.pointCount > path.maxSize)
    {
        return;
    }

    std::copy(scratch.begin(), scratch.end(), path.points);
    path.pointCount = scratch.pointCount;
}

void AmeisenNavigation::RefinePath(dtNavMeshQuery* query, const dtQueryFilter* filter, Path& path,
                                   const Vector3& target, const PathOptions& options, bool allowShortcut,
                                   Path& scratch) const noexcept
{
    const PathTuning& t = options.tuning;

    if (options.shortcut && allowShortcut)
    {
        ShortcutPath(query, filter, path);
    }

    if (t.stopDistance > 0.0f && allowShortcut)
    {
        CutAtStopDistance(query, filter, path, target, std::min(t.stopDistance, MAX_STOP_DISTANCE));
    }

    const float margin = options.avoidWalls ? std::clamp(t.wallMargin, 0.0f, MAX_WALL_MARGIN) : 0.0f;

    if (options.roundCorners)
    {
        // Arcs keep their radius from the corners: with a wall margin they are at least that far away.
        RoundCorners(query, filter, path, std::max(std::clamp(t.cornerRadius, 0.0f, MAX_CORNER_RADIUS), margin),
                     std::clamp(t.cornerPoints, 1, MAX_CORNER_POINTS), scratch);
    }
    else if (margin > 0.0f)
    {
        PushCornersFromWalls(query, filter, path, margin);
    }
}

bool AmeisenNavigation::CalculateFollowPath(dtNavMeshQuery* query, const dtQueryFilter* filter,
                                            AmeisenNavClient& client, int mapId, const Vector3& startPosition,
                                            const Vector3& endPosition, Path& path, bool* partial,
                                            dtPolyRef* straightPathRefs, unsigned char* straightPathFlags,
                                            const NavMeshIslands* islands, PathPartialReason* reason) noexcept
{
    path.Clear();
    FollowState* state = client.GetFollowState();

    if (!state)
    {
        return CalculateNormalPath(query, filter, client, startPosition, endPosition, path, straightPathRefs, partial,
                                   straightPathFlags, islands, reason);
    }

    Vector3 rdStart;
    Vector3 rdEnd;

    if (!ToValidRdCoords(startPosition, rdStart) || !ToValidRdCoords(endPosition, rdEnd))
    {
        return false;
    }

    dtPathCorridor& corridor = state->corridor;
    bool reuse = state->mapId == mapId && corridor.getPathCount() > 0
                 && dtVdist(corridor.getTarget(), rdEnd) <= FOLLOW_MAX_TARGET_MOVE;

    if (reuse)
    {
        // Walk the corridor's start to the new position (trims the polygons left behind).
        reuse = corridor.movePosition(rdStart, query, filter) && dtVdist2D(corridor.getPos(), rdStart) <= FOLLOW_MAX_START_DEVIATION
                && std::fabs(corridor.getPos()[1] - rdStart.y) <= FOLLOW_MAX_START_DEVIATION
                && corridor.isValid(corridor.getPathCount(), query, filter);
    }

    if (reuse && dtVdist(corridor.getTarget(), rdEnd) > 0.01f && !state->partial)
    {
        reuse = corridor.moveTargetPosition(rdEnd, query, filter)
                && dtVdist2D(corridor.getTarget(), rdEnd) <= FOLLOW_MAX_START_DEVIATION;
    }

    if (!reuse)
    {
        state->mapId = -1;
        PolyPosition start;
        PolyPosition end;

        if (!FindNearestPolyWow(query, filter, startPosition, start) || !FindNearestPolyWow(query, filter, endPosition, end))
        {
            return false;
        }

        // Another island: aim at the closest point of the start's island (see CalculateNormalPath).
        bool unreachable = false;

        if (islands)
        {
            const uint32_t from = islands->Of(start.poly);
            const uint32_t to = islands->Of(end.poly);
            PolyPosition reachable;

            if (from && to && from != to)
            {
                unreachable = true;

                if (ClosestReachable(query, filter, *islands, from, end.pos, reachable))
                {
                    end = reachable;
                }
            }
        }

        dtPolyRef* polys = client.GetPolyPathBuffer();
        int polyCount = 0;
        const dtStatus status = query->findPath(start.poly, end.poly, start.pos, end.pos, filter, polys, &polyCount,
                                                client.GetPolyPathBufferSize());

        if (dtStatusFailed(status) || polyCount <= 0)
        {
            return false;
        }

        Vector3 target = end.pos;

        if (polys[polyCount - 1] != end.poly)
        {
            query->closestPointOnPoly(polys[polyCount - 1], end.pos, target, nullptr);
        }

        state->partial = polys[polyCount - 1] != end.poly || unreachable;
        state->reason = static_cast<unsigned char>(!state->partial ? PathPartialReason::NONE
                                                   : !unreachable && dtStatusDetail(status, DT_OUT_OF_NODES)
                                                       ? PathPartialReason::SEARCH_LIMIT
                                                       : PathPartialReason::UNREACHABLE);

        corridor.reset(start.poly, start.pos);
        corridor.setCorridor(target, polys, polyCount);
        state->mapId = mapId;
    }

    if (partial)
    {
        *partial = state->partial;
    }

    if (reason)
    {
        *reason = static_cast<PathPartialReason>(state->reason);
    }

    path.Clear();
    const dtStatus status = query->findStraightPath(
        corridor.getPos(), corridor.getTarget(), corridor.getPath(), corridor.getPathCount(),
        reinterpret_cast<float*>(path.points), straightPathFlags, straightPathRefs, &path.pointCount,
        std::min(path.maxSize, client.GetStraightPathRefBufferSize()));

    if (dtStatusFailed(status) || path.pointCount <= 0)
    {
        state->mapId = -1;
        path.Clear();
        return false;
    }

    return true;
}

void AmeisenNavigation::ClearBody(const dtNavMeshQuery* query, const dtQueryFilter* filter, int mapId, Path& path,
                                  unsigned char* flags, const PathOptions& options) noexcept
{
    World::StaticWorld* world = Worlds.Get(mapId);

    if (!world || path.pointCount < 2)
    {
        return;
    }

    const float radius = std::clamp(options.bodyRadius, 0.0f, 5.0f);
    const float height = std::clamp(options.bodyHeight, 0.5f, 10.0f);
    const float step = std::clamp(options.bodyStepHeight, 0.0f, height - 0.1f);

    const auto blocked = [&](const Vector3& rdA, const Vector3& rdB, World::Hit& hit) {
        Vector3 a = rdA;
        Vector3 b = rdB;
        return world->Sweep(a.ToWowCoords(), b.ToWowCoords(), radius, height, step, 0, hit) && hit.hit;
    };

    try
    {
        std::vector<Vector3> points(path.points, path.points + path.pointCount);
        std::vector<unsigned char> pointFlags(flags, flags + path.pointCount);
        constexpr int MAX_CHECKS = 256;

        for (size_t i = 0; i + 1 < points.size() && static_cast<int>(i) < MAX_CHECKS; ++i)
        {
            World::Hit hit;

            if ((pointFlags[i] & PATH_POINT_OFFMESH) || !blocked(points[i], points[i + 1], hit))
            {
                continue;
            }

            // Candidates beside the obstacle: where the body stops plus a bit, shifted to either side, further out
            // each time. Each has to be on the navmesh, reachable on it and clear for the body both ways.
            Vector3 stop = hit.position;
            stop.ToRDCoords();
            Vector3 dir = points[i + 1] - points[i];
            dir.y = 0.0f;
            const float length = dir.Length();

            if (length < 0.01f)
            {
                continue;
            }

            dir = dir * (1.0f / length);
            const Vector3 side{-dir.z, 0.0f, dir.x};
            bool bypassed = false;

            for (const float offset : {radius + 1.0f, radius + 2.5f, radius + 4.5f})
            {
                for (const float sign : {1.0f, -1.0f})
                {
                    const Vector3 candidate = stop + dir * (radius + 0.5f) + side * (offset * sign);
                    PolyPosition snapped;
                    World::Hit unused;

                    if (!FindNearestPoly(query, filter, candidate, snapped)
                        || std::fabs(snapped.pos.y - candidate.y) > 3.0f
                        || !SegmentClear(query, filter, points[i], snapped.pos)
                        || !SegmentClear(query, filter, snapped.pos, points[i + 1])
                        || blocked(points[i], snapped.pos, unused) || blocked(snapped.pos, points[i + 1], unused))
                    {
                        continue;
                    }

                    if (static_cast<int>(points.size()) >= path.maxSize)
                    {
                        break;
                    }

                    points.insert(points.begin() + static_cast<std::ptrdiff_t>(i) + 1, snapped.pos);
                    pointFlags.insert(pointFlags.begin() + static_cast<std::ptrdiff_t>(i) + 1, PATH_POINT_BYPASS);
                    bypassed = true;
                    i++; // both new segments are clear
                    break;
                }

                if (bypassed)
                {
                    break;
                }
            }

            if (!bypassed)
            {
                pointFlags[i] |= PATH_POINT_BLOCKED;
            }
        }

        std::copy(points.begin(), points.end(), path.points);
        std::copy(pointFlags.begin(), pointFlags.end(), flags);
        path.pointCount = static_cast<int>(points.size());
    }
    catch (...)
    {
    }
}

namespace {
/// One sample of a segment's surface profile.
struct ProfileSample
{
    float t;      // along the segment, 0..1
    Vector3 pos;  // RD, on the navmesh
    unsigned char area;
    bool onMesh;
};

/// Surface under a point of the segment from the polygons the raycast visited (in order, so the right floor is
/// used on stacked geometry), else from the nearest polygon holding the point (segments along the navmesh edge,
/// where the raycast stops early). hint: where to start looking.
bool SurfaceAt(const dtNavMeshQuery* query, const dtQueryFilter* filter, const dtPolyRef* visited, int count, int& hint,
               Vector3& pos, unsigned char& area)
{
    const auto test = [&](int i) {
        float height = 0.0f;

        if (dtStatusFailed(query->getPolyHeight(visited[i], pos, &height)))
        {
            return false;
        }

        pos.y = height;
        query->getAttachedNavMesh()->getPolyArea(visited[i], &area);
        hint = i;
        return true;
    };

    for (int i = std::max(hint, 0); i < count; ++i)
    {
        if (test(i))
        {
            return true;
        }
    }

    for (int i = 0; i < std::min(hint, count); ++i)
    {
        if (test(i))
        {
            return true;
        }
    }

    constexpr float EXTENTS[3]{0.3f, 3.0f, 0.3f};
    dtPolyRef ref = 0;
    float nearest[3]{};
    float height = 0.0f;

    if (dtStatusFailed(query->findNearestPoly(pos, EXTENTS, filter, &ref, nearest)) || !ref)
    {
        return false;
    }

    // getPolyHeight fails when the point lies outside the polygon: the sample is off the navmesh, unless it sits on
    // the edge itself (a path along a wall), where the nearest point is as good.
    if (dtStatusSucceed(query->getPolyHeight(ref, pos, &height)))
    {
        pos.y = height;
    }
    else if (Length2D(nearest[0] - pos.x, nearest[2] - pos.z) <= 0.2f)
    {
        pos.y = nearest[1];
    }
    else
    {
        return false;
    }

    query->getAttachedNavMesh()->getPolyArea(ref, &area);
    return true;
}

/// Why a profile sample becomes a waypoint.
enum SampleKind : unsigned char
{
    SAMPLE_SKIP = 0,
    SAMPLE_END,    // segment start/end or the far side of a ground change
    SAMPLE_BORDER, // the near side of a ground change
    SAMPLE_BEND,   // the height profile bends here
};

/// Douglas-Peucker on the height profile: keep the samples where the surface leaves the line by more than the error.
void KeepBends(const std::vector<ProfileSample>& samples, size_t from, size_t to, float error,
               std::vector<unsigned char>& keep)
{
    if (to <= from + 1)
    {
        return;
    }

    float worst = 0.0f;
    size_t index = from;

    for (size_t i = from + 1; i < to; ++i)
    {
        const float f = (samples[i].t - samples[from].t) / std::max(samples[to].t - samples[from].t, 1e-6f);
        const float line = samples[from].pos.y + (samples[to].pos.y - samples[from].pos.y) * f;
        const float deviation = std::fabs(samples[i].pos.y - line);

        if (deviation > worst)
        {
            worst = deviation;
            index = i;
        }
    }

    if (worst > error)
    {
        keep[index] = SAMPLE_BEND;
        KeepBends(samples, from, index, error, keep);
        KeepBends(samples, index, to, error, keep);
    }
}
} // namespace

void AmeisenNavigation::DetailPath(const dtNavMeshQuery* query, const dtQueryFilter* filter, Path& path,
                                   unsigned char* flags, const PathOptions& options, unsigned char* info) noexcept
{
    if (path.pointCount < 2)
    {
        return;
    }

    try
    {
        const float error = std::max(options.waypointHeightError, MIN_WAYPOINT_HEIGHT_ERROR);
        float spacing = std::clamp(options.waypointSpacing, MIN_WAYPOINT_SPACING, MAX_WAYPOINT_SPACING);
        constexpr float SAMPLE_STEP = 0.5f; // profile resolution (about two navmesh cells)
        constexpr int MAX_VISITED = 512;
        dtPolyRef visited[MAX_VISITED];

        // Each segment's kept samples (without its start), computed once; the uniform fill depends on the spacing.
        struct Segment
        {
            std::vector<ProfileSample> samples; // empty: not detailed (off-mesh connection, raycast failed)
            std::vector<unsigned char> keep;    // SampleKind
        };

        std::vector<Segment> segments(static_cast<size_t>(path.pointCount - 1));

        for (int i = 0; i + 1 < path.pointCount; ++i)
        {
            const Vector3 a = path[i];
            const Vector3 b = path[i + 1];
            const float length = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.z - a.z) * (b.z - a.z));

            if ((flags[i] & PATH_POINT_OFFMESH) || length < 0.5f)
            {
                continue;
            }

            PolyPosition start;
            dtRaycastHit hit{};

            if (dtStatusFailed(query->findNearestPoly(a, RAYCAST_POLY_EXTENTS, filter, &start.poly, start.pos)) ||
                !start.poly)
            {
                continue;
            }

            // Along the navmesh edge the raycast stops early: the samples past its end find their polygon themselves.
            const int count = Raycast(query, filter, start, b, hit, visited, MAX_VISITED) ? hit.pathCount : 0;
            const int n = std::max(1, static_cast<int>(std::ceil(length / SAMPLE_STEP)));
            Segment& segment = segments[static_cast<size_t>(i)];
            segment.samples.reserve(static_cast<size_t>(n) + 1);
            int hint = 0;

            for (int k = 0; k <= n; ++k)
            {
                ProfileSample sample{};
                sample.t = static_cast<float>(k) / static_cast<float>(n);
                sample.pos = a + (b - a) * sample.t;
                sample.onMesh = SurfaceAt(query, filter, visited, count, hint, sample.pos, sample.area);

                if (k == 0 || k == n)
                {
                    sample.pos = k == 0 ? a : b; // the ends stay exactly where they are
                }

                segment.samples.push_back(sample);
            }

            // Area changes (water, road, floor of a building, faction border): a point on each side of the border,
            // found by bisection between the two samples.
            std::vector<ProfileSample> refined;
            refined.reserve(segment.samples.size() + 8);
            refined.push_back(segment.samples[0]);
            std::vector<unsigned char> forced{SAMPLE_END};

            for (size_t k = 1; k < segment.samples.size(); ++k)
            {
                const ProfileSample& previous = segment.samples[k - 1];
                const ProfileSample& current = segment.samples[k];

                if (previous.onMesh && current.onMesh && previous.area != current.area)
                {
                    float lo = previous.t;
                    float hi = current.t;
                    ProfileSample before = previous;
                    ProfileSample after = current;

                    for (int iteration = 0; iteration < 6; ++iteration)
                    {
                        ProfileSample mid{};
                        mid.t = (lo + hi) * 0.5f;
                        mid.pos = a + (b - a) * mid.t;
                        mid.onMesh = SurfaceAt(query, filter, visited, count, hint, mid.pos, mid.area);

                        if (!mid.onMesh)
                        {
                            break;
                        }

                        if (mid.area == previous.area)
                        {
                            lo = mid.t;
                            before = mid;
                        }
                        else
                        {
                            hi = mid.t;
                            after = mid;
                        }
                    }

                    if (before.t > refined.back().t + 1e-4f)
                    {
                        refined.push_back(before);
                        forced.push_back(SAMPLE_BORDER);
                    }
                    else
                    {
                        forced.back() = SAMPLE_BORDER;
                    }

                    if (after.t < current.t - 1e-4f)
                    {
                        refined.push_back(after);
                        forced.push_back(SAMPLE_END);
                    }
                }

                refined.push_back(current);
                forced.push_back(k + 1 == segment.samples.size() ? SAMPLE_END : SAMPLE_SKIP);
            }

            segment.samples = std::move(refined);
            segment.keep = std::move(forced);

            // Height bends between the forced points.
            size_t from = 0;

            for (size_t k = 1; k < segment.samples.size(); ++k)
            {
                if (segment.keep[k] == SAMPLE_END || segment.keep[k] == SAMPLE_BORDER)
                {
                    KeepBends(segment.samples, from, k, error, segment.keep);
                    from = k;
                }
            }
        }

        // Assemble; a path that doesn't fit gets a larger spacing, then no fill at all.
        std::vector<Vector3> points;
        std::vector<unsigned char> pointFlags;
        std::vector<unsigned char> pointInfo;

        for (int attempt = 0; attempt < 8; ++attempt)
        {
            const bool fill = attempt < 7;
            points.clear();
            pointFlags.clear();
            pointInfo.clear();
            points.push_back(path[0]);
            pointFlags.push_back(flags[0]);
            pointInfo.push_back(0);

            for (int i = 0; i + 1 < path.pointCount; ++i)
            {
                const Segment& segment = segments[static_cast<size_t>(i)];

                if (!segment.samples.empty())
                {
                    const float length = path[i].DistanceTo(path[i + 1]);
                    const ProfileSample* last = &segment.samples[0];

                    for (size_t k = 1; k + 1 < segment.samples.size(); ++k)
                    {
                        const ProfileSample& sample = segment.samples[k];
                        const bool gap = fill && (sample.t - last->t) * length >= spacing - SAMPLE_STEP * 0.5f;

                        // Samples off the navmesh (a polygon edge between two samples) are never added.
                        if (sample.onMesh && (segment.keep[k] != SAMPLE_SKIP || gap))
                        {
                            points.push_back(sample.pos);
                            pointFlags.push_back(0);
                            pointInfo.push_back(segment.keep[k] == SAMPLE_BORDER ? POINT_INFO_SURFACE_CHANGE
                                                : segment.keep[k] == SAMPLE_BEND ? POINT_INFO_SLOPE_CHANGE
                                                : segment.keep[k] == SAMPLE_SKIP ? POINT_INFO_SPACING
                                                                                 : 0);
                            last = &sample;
                        }
                    }
                }

                points.push_back(path[i + 1]);
                pointFlags.push_back(flags[i + 1]);
                pointInfo.push_back(0);
            }

            if (static_cast<int>(points.size()) <= path.maxSize)
            {
                // Bends: a crest or a dip where the slope changes sign, else a change of slope.
                constexpr float FLAT = 0.02f; // rise per yard below which a slope counts as flat

                for (size_t k = 1; k + 1 < points.size(); ++k)
                {
                    if (!(pointInfo[k] & POINT_INFO_SLOPE_CHANGE))
                    {
                        continue;
                    }

                    const auto slope = [](const Vector3& from, const Vector3& to) {
                        const float run =
                            std::sqrt((to.x - from.x) * (to.x - from.x) + (to.z - from.z) * (to.z - from.z));
                        return run > 1e-3f ? (to.y - from.y) / run : 0.0f;
                    };

                    const float before = slope(points[k - 1], points[k]);
                    const float after = slope(points[k], points[k + 1]);

                    if (before > FLAT && after < -FLAT)
                    {
                        pointInfo[k] = POINT_INFO_CREST;
                    }
                    else if (before < -FLAT && after > FLAT)
                    {
                        pointInfo[k] = POINT_INFO_DIP;
                    }
                }

                // Added points keep their distance from the navmesh edge (cliffs, docks, forbidden water); the
                // ground changes stay where the ground changes. Each move has to stay on the navmesh both ways.
                if (options.edgeMargin > 0.0f)
                {
                    const float margin = std::min(options.edgeMargin, MAX_WALL_MARGIN);

                    for (size_t k = 1; k + 1 < points.size(); ++k)
                    {
                        if (pointFlags[k] != 0 || pointInfo[k] == 0 || (pointInfo[k] & POINT_INFO_SURFACE_CHANGE))
                        {
                            continue;
                        }

                        Vector3 moved;

                        if (PushFromEdge(query, filter, points[k], points[k - 1], points[k + 1], margin, moved) &&
                            SegmentClear(query, filter, points[k - 1], moved) &&
                            SegmentClear(query, filter, moved, points[k + 1]))
                        {
                            points[k] = moved;
                        }
                    }
                }

                std::copy(points.begin(), points.end(), path.points);
                std::copy(pointFlags.begin(), pointFlags.end(), flags);

                if (info)
                {
                    std::copy(pointInfo.begin(), pointInfo.end(), info);
                }

                path.pointCount = static_cast<int>(points.size());
                return;
            }

            spacing *= 2.0f;
        }

        // Even the bends don't fit: keep the plain path.
    }
    catch (...)
    {
    }
}

bool AmeisenNavigation::PushFromEdge(const dtNavMeshQuery* query, const dtQueryFilter* filter, const Vector3& point,
                                     const Vector3& previous, const Vector3& next, float margin, Vector3& out) noexcept
{
    PolyPosition start;

    if (dtStatusFailed(query->findNearestPoly(point, RAYCAST_POLY_EXTENTS, filter, &start.poly, start.pos)) ||
        !start.poly)
    {
        return false;
    }

    // Room to either side of the path (across it), up to twice the margin: a raycast along the navmesh. The wall
    // side of a point sitting on the edge has none, findDistanceToWall gives no direction there.
    const float dx = next.x - previous.x;
    const float dz = next.z - previous.z;
    const float length = Length2D(dx, dz);

    if (length < 1e-3f)
    {
        return false;
    }

    const Vector3 side{-dz / length, 0.0f, dx / length};
    const float reach = margin * 2.0f;
    float room[2]{reach, reach};

    for (int i = 0; i < 2; ++i)
    {
        const float sign = i == 0 ? 1.0f : -1.0f;
        dtRaycastHit hit{};

        if (!Raycast(query, filter, start, start.pos + side * (sign * reach), hit))
        {
            room[i] = 0.0f;
        }
        else if (hit.t != FLT_MAX)
        {
            room[i] = hit.t * reach;
        }
    }

    const int nearSide = room[0] < room[1] ? 0 : 1;

    if (room[nearSide] >= margin - 0.01f)
    {
        return false; // far enough from both edges
    }

    // Towards the far side, up to the margin or the middle of the passage.
    const float move = std::min(margin - room[nearSide], std::max(0.0f, (room[1 - nearSide] - room[nearSide]) * 0.5f));

    if (move < 0.05f)
    {
        return false;
    }

    out = start.pos + side * (nearSide == 0 ? -move : move);
    return SnapToSurface(query, filter, out) && out.DistanceTo(start.pos) > 0.05f;
}

void AmeisenNavigation::GroundPoints(int mapId, Path& path) noexcept
{
    World::StaticWorld* world = Worlds.Get(mapId);

    if (!world)
    {
        return;
    }

    try
    {
        std::vector<World::Surface> surfaces;

        for (int i = 0; i < path.pointCount; ++i)
        {
            Vector3 wow = path[i];
            wow.ToWowCoords();
            uint32_t areaId = 0;

            // The navmesh lies up to a few cell heights above the geometry it was built from, never far below it.
            if (!world->Surfaces(wow.x, wow.y, wow.z + 1.0f, wow.z - 2.5f, 0, surfaces, areaId) || surfaces.empty())
            {
                continue;
            }

            const World::Surface* best = nullptr;

            for (const World::Surface& surface : surfaces)
            {
                if ((surface.standable || surface.liquid) &&
                    (!best || std::fabs(surface.z - wow.z) < std::fabs(best->z - wow.z)))
                {
                    best = &surface;
                }
            }

            if (best)
            {
                path[i].y = best->z;
            }
        }
    }
    catch (...)
    {
    }
}

void AmeisenNavigation::DescribePoints(const dtNavMeshQuery* query, const dtQueryFilter* filter, const Path& path,
                                       unsigned char* info, unsigned char* areas) noexcept
{
    for (int i = 0; i < path.pointCount; ++i)
    {
        dtPolyRef ref = 0;
        float nearest[3]{};
        unsigned char area = 0;

        if (IsValidPosition(path[i]) &&
            dtStatusSucceed(query->findNearestPoly(path[i], RAYCAST_POLY_EXTENTS, filter, &ref, nearest)) && ref)
        {
            query->getAttachedNavMesh()->getPolyArea(ref, &area);
        }

        areas[i] = area;
    }

    // Ground changes between two points that DETAIL didn't mark (corners, paths without DETAIL).
    for (int i = 0; i + 1 < path.pointCount; ++i)
    {
        if (areas[i] && areas[i + 1] && areas[i] != areas[i + 1])
        {
            info[i] |= POINT_INFO_SURFACE_CHANGE;
        }
    }
}
