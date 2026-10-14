#include "AmeisenNavigation.hpp"

#include <cfloat>
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
                                            dtPolyRef* straightPathRefs, unsigned char* straightPathFlags) noexcept
{
    path.Clear();
    FollowState* state = client.GetFollowState();

    if (!state)
    {
        return CalculateNormalPath(query, filter, client, startPosition, endPosition, path, straightPathRefs, partial,
                                   straightPathFlags);
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

        dtPolyRef* polys = client.GetPolyPathBuffer();
        int polyCount = 0;
        const dtStatus status = query->findPath(start.poly, end.poly, start.pos, end.pos, filter, polys, &polyCount,
                                                client.GetPolyPathBufferSize());

        if (dtStatusFailed(status) || polyCount <= 0)
        {
            return false;
        }

        Vector3 target = end.pos;
        state->partial = polys[polyCount - 1] != end.poly;

        if (state->partial)
        {
            query->closestPointOnPoly(polys[polyCount - 1], end.pos, target, nullptr);
        }

        corridor.reset(start.poly, start.pos);
        corridor.setCorridor(target, polys, polyCount);
        state->mapId = mapId;
    }

    if (partial)
    {
        *partial = state->partial;
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
