#include "AmeisenNavigation.hpp"

#include <cfloat>
#include <cmath>
#include <vector>

#include <DetourLocalBoundary.h>
#include <DetourObstacleAvoidance.h>

namespace {
/// Zones are passed this much wider than their radius (plus a constant), so the path keeps a little distance.
constexpr float ZONE_DETOUR_SCALE = 1.15f;
constexpr float ZONE_DETOUR_EXTRA = 0.5f;
constexpr float ZONE_DETOUR_STEP = 0.3927f; // radians between detour points (22.5 degrees)

/// Obstacles further away than this (plus the speed over the avoidance horizon) are ignored by Steer.
constexpr float STEER_RANGE = 12.0f;

struct ObstacleAvoidanceDeleter
{
    void operator()(dtObstacleAvoidanceQuery* query) const noexcept { dtFreeObstacleAvoidanceQuery(query); }
};

/// One avoidance query per thread (each connection is served by one thread).
dtObstacleAvoidanceQuery* AvoidanceQuery() noexcept
{
    thread_local std::unique_ptr<dtObstacleAvoidanceQuery, ObstacleAvoidanceDeleter> query = [] {
        std::unique_ptr<dtObstacleAvoidanceQuery, ObstacleAvoidanceDeleter> q(dtAllocObstacleAvoidanceQuery());

        if (q && !q->init(MAX_STEER_UNITS + MAX_AVOID_ZONES, 16)) // dtLocalBoundary keeps at most 8 segments
        {
            q.reset();
        }

        return q;
    }();

    return query.get();
}

bool InsideZone2D(const Vector3& p, const AvoidZone& zone) noexcept
{
    return dtVdist2D(p, zone.center) < zone.radius;
}
} // namespace

bool AmeisenNavigation::SetAvoidZones(size_t clientId, int mapId, std::span<const AvoidZone> zones)
{
    const auto client = GetClient(clientId);

    if (!client || mapId < 0 || mapId > NavMeshCache::MAX_MAP_ID || zones.size() > static_cast<size_t>(MAX_AVOID_ZONES))
    {
        return false;
    }

    std::vector<AvoidZone> rd;
    rd.reserve(zones.size());

    for (const AvoidZone& zone : zones)
    {
        if (!IsValidPosition(zone.center) || !AvoidZoneStore::Valid(zone.center, zone))
        {
            return false;
        }

        AvoidZone converted = zone;
        zone.center.CopyToRDCoords(converted.center);
        rd.push_back(converted);
    }

    // The client is used by one thread at a time: its connection's.
    return client->SetAvoidZones(mapId, rd);
}

bool AmeisenNavigation::ReportStuck(int mapId, const Vector3& position, float radius)
{
    if (mapId < 0 || mapId > NavMeshCache::MAX_MAP_ID || !IsValidPosition(position) || !std::isfinite(radius))
    {
        return false;
    }

    AvoidZone zone;
    position.CopyToRDCoords(zone.center);
    zone.radius = radius > 0.0f ? std::min(radius, MAX_AVOID_ZONE_RADIUS) : Settings.stuckZoneRadius;
    zone.cost = Settings.stuckZoneCost;

    if (!GlobalZones.Add(mapId, zone))
    {
        return false;
    }

    LogI("Stuck report on map ", mapId, " at ", position, ", ", GlobalZones.Count(mapId), " zones on this map");
    return true;
}

void AmeisenNavigation::LeadAroundZones(const dtNavMeshQuery* query, const dtQueryFilter* filter, Path& path,
                                        Path& scratch) noexcept
{
    const auto* navFilter = dynamic_cast<const NavQueryFilter*>(filter);

    if (!navFilter || navFilter->GetZones().empty() || path.pointCount < 2)
    {
        return;
    }

    const std::span<const AvoidZone> zones = navFilter->GetZones();
    scratch.Clear();
    scratch.TryAppend(path[0]);
    std::vector<Vector3> detour;

    for (int i = 1; i < path.pointCount; ++i)
    {
        const Vector3 a = scratch[scratch.pointCount - 1];
        const Vector3 b = path[i];
        bool replaced = false;

        for (const AvoidZone& zone : zones)
        {
            // A segment starting or ending inside a zone can't avoid it.
            if (SegmentLengthInZone(a, b, zone) < 0.1f || InsideZone2D(a, zone) || InsideZone2D(b, zone))
            {
                continue;
            }

            // Entry and exit angles, then the short way round (the long way if that is blocked).
            const float dx = b.x - a.x;
            const float dz = b.z - a.z;
            const float fx = a.x - zone.center.x;
            const float fz = a.z - zone.center.z;
            const float qa = dx * dx + dz * dz;
            const float qb = 2.0f * (fx * dx + fz * dz);
            const float qc = fx * fx + fz * fz - zone.radius * zone.radius;
            const float root = std::sqrt(std::max(0.0f, qb * qb - 4.0f * qa * qc));
            const float t0 = (-qb - root) / (2.0f * qa);
            const float t1 = (-qb + root) / (2.0f * qa);
            const float angle0 = std::atan2(fz + dz * t0, fx + dx * t0);
            float angle1 = std::atan2(fz + dz * t1, fx + dx * t1);

            while (angle1 - angle0 > 3.14159265f)
            {
                angle1 -= 6.2831853f;
            }

            while (angle1 - angle0 < -3.14159265f)
            {
                angle1 += 6.2831853f;
            }

            const float detourRadius = zone.radius * ZONE_DETOUR_SCALE + ZONE_DETOUR_EXTRA;

            for (const float sweep : {angle1 - angle0, angle1 - angle0 > 0.0f ? angle1 - angle0 - 6.2831853f
                                                                                : angle1 - angle0 + 6.2831853f})
            {
                const int steps = std::max(1, static_cast<int>(std::ceil(std::fabs(sweep) / ZONE_DETOUR_STEP)));
                detour.clear();

                for (int k = 0; k <= steps; ++k)
                {
                    const float angle = angle0 + sweep * static_cast<float>(k) / static_cast<float>(steps);
                    const float h = a.y + (b.y - a.y) * (t0 + (t1 - t0) * static_cast<float>(k) / steps);
                    detour.emplace_back(zone.center.x + std::cos(angle) * detourRadius, h,
                                        zone.center.z + std::sin(angle) * detourRadius);
                }

                bool valid = scratch.GetSpace() > static_cast<int>(detour.size()) + (path.pointCount - i);
                constexpr float extents[3]{1.0f, 3.0f, 1.0f};

                for (size_t k = 0; valid && k < detour.size(); ++k)
                {
                    dtPolyRef ref = 0;
                    Vector3 nearest;
                    valid = dtStatusSucceed(query->findNearestPoly(detour[k], extents, filter, &ref, nearest)) && ref
                            && dtVdist2D(nearest, zone.center) >= zone.radius;
                    detour[k] = nearest;
                    valid = valid && SegmentClear(query, filter, k == 0 ? a : detour[k - 1], detour[k]);
                }

                if (valid && SegmentClear(query, filter, detour.back(), b))
                {
                    for (const Vector3& v : detour)
                    {
                        scratch.TryAppendUnique(v);
                    }

                    replaced = true;
                    break;
                }
            }

            if (replaced)
            {
                break; // the next segment starts at the detour's end and is checked against the other zones
            }
        }

        if (!scratch.TryAppendUnique(b))
        {
            return; // full, keep the path as it was
        }
    }

    if (scratch.pointCount <= path.maxSize)
    {
        std::copy(scratch.begin(), scratch.end(), path.points);
        path.pointCount = scratch.pointCount;
    }
}

bool AmeisenNavigation::Steer(size_t clientId, int mapId, const Vector3& position, const Vector3& velocity,
                              const Vector3& target, float maxSpeed, float radius, float deltaTime,
                              std::span<const SteerUnit> units, SteerResult& result)
{
    result = SteerResult{};

    if (!velocity.IsFinite() || !std::isfinite(maxSpeed) || maxSpeed <= 0.0f || maxSpeed > MAX_STEER_SPEED
        || !std::isfinite(radius) || radius <= 0.0f || radius > MAX_STEER_RADIUS || !std::isfinite(deltaTime)
        || deltaTime <= 0.0f || deltaTime > MAX_STEER_DELTA_TIME || units.size() > static_cast<size_t>(MAX_STEER_UNITS)
        || std::fabs(velocity.x) > MAX_STEER_SPEED || std::fabs(velocity.y) > MAX_STEER_SPEED)
    {
        return false;
    }

    const auto ctx = GetQueryContext(clientId, mapId);
    dtObstacleAvoidanceQuery* avoidance = AvoidanceQuery();

    if (!ctx || !avoidance)
    {
        return false;
    }

    PolyPosition start;

    if (!FindNearestPolyWow(ctx.query, ctx.filter, position, start))
    {
        return false;
    }

    // Desired velocity: towards the first corner of the navmesh path to the target, slowing down on arrival.
    Vector3 desired;
    Path& corners = ctx.client->GetScratchPathBuffer();

    if (CalculateNormalPath(ctx.query, ctx.filter, *ctx.client, position, target, corners) && corners.pointCount > 0)
    {
        const Vector3 end = corners[corners.pointCount - 1];
        const float remaining = dtVdist2D(start.pos, end);

        for (int i = 0; i < corners.pointCount; ++i)
        {
            const float dx = corners[i].x - start.pos.x;
            const float dz = corners[i].z - start.pos.z;
            const float length = std::sqrt(dx * dx + dz * dz);

            if (length > 0.1f)
            {
                const float speed = std::min(maxSpeed, remaining / deltaTime);
                desired = Vector3{dx / length * speed, 0.0f, dz / length * speed};
                break;
            }
        }
    }

    // Obstacles: units, avoid zones and the navmesh borders around the position.
    const float range = STEER_RANGE + maxSpeed * 2.5f;
    Vector3 current;
    velocity.CopyToRDCoords(current);
    current.y = 0.0f;
    avoidance->reset();

    for (const SteerUnit& unit : units)
    {
        Vector3 pos;
        Vector3 vel;

        if (!ToValidRdCoords(unit.position, pos) || !unit.velocity.IsFinite() || !std::isfinite(unit.radius)
            || unit.radius <= 0.0f || unit.radius > MAX_STEER_RADIUS || std::fabs(unit.velocity.x) > MAX_STEER_SPEED
            || std::fabs(unit.velocity.y) > MAX_STEER_SPEED || dtVdist2D(pos, start.pos) > range + unit.radius
            || std::fabs(pos.y - start.pos.y) > 5.0f)
        {
            continue;
        }

        unit.velocity.CopyToRDCoords(vel);
        vel.y = 0.0f;
        avoidance->addCircle(pos, unit.radius, vel, vel);
    }

    if (const auto* navFilter = dynamic_cast<const NavQueryFilter*>(ctx.filter))
    {
        const Vector3 still;

        for (const AvoidZone& zone : navFilter->GetZones())
        {
            if (dtVdist2D(zone.center, start.pos) <= range + zone.radius && !InsideZone2D(start.pos, zone))
            {
                avoidance->addCircle(zone.center, zone.radius, still, still);
            }
        }
    }

    dtLocalBoundary boundary;
    boundary.update(start.poly, start.pos, range, ctx.query, ctx.filter);

    for (int i = 0; i < boundary.getSegmentCount(); ++i)
    {
        const float* segment = boundary.getSegment(i);
        avoidance->addSegment(segment, segment + 3);
    }

    dtObstacleAvoidanceParams params{};
    params.velBias = 0.4f;
    params.weightDesVel = 2.0f;
    params.weightCurVel = 0.75f;
    params.weightSide = 0.75f;
    params.weightToi = 2.5f;
    params.horizTime = 2.5f;
    params.gridSize = 33;
    params.adaptiveDivs = 7;
    params.adaptiveRings = 2;
    params.adaptiveDepth = 5;

    Vector3 chosen;
    avoidance->sampleVelocityAdaptive(start.pos, radius, maxSpeed, current, desired, chosen, &params);

    if (!chosen.IsFinite())
    {
        chosen = Vector3{};
    }

    // Where the velocity leads within the tick, constrained to the navmesh.
    PolyPosition moved = start;
    const Vector3 goal = start.pos + chosen * deltaTime;

    if (dtStatusFailed(MoveAlong(ctx.query, ctx.filter, moved, goal)))
    {
        moved = start;
    }

    result.velocity = chosen;
    result.velocity.ToWowCoords();
    result.position = moved.pos;
    result.position.ToWowCoords();
    return true;
}
