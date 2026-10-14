#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>
#include <unordered_map>
#include <vector>

#include <DetourCommon.h>
#include <DetourNavMeshQuery.h>

#include "../Utils/Vector3.hpp"

/// A circle that costs more to cross (RD coordinates). Height: it applies to surfaces within
/// max(radius, AVOID_ZONE_MIN_HALF_HEIGHT) of the center, a bridge over a zone on the ground isn't affected.
struct AvoidZone
{
    Vector3 center;
    float radius = 0.0f;
    float cost = 1.0f; // travel cost multiplier inside the circle (>= 1)
};

constexpr int MAX_AVOID_ZONES = 64;            // per client and map
constexpr float MAX_AVOID_ZONE_RADIUS = 100.0f;
constexpr float MAX_AVOID_ZONE_COST = 10000.0f;
constexpr float AVOID_ZONE_MIN_HALF_HEIGHT = 5.0f;

/// Length of the part of segment a-b (x/z plane) inside the zone's circle, 0 if the segment is at another height.
inline float SegmentLengthInZone(const float* a, const float* b, const AvoidZone& zone) noexcept
{
    const float halfHeight = std::max(zone.radius, AVOID_ZONE_MIN_HALF_HEIGHT);

    if (std::min(a[1], b[1]) > zone.center.y + halfHeight || std::max(a[1], b[1]) < zone.center.y - halfHeight)
    {
        return 0.0f;
    }

    const float dx = b[0] - a[0];
    const float dz = b[2] - a[2];
    const float fx = a[0] - zone.center.x;
    const float fz = a[2] - zone.center.z;
    const float qa = dx * dx + dz * dz;
    const float qc = fx * fx + fz * fz - zone.radius * zone.radius;

    if (qa < 1e-8f)
    {
        return 0.0f;
    }

    const float qb = 2.0f * (fx * dx + fz * dz);
    const float disc = qb * qb - 4.0f * qa * qc;

    if (disc <= 0.0f)
    {
        return 0.0f;
    }

    const float root = std::sqrt(disc);
    const float t0 = std::max(0.0f, (-qb - root) / (2.0f * qa));
    const float t1 = std::min(1.0f, (-qb + root) / (2.0f * qa));
    return t1 > t0 ? (t1 - t0) * std::sqrt(qa) : 0.0f;
}

/// Query filter of a client: the base filter's flags and costs, plus areas excluded (CONFIGURE_FILTER cost 0) or
/// allowed regardless of the polygon flags (cost > 0 for an area the base filter excludes), plus avoid zones that
/// make crossing them more expensive.
class NavQueryFilter : public dtQueryFilter
{
    /// With many zones (stuck reports pile up) the cost function only looks at the zones in the grid cells a
    /// segment touches.
    static constexpr size_t GRID_MIN_ZONES = 16;
    static constexpr float GRID_CELL = 64.0f;

    uint64_t ExcludedAreas = 0;
    uint64_t AllowedAreas = 0;
    std::vector<AvoidZone> Zones;
    std::unordered_map<int64_t, std::vector<uint32_t>> Grid; // cell -> zone indices

    static int Cell(float v) noexcept { return static_cast<int>(std::floor(v / GRID_CELL)); }
    static int64_t Key(int x, int z) noexcept { return (static_cast<int64_t>(x) << 32) ^ static_cast<uint32_t>(z); }

    void BuildGrid()
    {
        Grid.clear();

        if (Zones.size() < GRID_MIN_ZONES)
        {
            return;
        }

        for (size_t i = 0; i < Zones.size(); ++i)
        {
            const AvoidZone& z = Zones[i];

            for (int x = Cell(z.center.x - z.radius); x <= Cell(z.center.x + z.radius); ++x)
            {
                for (int c = Cell(z.center.z - z.radius); c <= Cell(z.center.z + z.radius); ++c)
                {
                    Grid[Key(x, c)].push_back(static_cast<uint32_t>(i));
                }
            }
        }
    }

public:
    NavQueryFilter() = default;

    explicit NavQueryFilter(const dtQueryFilter& base) { SetBase(base); }

    /// Copy flags and area costs of a filter.
    void SetBase(const dtQueryFilter& base) noexcept
    {
        for (int i = 0; i < DT_MAX_AREAS; ++i)
        {
            setAreaCost(i, base.getAreaCost(i));
        }

        setIncludeFlags(base.getIncludeFlags());
        setExcludeFlags(base.getExcludeFlags());
    }

    void SetAreaRules(uint64_t excluded, uint64_t allowed) noexcept
    {
        ExcludedAreas = excluded;
        AllowedAreas = allowed & ~excluded;
    }

    void SetZones(std::span<const AvoidZone> a, std::span<const AvoidZone> b = {})
    {
        Zones.assign(a.begin(), a.end());
        Zones.insert(Zones.end(), b.begin(), b.end());
        BuildGrid();
    }

    std::span<const AvoidZone> GetZones() const noexcept { return Zones; }
    uint64_t GetExcludedAreas() const noexcept { return ExcludedAreas; }
    uint64_t GetAllowedAreas() const noexcept { return AllowedAreas; }

    bool passFilter(const dtPolyRef ref, const dtMeshTile* tile, const dtPoly* poly) const override
    {
        const uint64_t bit = uint64_t{1} << (poly->getArea() & 63);

        if (ExcludedAreas & bit)
        {
            return false;
        }

        if (AllowedAreas & bit)
        {
            return true;
        }

        return dtQueryFilter::passFilter(ref, tile, poly);
    }

    float getCost(const float* pa, const float* pb, const dtPolyRef prevRef, const dtMeshTile* prevTile,
                  const dtPoly* prevPoly, const dtPolyRef curRef, const dtMeshTile* curTile, const dtPoly* curPoly,
                  const dtPolyRef nextRef, const dtMeshTile* nextTile, const dtPoly* nextPoly) const override
    {
        float cost = dtQueryFilter::getCost(pa, pb, prevRef, prevTile, prevPoly, curRef, curTile, curPoly, nextRef,
                                            nextTile, nextPoly);

        if (Zones.empty())
        {
            return cost;
        }

        const float areaCost = getAreaCost(curPoly->getArea());

        if (Grid.empty())
        {
            for (const AvoidZone& zone : Zones)
            {
                cost += SegmentLengthInZone(pa, pb, zone) * areaCost * (zone.cost - 1.0f);
            }

            return cost;
        }

        // Segments are polygon sized, usually one or a few cells. A zone in several of them is counted once.
        const int x0 = Cell(std::min(pa[0], pb[0])), x1 = Cell(std::max(pa[0], pb[0]));
        const int z0 = Cell(std::min(pa[2], pb[2])), z1 = Cell(std::max(pa[2], pb[2]));

        if ((x1 - x0 + 1) * (z1 - z0 + 1) > 64)
        {
            for (const AvoidZone& zone : Zones)
            {
                cost += SegmentLengthInZone(pa, pb, zone) * areaCost * (zone.cost - 1.0f);
            }

            return cost;
        }

        thread_local std::vector<uint32_t> seen;
        seen.clear();

        for (int x = x0; x <= x1; ++x)
        {
            for (int z = z0; z <= z1; ++z)
            {
                const auto it = Grid.find(Key(x, z));

                if (it == Grid.end())
                {
                    continue;
                }

                for (const uint32_t index : it->second)
                {
                    if (std::find(seen.begin(), seen.end(), index) != seen.end())
                    {
                        continue;
                    }

                    seen.push_back(index);

                    const AvoidZone& zone = Zones[index];
                    cost += SegmentLengthInZone(pa, pb, zone) * areaCost * (zone.cost - 1.0f);
                }
            }
        }

        return cost;
    }
};
