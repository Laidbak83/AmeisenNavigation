#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <span>
#include <unordered_map>
#include <utility>

#include <DetourPathCorridor.h>

#include "../../../AmeisenNavigation.Pack/src/DetourUtils.hpp"
#include "../NavSources/IQueryFilterProvider.hpp"
#include "../NavSources/NavQueryFilter.hpp"
#include "../Utils/Path.hpp"
#include "ClientState.hpp"

/// Upper bound for area costs: big enough to effectively forbid an area, small enough that path costs stay finite.
constexpr float MAX_AREA_COST = 1e6f;

/// Area cost override sent by a client.
struct AreaCost
{
    unsigned char areaId;
    float cost;
};

/// Corridor of the client's last follow path (PathOptions::follow).
struct FollowState
{
    int mapId = -1; // -1 = no corridor
    bool partial = false;
    dtPathCorridor corridor;
};

/// Per-connection navigation state: the active query filter and reusable buffers. dtNavMeshQuery objects are
/// not per client, requests lease them from the map's NavMeshQueryPool.
///
/// A client is not thread-safe, it must only be used by one thread at a time (the server processes each
/// connection's requests sequentially on that connection's thread).
class AmeisenNavClient
{
    size_t Id;
    ClientState State;
    const IQueryFilterProvider* FilterProvider;

    // Per-client area cost overrides and area rules, empty = use the provider's default filter.
    std::optional<NavQueryFilter> CustomFilter;

    // Avoid zones per map (RD coordinates) and the filters combining them with the global zones.
    struct MapFilter
    {
        NavQueryFilter filter;
        std::shared_ptr<const std::vector<AvoidZone>> global; // snapshot the filter was built with
        bool dirty = true;
    };

    std::unordered_map<int, std::vector<AvoidZone>> Zones;
    std::unordered_map<int, MapFilter> MapFilters;

    // Reusable buffers for path calculation.
    int PolyPathBufferSize;
    int StraightPathRefsSize;
    std::unique_ptr<dtPolyRef[]> PolyPathBuffer;
    std::unique_ptr<dtPolyRef[]> StraightPathRefs;
    Path PrimaryPath;
    Path SecondaryPath;
    std::unique_ptr<FollowState> Follow;

public:
    AmeisenNavClient(size_t id, const IQueryFilterProvider* filterProvider, int polyPathBufferSize = 512,
                     int pointPathBufferSize = 256)
        : Id(id),
          State(ClientState::NORMAL),
          FilterProvider(filterProvider),
          CustomFilter(),
          PolyPathBufferSize(std::max(polyPathBufferSize, 1)),
          StraightPathRefsSize(std::max(pointPathBufferSize, 1)),
          PolyPathBuffer(std::make_unique<dtPolyRef[]>(static_cast<size_t>(PolyPathBufferSize))),
          StraightPathRefs(std::make_unique<dtPolyRef[]>(static_cast<size_t>(StraightPathRefsSize))),
          PrimaryPath(pointPathBufferSize),
          SecondaryPath(pointPathBufferSize)
    {
    }

    AmeisenNavClient(const AmeisenNavClient&) = delete;
    AmeisenNavClient& operator=(const AmeisenNavClient&) = delete;

    size_t GetId() const noexcept { return Id; }
    ClientState GetClientState() const noexcept { return State; }
    bool HasCustomFilter() const noexcept { return CustomFilter.has_value(); }

    /// The filter of this client without avoid zones.
    const dtQueryFilter* QueryFilter() const noexcept
    {
        return CustomFilter ? &*CustomFilter : FilterProvider->Get(State);
    }

    /// The filter for queries on a map: QueryFilter() plus the client's and the global avoid zones of the map.
    const dtQueryFilter* QueryFilter(int mapId, const std::shared_ptr<const std::vector<AvoidZone>>& global)
    {
        const auto own = Zones.find(mapId);
        const bool hasOwn = own != Zones.end() && !own->second.empty();
        const bool hasGlobal = global && !global->empty();

        if (!hasOwn && !hasGlobal)
        {
            return QueryFilter();
        }

        try
        {
            MapFilter& entry = MapFilters[mapId];

            if (entry.dirty || entry.global != global)
            {
                const dtQueryFilter* base = QueryFilter();
                entry.filter.SetBase(*base);
                entry.filter.SetAreaRules(CustomFilter ? CustomFilter->GetExcludedAreas() : 0,
                                          CustomFilter ? CustomFilter->GetAllowedAreas() : 0);
                entry.filter.SetZones(hasOwn ? std::span<const AvoidZone>(own->second) : std::span<const AvoidZone>(),
                                      hasGlobal ? std::span<const AvoidZone>(*global) : std::span<const AvoidZone>());
                entry.global = global;
                entry.dirty = false;
            }

            return &entry.filter;
        }
        catch (...)
        {
            return QueryFilter();
        }
    }

    /// Replace the client's avoid zones of a map (RD coordinates, empty = none). False if there are too many.
    bool SetAvoidZones(int mapId, std::span<const AvoidZone> zones)
    {
        if (zones.size() > static_cast<size_t>(MAX_AVOID_ZONES))
        {
            return false;
        }

        if (zones.empty())
        {
            Zones.erase(mapId);
            MapFilters.erase(mapId);
        }
        else
        {
            Zones[mapId].assign(zones.begin(), zones.end());
            MapFilters[mapId].dirty = true;
        }

        ResetFollow();
        return true;
    }

    std::span<const AvoidZone> GetAvoidZones(int mapId) const noexcept
    {
        const auto it = Zones.find(mapId);
        return it != Zones.end() ? std::span<const AvoidZone>(it->second) : std::span<const AvoidZone>();
    }

    int GetPolyPathBufferSize() const noexcept { return PolyPathBufferSize; }
    dtPolyRef* GetPolyPathBuffer() noexcept { return PolyPathBuffer.get(); }

    /// Poly refs of the straight path corners, sized like the path buffers.
    dtPolyRef* GetStraightPathRefBuffer() noexcept { return StraightPathRefs.get(); }
    int GetStraightPathRefBufferSize() const noexcept { return StraightPathRefsSize; }

    /// Two reusable path buffers (result + scratch for smoothing/validation).
    Path& GetPathBuffer() noexcept { return PrimaryPath; }
    Path& GetScratchPathBuffer() noexcept { return SecondaryPath; }

    /// The follow corridor (created on first use, nullptr if that fails).
    FollowState* GetFollowState() noexcept
    {
        if (!Follow)
        {
            auto state = std::unique_ptr<FollowState>(new (std::nothrow) FollowState());

            if (!state || !state->corridor.init(PolyPathBufferSize))
            {
                return nullptr;
            }

            Follow = std::move(state);
        }

        return Follow.get();
    }

    /// Forget the follow corridor (the next follow request searches again).
    void ResetFollow() noexcept
    {
        if (Follow)
        {
            Follow->mapId = -1;
        }
    }

    /// Select the base filter by state and apply area cost overrides on top of it.
    /// Returns false (and changes nothing) for invalid states, area ids or costs.
    bool ConfigureQueryFilter(ClientState state, std::span<const AreaCost> costs) noexcept
    {
        if (!IsValidClientState(state))
        {
            return false;
        }

        const dtQueryFilter* base = FilterProvider->Get(state);

        if (!base)
        {
            return false;
        }

        for (const auto& [areaId, cost] : costs)
        {
            // dtQueryFilter stores DT_MAX_AREAS costs, anything else would write out of bounds.
            // Negative/infinite costs would break the A* search, 0 forbids the area.
            if (areaId >= DT_MAX_AREAS || !std::isfinite(cost) || cost < 0.0f || cost > MAX_AREA_COST)
            {
                return false;
            }
        }

        State = state;
        ResetFollow(); // the corridor was searched with the old costs

        for (auto& entry : MapFilters)
        {
            entry.second.dirty = true;
        }

        if (costs.empty())
        {
            CustomFilter.reset();
            return true;
        }

        // Cost 0 forbids an area, a cost > 0 also allows it when the default filter excludes its polygons by flag
        // (e.g. steep slopes on TrinityCore MMAPs, jump links).
        CustomFilter.emplace(*base);
        uint64_t excluded = 0;
        uint64_t allowed = 0;

        for (const auto& [areaId, cost] : costs)
        {
            if (cost == 0.0f)
            {
                excluded |= uint64_t{1} << areaId;
                allowed &= ~(uint64_t{1} << areaId);
            }
            else
            {
                CustomFilter->setAreaCost(areaId, cost);
                allowed |= uint64_t{1} << areaId;
                excluded &= ~(uint64_t{1} << areaId);
            }
        }

        CustomFilter->SetAreaRules(excluded, allowed);
        return true;
    }
};
