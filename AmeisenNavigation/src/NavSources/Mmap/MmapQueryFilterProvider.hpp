#pragma once

#include "../../../../AmeisenNavigation.Pack/src/AnpFormat.hpp"
#include "../../Clients/ClientState.hpp"
#include "../IQueryFilterProvider.hpp"
#include "335a/NavArea335a.hpp"
#include "548/NavArea548.hpp"
#include "MmapFormat.hpp"

/// Default dtQueryFilters for TrinityCore/SkyFire MMAPs. MMAPs have no faction areas, so the faction
/// states behave like NORMAL. DEAD walks through bad liquids at base cost.
class MmapQueryFilterProvider : public IQueryFilterProvider
{
public:
    explicit MmapQueryFilterProvider(MmapFormat format, float waterCost = 1.3f, float badLiquidCost = 4.0f) noexcept
    {
        if (format == MmapFormat::SF548)
        {
            Sf548Config(waterCost, badLiquidCost);
        }
        else
        {
            Tc335aConfig(waterCost, badLiquidCost);
        }
    }

private:
    void Tc335aConfig(float waterCost, float badLiquidCost) noexcept
    {
        // Off-mesh links added by the server use the ANP link flags (unused by TrinityCore).
        const auto includeFlags = static_cast<unsigned short>(static_cast<unsigned short>(NavFlag335a::GROUND)
                                                              | static_cast<unsigned short>(NavFlag335a::WATER)
                                                              | static_cast<unsigned short>(NavFlag335a::MAGMA_SLIME)
                                                              | NAV_DROP);
        const auto excludeFlags = static_cast<unsigned short>(static_cast<unsigned short>(NavFlag335a::GROUND_STEEP)
                                                              | NAV_JUMP);

        for (int i = 0; i < ClientStateCount; ++i)
        {
            auto& filter = Filter(static_cast<ClientState>(i));
            filter.setIncludeFlags(includeFlags);
            filter.setExcludeFlags(excludeFlags);
            filter.setAreaCost(AREA_OFFMESH_JUMP, 2.0f);

            if (static_cast<ClientState>(i) != ClientState::DEAD)
            {
                // Costs are indexed by area id (not by flag).
                filter.setAreaCost(static_cast<int>(NavArea335a::WATER), waterCost);
                filter.setAreaCost(static_cast<int>(NavArea335a::MAGMA_SLIME), badLiquidCost);
            }
        }
    }

    void Sf548Config(float waterCost, float badLiquidCost) noexcept
    {
        const auto includeFlags = static_cast<unsigned short>(
            static_cast<unsigned short>(NavArea548::GROUND) | static_cast<unsigned short>(NavArea548::WATER)
            | static_cast<unsigned short>(NavArea548::MAGMA) | static_cast<unsigned short>(NavArea548::SLIME)
            | NAV_DROP);

        for (int i = 0; i < ClientStateCount; ++i)
        {
            auto& filter = Filter(static_cast<ClientState>(i));
            filter.setIncludeFlags(includeFlags);
            filter.setExcludeFlags(NAV_JUMP);
            filter.setAreaCost(AREA_OFFMESH_JUMP, 2.0f);

            if (static_cast<ClientState>(i) != ClientState::DEAD)
            {
                filter.setAreaCost(static_cast<int>(NavArea548::WATER), waterCost);
                filter.setAreaCost(static_cast<int>(NavArea548::MAGMA), badLiquidCost);
                filter.setAreaCost(static_cast<int>(NavArea548::SLIME), badLiquidCost);
            }
        }
    }
};
