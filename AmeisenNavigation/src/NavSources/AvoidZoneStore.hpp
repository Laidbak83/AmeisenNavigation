#pragma once

#include <charconv>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "../Utils/Logger.hpp"
#include "NavQueryFilter.hpp"

/// Avoid zones every client gets (REPORT_STUCK), per map. Readers get an immutable snapshot, a change replaces it
/// (copy on write), so clients notice changes by the pointer and queries never lock while searching.
///
/// Optionally persisted to a text file, one zone per line in WoW coordinates: "mapId x y z radius cost".
class AvoidZoneStore
{
public:
    /// Most zones kept per map (oldest are dropped).
    static constexpr size_t MAX_ZONES_PER_MAP = 4096;

    using Snapshot = std::shared_ptr<const std::vector<AvoidZone>>;

private:
    mutable std::shared_mutex Mutex;
    std::unordered_map<int, Snapshot> Maps;
    std::filesystem::path File;

public:
    AvoidZoneStore() = default;

    /// Zones of a map, nullptr if none.
    Snapshot Get(int mapId) const
    {
        std::shared_lock lock(Mutex);
        const auto it = Maps.find(mapId);
        return it != Maps.end() ? it->second : nullptr;
    }

    size_t Count(int mapId) const
    {
        const Snapshot snapshot = Get(mapId);
        return snapshot ? snapshot->size() : 0;
    }

    /// Load zones from a file and append new ones to it from now on. Returns the number of zones loaded, a
    /// missing file is fine (created with the first zone).
    size_t Open(const std::filesystem::path& file)
    {
        File = file;
        std::ifstream in(file);
        size_t loaded = 0;

        for (std::string line; std::getline(in, line);)
        {
            if (line.empty() || line[0] == '#')
            {
                continue;
            }

            std::istringstream fields(line);
            int mapId = 0;
            Vector3 wow;
            AvoidZone zone;

            if (!(fields >> mapId >> wow.x >> wow.y >> wow.z >> zone.radius >> zone.cost) || !Valid(wow, zone))
            {
                LogW("Skipping invalid avoid zone in ", file.string(), ": ", line);
                continue;
            }

            wow.CopyToRDCoords(zone.center);
            Insert(mapId, zone, false);
            loaded++;
        }

        return loaded;
    }

    /// Add a zone (center in RD coordinates). A zone around a position already inside an existing zone's inner half
    /// raises that zone's cost instead (repeated reports at the same spot). Returns false for invalid zones.
    bool Add(int mapId, const AvoidZone& zone) { return Insert(mapId, zone, true); }

    static bool Valid(const Vector3& center, const AvoidZone& zone) noexcept
    {
        return center.IsFinite() && std::isfinite(zone.radius) && zone.radius > 0.0f
               && zone.radius <= MAX_AVOID_ZONE_RADIUS && std::isfinite(zone.cost) && zone.cost >= 1.0f
               && zone.cost <= MAX_AVOID_ZONE_COST;
    }

private:
    bool Insert(int mapId, const AvoidZone& zone, bool persist)
    {
        if (!Valid(zone.center, zone))
        {
            return false;
        }

        std::unique_lock lock(Mutex);
        const Snapshot current = Maps[mapId];
        auto next = std::make_shared<std::vector<AvoidZone>>(current ? *current : std::vector<AvoidZone>{});
        bool merged = false;

        for (AvoidZone& existing : *next)
        {
            if (dtVdist2D(existing.center, zone.center) <= existing.radius * 0.5f
                && std::fabs(existing.center.y - zone.center.y) <= std::max(existing.radius, AVOID_ZONE_MIN_HALF_HEIGHT))
            {
                existing.cost = std::min(std::max(existing.cost, zone.cost) * 1.5f, MAX_AVOID_ZONE_COST);
                merged = true;
                break;
            }
        }

        if (!merged)
        {
            if (next->size() >= MAX_ZONES_PER_MAP)
            {
                next->erase(next->begin());
            }

            next->push_back(zone);
        }

        Maps[mapId] = std::move(next);

        if (persist && !File.empty())
        {
            Vector3 wow = zone.center;
            wow.ToWowCoords();
            std::ofstream out(File, std::ios::app);
            out << std::fixed << std::setprecision(3) << mapId << ' ' << wow.x << ' ' << wow.y << ' ' << wow.z << ' ' << zone.radius << ' ' << zone.cost
                << '\n';

            if (!out)
            {
                LogW("Failed to append to ", File.string());
            }
        }

        return true;
    }
};
