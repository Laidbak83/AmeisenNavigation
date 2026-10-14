#pragma once

#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <set>
#include <span>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "AmeisenNavigation.hpp"

/// A game object the client sees (WoW coordinates).
struct WorldObject
{
    uint64_t guid = 0;
    uint32_t displayId = 0;
    Vector3 position;
    float rotation[4]{}; // quaternion x, y, z, w (all 0 = none)
    float scale = 1.0f;
    bool blocking = true; // false: in the world but not in the way (an open door)
};

/// Game objects added at runtime: their collision (models of gameobjects.ano by display id) goes into the static
/// world queries at once and into the navmesh by rebuilding the tiles around them in the background (the export
/// settings and area rects come from the map's .anw). Thread-safe.
class DynamicWorld
{
public:
    DynamicWorld(AmeisenNavigation& navigation, std::filesystem::path folder);
    ~DynamicWorld();

    DynamicWorld(const DynamicWorld&) = delete;
    DynamicWorld& operator=(const DynamicWorld&) = delete;

    /// Add, move or change objects and remove others (by guid). Returns the number of objects whose model is known
    /// (others are ignored), -1 if the map can't be rebuilt (no .anw with build settings next to the navmesh).
    int SetObjects(int mapId, std::span<const WorldObject> objects, std::span<const uint64_t> removed);

    /// Tiles waiting to be rebuilt or being rebuilt.
    size_t PendingTiles() const;

    /// Wait until every pending tile is rebuilt (tests, shutdown). False on timeout.
    bool WaitIdle(std::chrono::milliseconds timeout);

    /// Objects on a map.
    size_t ObjectCount(int mapId) const;

    /// Most objects per map.
    static constexpr size_t MAX_OBJECTS_PER_MAP = 4096;

private:
    struct Object
    {
        WorldObject spawn;
        std::shared_ptr<const World::DynamicMesh> mesh; // RD, world space
    };

    struct MapState
    {
        bool checked = false;
        bool rebuildable = false;
        std::unordered_map<uint64_t, Object> objects;
    };

    AmeisenNavigation& Navigation;
    std::filesystem::path Folder;

    mutable std::mutex Mutex;
    std::condition_variable Cv;
    std::condition_variable IdleCv;
    std::unordered_map<int, MapState> Maps;
    std::set<std::tuple<int, int, int>> Dirty; // (map, adtX, adtY)
    size_t Running = 0;
    bool Stop = false;
    std::thread Worker;

    // Models by display id, read once.
    std::mutex ModelsMutex;
    std::unique_ptr<AnpWorld::WorldReader> Models;
    std::unordered_map<uint32_t, uint32_t> Displays; // display id -> model
    std::unordered_map<uint32_t, std::shared_ptr<const AnpWorld::Tile>> ModelCache;
    bool ModelsLoaded = false;

    std::shared_ptr<const AnpWorld::Tile> Model(uint32_t displayId);
    std::shared_ptr<const World::DynamicMesh> MakeMesh(const WorldObject& object);
    bool Rebuildable(int mapId, MapState& state);
    void MarkDirty(int mapId, const World::DynamicMesh& mesh);
    void PublishMeshes(int mapId, const MapState& state);
    void Run();
    void Rebuild(int mapId, int adtX, int adtY);
};
