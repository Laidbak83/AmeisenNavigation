#pragma once

#include <cstdint>
#include <filesystem>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../../../AmeisenNavigation.Pack/src/AnpWorld.hpp"
#include "../Utils/Vector3.hpp"

/// Static world: the collision geometry of a map with the provenance of every triangle (.anw files written by the
/// exporter), for the queries the navmesh can't answer. Positions in WoW coordinates.
namespace World {
/// Kinds of geometry a query ignores.
enum QueryFlags : uint32_t
{
    IGNORE_TERRAIN = 1u << 0,
    IGNORE_WMOS = 1u << 1,
    IGNORE_DOODADS = 1u << 2, // M2 models placed by the map (trees, rocks, fences, crates)
    IGNORE_LIQUIDS = 1u << 3,
    IGNORE_GAME_OBJECTS = 1u << 4,
};

/// Bit of a triangle's source in QueryFlags (liquid areas count as liquids whatever their source).
uint32_t IgnoreBit(uint32_t source, uint8_t area) noexcept;

/// What a ray or a body hit.
struct Hit
{
    bool hit = false;
    float t = 1.0f;      // fraction of start -> end before the hit (1 if clear)
    Vector3 position{};  // hit position (end if clear)
    Vector3 normal{};    // surface normal facing the ray (zero if clear)
    uint32_t source = 0; // AnpWorld::MakeSource(kind, id)
    uint8_t area = 0;    // navmesh area id of the triangle (0 = too steep to stand on)
};

/// A surface under a point.
struct Surface
{
    float z = 0.0f;
    Vector3 normal{}; // facing up
    uint32_t source = 0;
    uint8_t area = 0;
    bool standable = false; // walkable ground (not too steep, not liquid)
    bool liquid = false;
};

/// Collision mesh of an object added at runtime (game object), in RD coordinates.
struct DynamicMesh
{
    uint64_t guid = 0;
    uint32_t source = 0; // AnpWorld::MakeSource(GAME_OBJECT, displayId)
    std::vector<float> verts;
    std::vector<uint32_t> indices;
    std::vector<uint8_t> areas;
    float bmin[3]{};
    float bmax[3]{};
};

using DynamicMeshes = std::vector<std::shared_ptr<const DynamicMesh>>;

/// One ADT of the static world with a bounding volume hierarchy over its triangles (RD coordinates).
class WorldTile
{
public:
    struct Node
    {
        float bmin[3];
        float bmax[3];
        uint32_t start; // first triangle in Order (leaf) or right child (inner node, left child = this + 1)
        uint32_t count; // triangles of a leaf, 0 for inner nodes
    };

    explicit WorldTile(AnpWorld::Tile&& tile);

    const AnpWorld::Tile& Data() const noexcept { return Tile; }
    size_t MemoryBytes() const noexcept;

    /// Closest hit of the segment origin + t * delta, t in [0, tMax] (RD). Lowers tMax and fills the hit. Triangles are
    /// double sided. Returns true if something was hit.
    bool Raycast(const float* origin, const float* delta, float& tMax, uint32_t ignore, uint32_t& triangle) const;

    /// Every hit of the vertical segment at (rdX, rdZ) from rdYTop down to rdYBottom: (height, triangle).
    void VerticalHits(float rdX, float rdZ, float rdYTop, float rdYBottom, uint32_t ignore,
                      std::vector<std::pair<float, uint32_t>>& hits) const;

    /// Triangle normal (RD, unit length, not oriented).
    Vector3 Normal(uint32_t triangle) const noexcept;

    /// AreaTable id of the chunk holding a WoW position (0 if unknown).
    uint32_t ChunkArea(float wowX, float wowY) const noexcept;

private:
    AnpWorld::Tile Tile;
    std::vector<Node> Nodes;
    std::vector<uint32_t> Order;

    void Build();
    uint32_t BuildNode(const std::vector<float>& boxes, const std::vector<float>& centers, uint32_t start,
                       uint32_t count);
    bool Ignored(uint32_t triangle, uint32_t ignore) const noexcept
    {
        return ignore && (IgnoreBit(Tile.sources[triangle], Tile.areas[triangle]) & ignore);
    }
};

/// The static world of one map: tiles read on demand from its .anw file and kept in a small LRU cache.
/// Thread-safe.
class StaticWorld
{
public:
    StaticWorld(const std::filesystem::path& file, size_t maxCachedTiles);

    bool IsValid() const noexcept { return Reader && Reader->IsValid(); }

    /// The tile of an ADT, nullptr if the map has none there.
    std::shared_ptr<const WorldTile> GetTile(int adtX, int adtY);

    /// Cast a segment against the geometry. False if the world has no tile on the way (result unknown).
    bool Raycast(const Vector3& start, const Vector3& end, uint32_t ignore, Hit& hit);

    /// Every surface under (x, y) between zTop and zBottom, highest first, and the AreaTable id there. False if the
    /// world has no tile there.
    bool Surfaces(float x, float y, float zTop, float zBottom, uint32_t ignore, std::vector<Surface>& surfaces,
                  uint32_t& areaId);

    /// Move a standing body (vertical cylinder of radius and height, feet at the positions) from start to end: the
    /// first geometry it runs into above stepHeight (steps and slopes below it are walked over). Approximated with
    /// rays at three heights along the center and both sides, the body's front included. False if the world has no
    /// tile on the way.
    bool Sweep(const Vector3& start, const Vector3& end, float radius, float height, float stepHeight, uint32_t ignore,
               Hit& hit);

    size_t CachedTiles() const;

    /// Objects added at runtime: every query sees them too (replaced as a whole, copy on write).
    void SetDynamicMeshes(std::shared_ptr<const DynamicMeshes> meshes);
    std::shared_ptr<const DynamicMeshes> GetDynamicMeshes() const;

    /// Raw access for rebuilding navmesh tiles (not cached).
    std::optional<AnpWorld::Tile> ReadRawTile(int adtX, int adtY)
    {
        return Reader ? Reader->ReadTile(adtX, adtY) : std::nullopt;
    }
    std::optional<std::vector<unsigned char>> ReadEntry(const std::string& name, uint64_t maxSize)
    {
        return Reader ? Reader->ReadEntry(name, maxSize) : std::nullopt;
    }

    /// Longest segment Raycast and Sweep accept (yards).
    static constexpr float MAX_SEGMENT_LENGTH = 1000.0f;

private:
    std::unique_ptr<AnpWorld::WorldReader> Reader;
    size_t MaxTiles;
    mutable std::mutex Mutex;
    std::list<std::pair<int, std::shared_ptr<const WorldTile>>> Lru; // most recent first
    std::unordered_map<int, decltype(Lru)::iterator> Index;
    std::unordered_map<int, bool> Missing; // ADTs without a tile
    std::shared_ptr<const DynamicMeshes> Dynamic;

    /// Tiles a segment's bounding box touches (WoW coordinates), false if none exists.
    bool TilesAlong(const Vector3& a, const Vector3& b, float margin,
                    std::vector<std::shared_ptr<const WorldTile>>& tiles);
};

/// Static worlds of all maps, opened on first use from {folder}/{mapId:03}.anw. Thread-safe.
class WorldCache
{
public:
    WorldCache(std::filesystem::path folder, size_t maxCachedTilesPerMap);

    /// The map's world, nullptr if it has no .anw file.
    StaticWorld* Get(int mapId);

private:
    std::filesystem::path Folder;
    size_t MaxTiles;
    std::mutex Mutex;
    std::unordered_map<int, std::unique_ptr<StaticWorld>> Worlds; // nullptr = no file
};
} // namespace World
