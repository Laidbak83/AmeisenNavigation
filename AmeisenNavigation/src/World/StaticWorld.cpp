#include "StaticWorld.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "../../../AmeisenNavigation.Pack/src/AnpFormat.hpp"

namespace World {
namespace {
constexpr uint32_t LEAF_SIZE = 4;
constexpr float ADT = AnpWorld::ADT_SIZE;
constexpr float CHUNK = ADT / 16.0f;

bool IsLiquidArea(uint8_t area) noexcept
{
    const auto neutral = GetNeutralArea(area);
    return neutral == LIQUID_WATER || neutral == LIQUID_OCEAN || neutral == LIQUID_LAVA || neutral == LIQUID_SLIME;
}

/// Slab test of a segment against a box, true if it enters before tMax.
bool SegmentHitsBox(const float* origin, const float* inverse, const float* bmin, const float* bmax,
                    float tMax) noexcept
{
    float t0 = 0.0f;
    float t1 = tMax;

    for (int a = 0; a < 3; ++a)
    {
        if (std::isinf(inverse[a]))
        {
            if (origin[a] < bmin[a] || origin[a] > bmax[a])
            {
                return false;
            }

            continue;
        }

        float near = (bmin[a] - origin[a]) * inverse[a];
        float far = (bmax[a] - origin[a]) * inverse[a];

        if (near > far)
        {
            std::swap(near, far);
        }

        t0 = std::max(t0, near);
        t1 = std::min(t1, far);

        if (t0 > t1)
        {
            return false;
        }
    }

    return true;
}

/// Möller-Trumbore, double sided. t along origin + t * delta.
bool SegmentHitsTriangle(const float* origin, const float* delta, const float* v0, const float* v1, const float* v2,
                         float& t) noexcept
{
    const float e1[3]{v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]};
    const float e2[3]{v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]};
    const float p[3]{delta[1] * e2[2] - delta[2] * e2[1], delta[2] * e2[0] - delta[0] * e2[2],
                     delta[0] * e2[1] - delta[1] * e2[0]};
    const float det = e1[0] * p[0] + e1[1] * p[1] + e1[2] * p[2];

    if (std::fabs(det) < 1e-12f)
    {
        return false;
    }

    const float inv = 1.0f / det;
    const float s[3]{origin[0] - v0[0], origin[1] - v0[1], origin[2] - v0[2]};
    const float u = (s[0] * p[0] + s[1] * p[1] + s[2] * p[2]) * inv;

    if (u < -1e-6f || u > 1.0f + 1e-6f)
    {
        return false;
    }

    const float q[3]{s[1] * e1[2] - s[2] * e1[1], s[2] * e1[0] - s[0] * e1[2], s[0] * e1[1] - s[1] * e1[0]};
    const float v = (delta[0] * q[0] + delta[1] * q[1] + delta[2] * q[2]) * inv;

    if (v < -1e-6f || u + v > 1.0f + 1e-6f)
    {
        return false;
    }

    t = (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]) * inv;
    return true;
}
} // namespace

uint32_t IgnoreBit(uint32_t source, uint8_t area) noexcept
{
    if (IsLiquidArea(area))
    {
        return IGNORE_LIQUIDS;
    }

    switch (AnpWorld::KindOf(source))
    {
        case AnpWorld::SourceKind::TERRAIN:
            return IGNORE_TERRAIN;
        case AnpWorld::SourceKind::WMO:
            return IGNORE_WMOS;
        case AnpWorld::SourceKind::DOODAD:
            return IGNORE_DOODADS;
        case AnpWorld::SourceKind::LIQUID:
            return IGNORE_LIQUIDS;
        case AnpWorld::SourceKind::GAME_OBJECT:
            return IGNORE_GAME_OBJECTS;
        default:
            return 0;
    }
}

// ── WorldTile ──

WorldTile::WorldTile(AnpWorld::Tile&& tile) : Tile(std::move(tile)) { Build(); }

size_t WorldTile::MemoryBytes() const noexcept
{
    return Tile.verts.size() * 4 + Tile.indices.size() * 4 + Tile.areas.size() + Tile.sources.size() * 4 +
           Nodes.size() * sizeof(Node) + Order.size() * 4;
}

void WorldTile::Build()
{
    const size_t count = Tile.TriCount();
    Order.resize(count);
    std::iota(Order.begin(), Order.end(), 0u);

    if (count == 0)
    {
        return;
    }

    // Triangle bounds and centroids.
    std::vector<float> boxes(count * 6);
    std::vector<float> centers(count * 3);

    for (size_t t = 0; t < count; ++t)
    {
        const float* v[3]{&Tile.verts[Tile.indices[t * 3] * 3ull], &Tile.verts[Tile.indices[t * 3 + 1] * 3ull],
                          &Tile.verts[Tile.indices[t * 3 + 2] * 3ull]};

        for (int a = 0; a < 3; ++a)
        {
            boxes[t * 6 + a] = std::min({v[0][a], v[1][a], v[2][a]});
            boxes[t * 6 + 3 + a] = std::max({v[0][a], v[1][a], v[2][a]});
            centers[t * 3 + a] = (v[0][a] + v[1][a] + v[2][a]) / 3.0f;
        }
    }

    Nodes.reserve(count / LEAF_SIZE * 2 + 1);
    BuildNode(boxes, centers, 0, static_cast<uint32_t>(count));
}

uint32_t WorldTile::BuildNode(const std::vector<float>& boxes, const std::vector<float>& centers, uint32_t start,
                              uint32_t count)
{
    const auto index = static_cast<uint32_t>(Nodes.size());
    Nodes.push_back({});
    Node node{};
    float cmin[3]{INFINITY, INFINITY, INFINITY};
    float cmax[3]{-INFINITY, -INFINITY, -INFINITY};

    for (int a = 0; a < 3; ++a)
    {
        node.bmin[a] = INFINITY;
        node.bmax[a] = -INFINITY;
    }

    for (uint32_t i = start; i < start + count; ++i)
    {
        const uint32_t t = Order[i];

        for (int a = 0; a < 3; ++a)
        {
            node.bmin[a] = std::min(node.bmin[a], boxes[t * 6ull + a]);
            node.bmax[a] = std::max(node.bmax[a], boxes[t * 6ull + 3 + a]);
            cmin[a] = std::min(cmin[a], centers[t * 3ull + a]);
            cmax[a] = std::max(cmax[a], centers[t * 3ull + a]);
        }
    }

    const float extent[3]{cmax[0] - cmin[0], cmax[1] - cmin[1], cmax[2] - cmin[2]};
    const int axis = extent[0] >= extent[1] && extent[0] >= extent[2] ? 0 : extent[1] >= extent[2] ? 1 : 2;

    if (count <= LEAF_SIZE || extent[axis] <= 0.0f)
    {
        node.start = start;
        node.count = count;
        Nodes[index] = node;
        return index;
    }

    // Median split on the longest axis of the centroids: left child right after this node, then the right one.
    const uint32_t half = count / 2;
    const auto first = Order.begin() + start;
    std::nth_element(first, first + half, first + count, [&](uint32_t l, uint32_t r) {
        return centers[l * 3ull + static_cast<size_t>(axis)] < centers[r * 3ull + static_cast<size_t>(axis)];
    });

    BuildNode(boxes, centers, start, half);
    node.start = BuildNode(boxes, centers, start + half, count - half);
    node.count = 0;
    Nodes[index] = node;
    return index;
}

bool WorldTile::Raycast(const float* origin, const float* delta, float& tMax, uint32_t ignore, uint32_t& triangle) const
{
    if (Nodes.empty())
    {
        return false;
    }

    const float inverse[3]{1.0f / delta[0], 1.0f / delta[1], 1.0f / delta[2]};
    bool hit = false;
    uint32_t stack[64];
    int top = 0;
    stack[top++] = 0;

    while (top > 0)
    {
        const Node& node = Nodes[stack[--top]];

        if (!SegmentHitsBox(origin, inverse, node.bmin, node.bmax, tMax))
        {
            continue;
        }

        if (node.count == 0)
        {
            const auto index = static_cast<uint32_t>(&node - Nodes.data());

            if (top + 2 > 64)
            {
                continue; // can't happen with a balanced tree of < 2^32 triangles
            }

            stack[top++] = node.start;
            stack[top++] = index + 1;
            continue;
        }

        for (uint32_t i = node.start; i < node.start + node.count; ++i)
        {
            const uint32_t t = Order[i];

            if (Ignored(t, ignore))
            {
                continue;
            }

            const float* v0 = &Tile.verts[Tile.indices[t * 3] * 3ull];
            const float* v1 = &Tile.verts[Tile.indices[t * 3 + 1] * 3ull];
            const float* v2 = &Tile.verts[Tile.indices[t * 3 + 2] * 3ull];
            float th = 0.0f;

            if (SegmentHitsTriangle(origin, delta, v0, v1, v2, th) && th >= 0.0f && th <= tMax)
            {
                tMax = th;
                triangle = t;
                hit = true;
            }
        }
    }

    return hit;
}

void WorldTile::VerticalHits(float rdX, float rdZ, float rdYTop, float rdYBottom, uint32_t ignore,
                             std::vector<std::pair<float, uint32_t>>& hits) const
{
    if (Nodes.empty() || rdYTop <= rdYBottom)
    {
        return;
    }

    const float origin[3]{rdX, rdYTop, rdZ};
    const float delta[3]{0.0f, rdYBottom - rdYTop, 0.0f};
    const float inverse[3]{INFINITY, 1.0f / delta[1], INFINITY};
    uint32_t stack[64];
    int top = 0;
    stack[top++] = 0;

    while (top > 0)
    {
        const Node& node = Nodes[stack[--top]];

        if (!SegmentHitsBox(origin, inverse, node.bmin, node.bmax, 1.0f))
        {
            continue;
        }

        if (node.count == 0)
        {
            if (top + 2 <= 64)
            {
                stack[top++] = node.start;
                stack[top++] = static_cast<uint32_t>(&node - Nodes.data()) + 1;
            }

            continue;
        }

        for (uint32_t i = node.start; i < node.start + node.count; ++i)
        {
            const uint32_t t = Order[i];
            float th = 0.0f;

            if (!Ignored(t, ignore) &&
                SegmentHitsTriangle(origin, delta, &Tile.verts[Tile.indices[t * 3] * 3ull],
                                    &Tile.verts[Tile.indices[t * 3 + 1] * 3ull],
                                    &Tile.verts[Tile.indices[t * 3 + 2] * 3ull], th) &&
                th >= 0.0f && th <= 1.0f)
            {
                hits.emplace_back(rdYTop + delta[1] * th, t);
            }
        }
    }
}

Vector3 WorldTile::Normal(uint32_t triangle) const noexcept
{
    const float* v0 = &Tile.verts[Tile.indices[triangle * 3] * 3ull];
    const float* v1 = &Tile.verts[Tile.indices[triangle * 3 + 1] * 3ull];
    const float* v2 = &Tile.verts[Tile.indices[triangle * 3 + 2] * 3ull];
    const Vector3 e1{v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]};
    const Vector3 e2{v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]};
    Vector3 n{e1.y * e2.z - e1.z * e2.y, e1.z * e2.x - e1.x * e2.z, e1.x * e2.y - e1.y * e2.x};
    const float length = n.Length();
    return length > 0.0f ? n * (1.0f / length) : Vector3{};
}

uint32_t WorldTile::ChunkArea(float wowX, float wowY) const noexcept
{
    const int row = std::clamp(static_cast<int>(((32 - Tile.adtY) * ADT - wowX) / CHUNK), 0, 15);
    const int column = std::clamp(static_cast<int>(((32 - Tile.adtX) * ADT - wowY) / CHUNK), 0, 15);
    return Tile.chunkAreas[static_cast<size_t>(row * 16 + column)];
}

// ── StaticWorld ──

StaticWorld::StaticWorld(const std::filesystem::path& file, size_t maxCachedTiles)
    : Reader(std::make_unique<AnpWorld::WorldReader>(file)), MaxTiles(std::max<size_t>(1, maxCachedTiles))
{
}

std::shared_ptr<const WorldTile> StaticWorld::GetTile(int adtX, int adtY)
{
    if (!IsValid() || adtX < 0 || adtX > 63 || adtY < 0 || adtY > 63)
    {
        return nullptr;
    }

    const int key = adtX * 64 + adtY;
    const std::lock_guard lock(Mutex);

    if (const auto it = Index.find(key); it != Index.end())
    {
        Lru.splice(Lru.begin(), Lru, it->second);
        return it->second->second;
    }

    if (Missing.contains(key))
    {
        return nullptr;
    }

    auto data = Reader->ReadTile(adtX, adtY);

    if (!data)
    {
        Missing[key] = true;
        return nullptr;
    }

    auto tile = std::make_shared<const WorldTile>(std::move(*data));
    Lru.emplace_front(key, tile);
    Index[key] = Lru.begin();

    while (Lru.size() > MaxTiles)
    {
        Index.erase(Lru.back().first);
        Lru.pop_back();
    }

    return tile;
}

size_t StaticWorld::CachedTiles() const
{
    const std::lock_guard lock(Mutex);
    return Lru.size();
}

bool StaticWorld::TilesAlong(const Vector3& a, const Vector3& b, float margin,
                             std::vector<std::shared_ptr<const WorldTile>>& tiles)
{
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;

    if (!AnpWorld::AdtOf(std::max(a.x, b.x) + margin, std::max(a.y, b.y) + margin, x0, y0) ||
        !AnpWorld::AdtOf(std::min(a.x, b.x) - margin, std::min(a.y, b.y) - margin, x1, y1))
    {
        return false;
    }

    for (int x = x0; x <= x1; ++x)
    {
        for (int y = y0; y <= y1; ++y)
        {
            if (auto tile = GetTile(x, y))
            {
                tiles.push_back(std::move(tile));
            }
        }
    }

    return !tiles.empty();
}

bool StaticWorld::Raycast(const Vector3& start, const Vector3& end, uint32_t ignore, Hit& hit)
{
    hit = {};
    hit.position = end;

    if (!start.IsFinite() || !end.IsFinite() || start.DistanceTo(end) > MAX_SEGMENT_LENGTH)
    {
        return false;
    }

    std::vector<std::shared_ptr<const WorldTile>> tiles;

    if (!TilesAlong(start, end, 0.0f, tiles))
    {
        return false;
    }

    float origin[3];
    float target[3];
    start.CopyToRDCoords(origin);
    end.CopyToRDCoords(target);
    const float delta[3]{target[0] - origin[0], target[1] - origin[1], target[2] - origin[2]};
    float tMax = 1.0f;
    const WorldTile* hitTile = nullptr;
    uint32_t triangle = 0;

    for (const auto& tile : tiles)
    {
        if (tile->Raycast(origin, delta, tMax, ignore, triangle))
        {
            hitTile = tile.get();
        }
    }

    if (!hitTile)
    {
        return true;
    }

    hit.hit = true;
    hit.t = tMax;
    hit.position = start + (end - start) * tMax;
    Vector3 normal = hitTile->Normal(triangle);

    if (normal.x * delta[0] + normal.y * delta[1] + normal.z * delta[2] > 0.0f)
    {
        normal = normal * -1.0f;
    }

    hit.normal = normal.ToWowCoords();
    hit.source = hitTile->Data().sources[triangle];
    hit.area = hitTile->Data().areas[triangle];
    return true;
}

bool StaticWorld::Surfaces(float x, float y, float zTop, float zBottom, uint32_t ignore, std::vector<Surface>& surfaces,
                           uint32_t& areaId)
{
    surfaces.clear();
    areaId = 0;
    int adtX = 0;
    int adtY = 0;

    if (!std::isfinite(zTop) || !std::isfinite(zBottom) || zTop <= zBottom || zTop - zBottom > 20000.0f ||
        !AnpWorld::AdtOf(x, y, adtX, adtY))
    {
        return false;
    }

    const auto tile = GetTile(adtX, adtY);

    if (!tile)
    {
        return false;
    }

    areaId = tile->ChunkArea(x, y);
    std::vector<std::pair<float, uint32_t>> hits;
    // RD: x = WoW y, z = WoW x, y = up.
    tile->VerticalHits(y, x, zTop, zBottom, ignore, hits);
    std::sort(hits.begin(), hits.end(), [](const auto& l, const auto& r) { return l.first > r.first; });

    for (const auto& [z, triangle] : hits)
    {
        const uint32_t source = tile->Data().sources[triangle];
        const uint8_t area = tile->Data().areas[triangle];

        // A point on a shared edge or vertex hits every triangle around it: once per surface and source.
        if (!surfaces.empty() && surfaces.back().z - z < 0.01f && surfaces.back().source == source)
        {
            continue;
        }

        Surface surface;
        surface.z = z;
        Vector3 normal = tile->Normal(triangle);

        if (normal.y < 0.0f)
        {
            normal = normal * -1.0f;
        }

        surface.normal = normal.ToWowCoords();
        surface.source = source;
        surface.area = area;
        surface.liquid = IgnoreBit(source, area) == IGNORE_LIQUIDS;
        surface.standable = area != 0 && !surface.liquid;
        surfaces.push_back(surface);
    }

    return true;
}

bool StaticWorld::Sweep(const Vector3& start, const Vector3& end, float radius, float height, float stepHeight,
                        uint32_t ignore, Hit& hit)
{
    hit = {};
    hit.position = end;
    const Vector3 delta = end - start;
    const float length = std::sqrt(delta.x * delta.x + delta.y * delta.y);

    if (!start.IsFinite() || !end.IsFinite() || !(radius >= 0.0f) || radius > 10.0f || !(height > stepHeight) ||
        height > 20.0f || !(stepHeight >= 0.0f) || start.DistanceTo(end) > MAX_SEGMENT_LENGTH)
    {
        return false;
    }

    if (length < 1e-4f)
    {
        return true;
    }

    // Forward and sideways in the horizontal plane; the rays follow the slope between start and end.
    const Vector3 forward{delta.x / length, delta.y / length, 0.0f};
    const Vector3 side{-forward.y, forward.x, 0.0f};
    const float heights[3]{stepHeight + 0.05f, (stepHeight + height) * 0.5f, height - 0.05f};
    const float offsets[3]{0.0f, -radius, radius};
    const Vector3 front = forward * radius;

    // Every ray goes from the body's start to its front at the end, t is converted to the center's travel.
    const float travel = length + radius;
    float best = 1.0f;

    for (const float h : heights)
    {
        for (const float o : offsets)
        {
            const Vector3 lift{0.0f, 0.0f, h};
            const Vector3 a = start + side * o + lift;
            const Vector3 b = end + side * o + front + lift;
            Hit rayHit;

            if (!Raycast(a, b, ignore | IGNORE_LIQUIDS, rayHit))
            {
                return false;
            }

            if (rayHit.hit)
            {
                const float t = std::clamp((rayHit.t * travel - radius) / length, 0.0f, 1.0f);

                if (!hit.hit || t < best)
                {
                    best = t;
                    hit = rayHit;
                    hit.t = t;
                }
            }
        }
    }

    hit.position = start + delta * hit.t;
    return true;
}

// ── WorldCache ──

WorldCache::WorldCache(std::filesystem::path folder, size_t maxCachedTilesPerMap)
    : Folder(std::move(folder)), MaxTiles(maxCachedTilesPerMap)
{
}

StaticWorld* WorldCache::Get(int mapId)
{
    if (mapId < 0 || mapId > 65535)
    {
        return nullptr;
    }

    const std::lock_guard lock(Mutex);

    if (const auto it = Worlds.find(mapId); it != Worlds.end())
    {
        return it->second.get();
    }

    std::unique_ptr<StaticWorld> world;
    const auto path = Folder / AnpWorld::FileName(mapId);
    std::error_code ec;

    if (std::filesystem::is_regular_file(path, ec))
    {
        world = std::make_unique<StaticWorld>(path, MaxTiles);

        if (!world->IsValid())
        {
            LogW("Invalid static world file: ", path.string());
            world.reset();
        }
    }

    return Worlds.emplace(mapId, std::move(world)).first->second.get();
}
} // namespace World
