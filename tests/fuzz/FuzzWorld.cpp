// Fuzz the static world: .anw tile parsing, the bounding volume hierarchy built on it and ray/vertical queries.

#include <cstdint>
#include <vector>

#include "World/StaticWorld.hpp"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    auto tile = AnpWorld::Deserialize(data, size);

    if (!tile)
    {
        return 0;
    }

    const World::WorldTile world(std::move(*tile));
    const auto& t = world.Data();

    if (t.verts.empty())
    {
        return 0;
    }

    // Rays between vertices and through the tile's middle, vertical queries over the vertices.
    float mid[3]{};

    for (size_t i = 0; i < t.verts.size(); ++i)
    {
        mid[i % 3] += t.verts[i] / static_cast<float>(t.verts.size() / 3);
    }

    std::vector<std::pair<float, uint32_t>> hits;

    for (size_t v = 0; v + 3 <= t.verts.size() && v < 3 * 64; v += 3)
    {
        const float* p = &t.verts[v];
        const float delta[3]{mid[0] - p[0] + 0.37f, mid[1] - p[1] + 0.11f, mid[2] - p[2] - 0.23f};
        float tMax = 1.0f;
        uint32_t triangle = 0;
        world.Raycast(p, delta, tMax, static_cast<uint32_t>(v) & 0x1F, triangle);
        world.VerticalHits(p[0] + 0.01f, p[2] - 0.01f, p[1] + 10.0f, p[1] - 10.0f, 0, hits);
        (void)world.ChunkArea(p[2], p[0]);
    }

    return 0;
}
