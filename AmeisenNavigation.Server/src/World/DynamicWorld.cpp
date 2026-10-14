#include "DynamicWorld.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "Processors/AdtTileProcessor.hpp"
#include "Processors/WorldBuilder.hpp"
#include "Wow/GameObjectSpawn.hpp"

namespace {
/// Tiles reach this far beyond their ADT (Recast border), objects closer than that to a neighbour change it too.
constexpr float TILE_MARGIN = 8.0f;
constexpr uint64_t MAX_ENTRY_BYTES = 256ull << 20;
} // namespace

DynamicWorld::DynamicWorld(AmeisenNavigation& navigation, std::filesystem::path folder)
    : Navigation(navigation), Folder(std::move(folder))
{
    Worker = std::thread([this] { Run(); });
}

DynamicWorld::~DynamicWorld()
{
    {
        const std::lock_guard lock(Mutex);
        Stop = true;
    }

    Cv.notify_all();

    if (Worker.joinable())
    {
        Worker.join();
    }
}

std::shared_ptr<const AnpWorld::Tile> DynamicWorld::Model(uint32_t displayId)
{
    const std::lock_guard lock(ModelsMutex);

    if (!ModelsLoaded)
    {
        ModelsLoaded = true;
        const auto path = Folder / ObjectModels::FILE_NAME;
        std::error_code ec;

        if (std::filesystem::is_regular_file(path, ec))
        {
            Models = std::make_unique<AnpWorld::WorldReader>(path);

            if (const auto table = Models->ReadEntry(ObjectModels::DISPLAYS_ENTRY, 64ull << 20))
            {
                for (size_t i = 0; i + 8 <= table->size(); i += 8)
                {
                    uint32_t entry[2];
                    std::memcpy(entry, table->data() + i, 8);
                    Displays[entry[0]] = entry[1];
                }
            }

            LogI("Game object models: ", Displays.size(), " displays from ", path.string());
        }
    }

    if (const auto it = ModelCache.find(displayId); it != ModelCache.end())
    {
        return it->second;
    }

    std::shared_ptr<const AnpWorld::Tile> model;
    const auto display = Displays.find(displayId);

    if (Models && display != Displays.end())
    {
        if (const auto bytes = Models->ReadEntry(ObjectModels::ModelEntryName(display->second), MAX_ENTRY_BYTES))
        {
            if (auto tile = AnpWorld::Deserialize(bytes->data(), bytes->size()))
            {
                model = std::make_shared<const AnpWorld::Tile>(std::move(*tile));
            }
        }
    }

    // Unknown displays are cached too (clients send the same objects again and again).
    ModelCache[displayId] = model;
    return model;
}

std::shared_ptr<const World::DynamicMesh> DynamicWorld::MakeMesh(const WorldObject& object)
{
    const auto model = Model(object.displayId);

    if (!model || model->indices.empty())
    {
        return nullptr;
    }

    // The same transform as game objects added at export (model space -> WoW).
    GameObjectSpawn spawn;
    spawn.position = object.position;
    std::copy(std::begin(object.rotation), std::end(object.rotation), spawn.rotation);
    spawn.scale = object.scale;
    const Matrix4x4 transform = GameObjects::SpawnTransform(spawn);

    auto mesh = std::make_shared<World::DynamicMesh>();
    mesh->guid = object.guid;
    mesh->source = AnpWorld::MakeSource(AnpWorld::SourceKind::GAME_OBJECT, object.displayId);
    mesh->indices = model->indices;
    mesh->areas = model->areas;
    mesh->verts.resize(model->verts.size());

    for (int a = 0; a < 3; ++a)
    {
        mesh->bmin[a] = INFINITY;
        mesh->bmax[a] = -INFINITY;
    }

    for (size_t i = 0; i + 2 < model->verts.size(); i += 3)
    {
        // Stored in RD order (the exporter converted model space like world space).
        Vector3 local{model->verts[i], model->verts[i + 1], model->verts[i + 2]};
        Vector3 world = transform.Transform(local.ToWowCoords()).ToRDCoords();

        for (int a = 0; a < 3; ++a)
        {
            mesh->verts[i + a] = world.pos[a];
            mesh->bmin[a] = std::min(mesh->bmin[a], world.pos[a]);
            mesh->bmax[a] = std::max(mesh->bmax[a], world.pos[a]);
        }
    }

    return mesh;
}

bool DynamicWorld::Rebuildable(int mapId, MapState& state)
{
    if (!state.checked)
    {
        state.checked = true;
        World::StaticWorld* world = Navigation.GetWorld(mapId);
        const auto build = world ? world->ReadEntry(WorldExtras::BUILD_ENTRY, 4096) : std::nullopt;
        state.rebuildable = build && WorldExtras::DeserializeBuild(*build).has_value();

        if (!state.rebuildable)
        {
            LogW("Map ", mapId, ": no static world with build settings, game objects can't change its navmesh");
        }
    }

    return state.rebuildable;
}

void DynamicWorld::MarkDirty(int mapId, const World::DynamicMesh& mesh)
{
    // RD x = WoW y, RD z = WoW x.
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;

    if (AnpWorld::AdtOf(mesh.bmax[2] + TILE_MARGIN, mesh.bmax[0] + TILE_MARGIN, x0, y0) &&
        AnpWorld::AdtOf(mesh.bmin[2] - TILE_MARGIN, mesh.bmin[0] - TILE_MARGIN, x1, y1))
    {
        for (int x = x0; x <= x1; ++x)
        {
            for (int y = y0; y <= y1; ++y)
            {
                Dirty.emplace(mapId, x, y);
            }
        }
    }
}

void DynamicWorld::PublishMeshes(int mapId, const MapState& state)
{
    if (World::StaticWorld* world = Navigation.GetWorld(mapId))
    {
        auto meshes = std::make_shared<World::DynamicMeshes>();

        for (const auto& [guid, object] : state.objects)
        {
            if (object.spawn.blocking && object.mesh)
            {
                meshes->push_back(object.mesh);
            }
        }

        world->SetDynamicMeshes(std::move(meshes));
    }
}

int DynamicWorld::SetObjects(int mapId, std::span<const WorldObject> objects, std::span<const uint64_t> removed)
{
    // Meshes are made outside the lock (model reads).
    std::vector<std::pair<WorldObject, std::shared_ptr<const World::DynamicMesh>>> made;

    for (const WorldObject& object : objects)
    {
        if (!object.position.IsFinite() || !std::isfinite(object.scale) || object.scale <= 0.0f ||
            object.scale > 100.0f || !std::all_of(std::begin(object.rotation), std::end(object.rotation), [](float v) {
                return std::isfinite(v);
            }))
        {
            continue;
        }

        if (auto mesh = MakeMesh(object))
        {
            made.emplace_back(object, std::move(mesh));
        }
    }

    const std::lock_guard lock(Mutex);
    MapState& state = Maps[mapId];

    if (!Rebuildable(mapId, state))
    {
        return -1;
    }

    const auto dirtyBefore = Dirty.size();
    const auto changed = [&](const Object& object) {
        if (object.spawn.blocking && object.mesh)
        {
            MarkDirty(mapId, *object.mesh);
        }
    };

    for (const uint64_t guid : removed)
    {
        if (const auto it = state.objects.find(guid); it != state.objects.end())
        {
            changed(it->second);
            state.objects.erase(it);
        }
    }

    int accepted = 0;

    for (auto& [object, mesh] : made)
    {
        const auto it = state.objects.find(object.guid);

        if (it != state.objects.end())
        {
            // Same place and state: nothing to rebuild.
            const WorldObject& old = it->second.spawn;

            if (old.displayId == object.displayId && old.blocking == object.blocking &&
                old.position.DistanceTo(object.position) < 0.01f && old.scale == object.scale &&
                std::equal(std::begin(old.rotation), std::end(old.rotation), std::begin(object.rotation)))
            {
                accepted++;
                continue;
            }

            changed(it->second);
        }
        else if (state.objects.size() >= MAX_OBJECTS_PER_MAP)
        {
            continue;
        }

        Object& entry = state.objects[object.guid];
        entry.spawn = object;
        entry.mesh = std::move(mesh);
        changed(entry);
        accepted++;
    }

    PublishMeshes(mapId, state);

    if (Dirty.size() != dirtyBefore)
    {
        Cv.notify_all();
    }

    return accepted;
}

size_t DynamicWorld::PendingTiles() const
{
    const std::lock_guard lock(Mutex);
    return Dirty.size() + Running;
}

size_t DynamicWorld::ObjectCount(int mapId) const
{
    const std::lock_guard lock(Mutex);
    const auto it = Maps.find(mapId);
    return it != Maps.end() ? it->second.objects.size() : 0;
}

bool DynamicWorld::WaitIdle(std::chrono::milliseconds timeout)
{
    std::unique_lock lock(Mutex);
    return IdleCv.wait_for(lock, timeout, [this] { return Dirty.empty() && Running == 0; });
}

void DynamicWorld::Run()
{
    std::unique_lock lock(Mutex);

    while (true)
    {
        Cv.wait(lock, [this] { return Stop || !Dirty.empty(); });

        if (Stop)
        {
            return;
        }

        const auto [mapId, adtX, adtY] = *Dirty.begin();
        Dirty.erase(Dirty.begin());
        Running++;
        lock.unlock();

        try
        {
            Rebuild(mapId, adtX, adtY);
        }
        catch (const std::exception& e)
        {
            LogE("Map ", mapId, ": rebuilding ADT ", adtX, ",", adtY, " failed: ", e.what());
        }

        lock.lock();
        Running--;

        if (Dirty.empty() && Running == 0)
        {
            IdleCv.notify_all();
        }
    }
}

void DynamicWorld::Rebuild(int mapId, int adtX, int adtY)
{
    World::StaticWorld* world = Navigation.GetWorld(mapId);
    const auto buildEntry = world ? world->ReadEntry(WorldExtras::BUILD_ENTRY, 4096) : std::nullopt;
    const auto build = buildEntry ? WorldExtras::DeserializeBuild(*buildEntry) : std::nullopt;

    if (!build || !world->ReadRawTile(adtX, adtY))
    {
        return; // no build settings, or nothing exported at this ADT (an object on the map's edge)
    }

    const auto start = std::chrono::steady_clock::now();

    // The ADT and its neighbours (the tile border reaches into them).
    Structure geometry;

    for (int x = adtX - 1; x <= adtX + 1; ++x)
    {
        for (int y = adtY - 1; y <= adtY + 1; ++y)
        {
            const auto tile = x >= 0 && x < 64 && y >= 0 && y < 64 ? world->ReadRawTile(x, y) : std::nullopt;

            if (!tile)
            {
                continue;
            }

            const int base = static_cast<int>(geometry.verts.size());

            for (size_t i = 0; i + 2 < tile->verts.size(); i += 3)
            {
                geometry.verts.emplace_back(tile->verts[i], tile->verts[i + 1], tile->verts[i + 2]);
            }

            for (size_t t = 0; t < tile->TriCount(); ++t)
            {
                geometry.AddTri(Tri{base + static_cast<int>(tile->indices[t * 3]),
                                    base + static_cast<int>(tile->indices[t * 3 + 1]),
                                    base + static_cast<int>(tile->indices[t * 3 + 2])},
                                static_cast<TriAreaId>(tile->areas[t]));
            }
        }
    }

    // Blocking objects of the map near the ADT.
    {
        const std::lock_guard lock(Mutex);
        const float minX = (31 - adtX) * TILESIZE - TILE_MARGIN, maxX = (32 - adtX) * TILESIZE + TILE_MARGIN;
        const float minZ = (31 - adtY) * TILESIZE - TILE_MARGIN, maxZ = (32 - adtY) * TILESIZE + TILE_MARGIN;

        for (const auto& [guid, object] : Maps[mapId].objects)
        {
            const World::DynamicMesh* mesh = object.mesh.get();

            if (!object.spawn.blocking || !mesh || mesh->bmax[0] < minX || mesh->bmin[0] > maxX ||
                mesh->bmax[2] < minZ || mesh->bmin[2] > maxZ)
            {
                continue;
            }

            const int base = static_cast<int>(geometry.verts.size());

            for (size_t i = 0; i + 2 < mesh->verts.size(); i += 3)
            {
                geometry.verts.emplace_back(mesh->verts[i], mesh->verts[i + 1], mesh->verts[i + 2]);
            }

            for (size_t t = 0; t + 2 < mesh->indices.size(); t += 3)
            {
                const uint8_t stored = t / 3 < mesh->areas.size() ? mesh->areas[t / 3] : uint8_t{0};
                const auto area = static_cast<TriAreaId>(stored ? stored : static_cast<uint8_t>(DOODAD));
                geometry.AddTri(Tri{base + static_cast<int>(mesh->indices[t]),
                                    base + static_cast<int>(mesh->indices[t + 1]),
                                    base + static_cast<int>(mesh->indices[t + 2])},
                                area);
            }
        }
    }

    WaterMap water;
    RoadMap roads;
    CityMap cities;
    FactionMap factions;

    if (const auto rects = world->ReadEntry(WorldExtras::RectEntryName(adtX, adtY), MAX_ENTRY_BYTES))
    {
        WorldExtras::DeserializeRects(*rects, water, roads, cities, factions);
    }

    water.BuildSpatialIndex();
    roads.BuildSpatialIndex();
    cities.BuildSpatialIndex();
    factions.BuildSpatialIndex();

    // Detour tile of the ADT (the export aligned the grid: tile x = maxAdtX - adtX).
    const dtNavMeshParams& params = build->params;
    const TileCoord coord{static_cast<int>(std::lround(((31 - adtX) * TILESIZE - params.orig[0]) / params.tileWidth)),
                          static_cast<int>(std::lround(((31 - adtY) * TILESIZE - params.orig[2]) / params.tileHeight)),
                          adtX, adtY};

    Anp::MemoryTileSink sink(mapId, params);
    AdtTileProcessor processor(&sink, {}, std::format("Map{}", mapId), build->config);
    processor.Process(&geometry, {coord}, &water, &roads, &factions, &cities);
    const int replaced = Navigation.ReplaceTiles(mapId, sink.TakeTiles());

    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
    LogI("Map ", mapId, ": ADT ", adtX, ",", adtY, " rebuilt around game objects in ", ms, "ms",
         replaced ? "" : " (no tile)");
}
