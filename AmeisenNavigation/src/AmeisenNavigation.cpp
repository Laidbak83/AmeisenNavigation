#include "AmeisenNavigation.hpp"

#include <cfloat>
#include <chrono>
#include <cmath>

#include <DetourNode.h>

#ifdef _WIN32
#include <excpt.h>
#include <windows.h>

/// Isolate findStraightPath in its own function so Windows SEH (__try/__except) can catch access violations
/// caused by corrupt navmesh tile data. SEH and C++ destructors cannot coexist in the same function under /EHsc,
/// this function only has POD locals.
static dtStatus SafeFindStraightPath(const dtNavMeshQuery* query, const float* startPos, const float* endPos,
                                     const dtPolyRef* polyPath, int polyPathCount, float* straightPath,
                                     unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
                                     int* straightPathCount, int maxStraightPath) noexcept
{
    __try
    {
        return query->findStraightPath(startPos, endPos, polyPath, polyPathCount, straightPath, straightPathFlags,
                                       straightPathRefs, straightPathCount, maxStraightPath);
    }
    __except (GetExceptionCode() == EXCEPTION_ACCESS_VIOLATION ? EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH)
    {
        return DT_FAILURE;
    }
}
#else
static dtStatus SafeFindStraightPath(const dtNavMeshQuery* query, const float* startPos, const float* endPos,
                                     const dtPolyRef* polyPath, int polyPathCount, float* straightPath,
                                     unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
                                     int* straightPathCount, int maxStraightPath) noexcept
{
    return query->findStraightPath(startPos, endPos, polyPath, polyPathCount, straightPath, straightPathFlags,
                                   straightPathRefs, straightPathCount, maxStraightPath);
}
#endif

AmeisenNavigation::AmeisenNavigation(const AmeisenNavigationSettings& settings)
    : Settings(settings),
      Worlds(settings.worldFolder.empty() ? settings.meshFolder : settings.worldFolder, settings.worldCacheTiles)
{
    Settings.maxPolyPath = std::max(Settings.maxPolyPath, 1);
    Settings.maxPointPath = std::max(Settings.maxPointPath, 2);
    Settings.maxSearchNodes = std::clamp(Settings.maxSearchNodes, 1, 65535);

    if (Settings.useAnp)
    {
        NavSource = std::make_unique<AnpNavSource>(Settings.meshFolder);
        FilterProvider = std::make_unique<AnpQueryFilterProvider>(Settings.waterCost, Settings.badLiquidCost,
                                                                  Settings.roadCost, Settings.factionDangerCost);
    }
    else
    {
        auto mmapSource =
            std::make_unique<MmapNavSource>(Settings.meshFolder, Settings.mmapFormat, Settings.customMmapPatterns,
                                            Settings.parentMaps);
        Settings.mmapFormat = mmapSource->GetFormat();
        FilterProvider =
            std::make_unique<MmapQueryFilterProvider>(Settings.mmapFormat, Settings.waterCost, Settings.badLiquidCost);
        NavSource = std::move(mmapSource);
    }

    NavSource->SetPostLoad([this](int mapId, dtNavMesh& navMesh) { AddOffMeshLinks(mapId, navMesh); });

    if (!Settings.avoidZonesFile.empty())
    {
        const size_t zones = GlobalZones.Open(Settings.avoidZonesFile);
        LogI("Loaded ", zones, " avoid zones from ", Settings.avoidZonesFile.string());
    }
}

void AmeisenNavigation::AddOffMeshLinks(int mapId, dtNavMesh& navMesh)
{
    if (Settings.offMeshFile.empty() && Settings.autoDropHeight <= 0.0f)
    {
        return;
    }

    // Walkable flags of the navmesh format for links without their own.
    unsigned short groundFlags = NAV_GROUND;

    if (!Settings.useAnp)
    {
        groundFlags = Settings.mmapFormat == MmapFormat::SF548 ? static_cast<unsigned short>(NavArea548::GROUND)
                                                               : static_cast<unsigned short>(NavFlag335a::GROUND);
    }

    const auto start = std::chrono::steady_clock::now();
    std::vector<OffMeshLink> links;
    size_t fromFile = 0;

    if (!Settings.offMeshFile.empty())
    {
        links = OffMeshLinks::ReadFile(Settings.offMeshFile, mapId, groundFlags);
        fromFile = links.size();
    }

    OffMeshLinks::DropSettings drops;
    drops.maxHeight = Settings.autoDropHeight;
    drops.reach = Settings.autoDropReach;
    const auto found = OffMeshLinks::FindDrops(navMesh, drops);
    links.insert(links.end(), found.begin(), found.end());

    if (links.empty())
    {
        return;
    }

    const int rebuilt = TileRebuilder::AddOffMeshLinks(navMesh, links);
    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
    LogI("Map ", mapId, ": ", fromFile, " off-mesh connections from file, ", found.size(), " drop links, ",
         rebuilt < 0 ? std::string("some tiles failed to rebuild") : std::format("{} tiles rebuilt", rebuilt), " in ",
         ms, "ms");

    std::lock_guard lock(OffMeshStatsMutex);
    AddedOffMeshLinks[mapId] = static_cast<int>(links.size());
}

int AmeisenNavigation::GetAddedOffMeshLinks(int mapId) const
{
    std::lock_guard lock(OffMeshStatsMutex);
    const auto it = AddedOffMeshLinks.find(mapId);
    return it != AddedOffMeshLinks.end() ? it->second : 0;
}

MmapFormat AmeisenNavigation::GetMmapFormat() const noexcept
{
    return Settings.useAnp ? MmapFormat::UNKNOWN : Settings.mmapFormat;
}

bool AmeisenNavigation::NewClient(size_t clientId)
{
    auto client = std::make_shared<AmeisenNavClient>(clientId, FilterProvider.get(), Settings.maxPolyPath,
                                                     Settings.maxPointPath);

    std::unique_lock lock(ClientsMutex);
    const auto [it, inserted] = Clients.try_emplace(clientId, std::move(client));
    LogD("New client: ", clientId);
    return inserted;
}

void AmeisenNavigation::FreeClient(size_t clientId)
{
    std::shared_ptr<AmeisenNavClient> client;

    {
        std::unique_lock lock(ClientsMutex);
        const auto it = Clients.find(clientId);

        if (it == Clients.end())
        {
            return;
        }

        // Destroy outside the lock (frees all queries/buffers).
        client = std::move(it->second);
        Clients.erase(it);
    }

    LogD("Freed client: ", clientId);
}

std::shared_ptr<AmeisenNavClient> AmeisenNavigation::GetClient(size_t clientId) const
{
    std::shared_lock lock(ClientsMutex);
    const auto it = Clients.find(clientId);
    return it != Clients.end() ? it->second : nullptr;
}

size_t AmeisenNavigation::GetClientCount() const
{
    std::shared_lock lock(ClientsMutex);
    return Clients.size();
}

bool AmeisenNavigation::PreloadMap(int mapId) noexcept
{
    return NavSource->Get(mapId) != nullptr;
}

bool AmeisenNavigation::GetPath(size_t clientId, int mapId, const Vector3& startPosition, const Vector3& endPosition,
                                Path& path, bool* partial, const PathOptions* options, unsigned char* pointFlags,
                                unsigned char* pointInfo, unsigned char* pointAreas)
{
    path.Clear();
    const auto ctx = GetQueryContext(clientId, mapId);

    if (!ctx)
    {
        return false;
    }

    LogD("[", clientId, "] GetPath (", mapId, ") ", startPosition, " -> ", endPosition);

    dtPolyRef* refs = ctx.client->GetStraightPathRefBuffer();
    unsigned char* flags = ctx.client->GetStraightPathFlagBuffer();
    const bool found =
        options && options->follow
            ? CalculateFollowPath(ctx.query, ctx.filter, *ctx.client, mapId, startPosition, endPosition, path,
                                  partial, refs, flags)
            : CalculateNormalPath(ctx.query, ctx.filter, *ctx.client, startPosition, endPosition, path, refs, partial,
                                  flags);

    if (!found)
    {
        return false;
    }

    ToPointFlags(ctx.query, flags, refs, path.pointCount, flags);
    Vector3 target;
    endPosition.CopyToRDCoords(target);
    RefineAll(ctx.query, ctx.filter, path, flags, target, options, true);

    if (options && options->clearBody)
    {
        ClearBody(ctx.query, ctx.filter, mapId, path, flags, *options);
    }

    if (pointInfo)
    {
        std::fill(pointInfo, pointInfo + path.maxSize, static_cast<unsigned char>(0));
    }

    if (options && options->detail)
    {
        DetailPath(ctx.query, ctx.filter, path, flags, *options, pointInfo);
        GroundPoints(mapId, path);
    }

    if (pointInfo && pointAreas)
    {
        DescribePoints(ctx.query, ctx.filter, path, pointInfo, pointAreas);
    }

    if (pointFlags)
    {
        std::copy(flags, flags + path.pointCount, pointFlags);
    }

    path.ToWowCoords();
    return true;
}

bool AmeisenNavigation::GetRandomPath(size_t clientId, int mapId, const Vector3& startPosition,
                                      const Vector3& endPosition, Path& path, float maxRandomDistance, bool* partial,
                                      const PathOptions* options, unsigned char* pointFlags, unsigned char* pointInfo,
                                      unsigned char* pointAreas)
{
    path.Clear();
    const auto ctx = GetQueryContext(clientId, mapId);

    if (!ctx)
    {
        return false;
    }

    LogD("[", clientId, "] GetRandomPath (", mapId, ") ", startPosition, " -> ", endPosition);

    // The straight path corner refs need their own buffer, the poly path buffer is still being read while
    // findStraightPath writes them.
    dtPolyRef* cornerRefs = ctx.client->GetStraightPathRefBuffer();
    unsigned char* flags = ctx.client->GetStraightPathFlagBuffer();

    if (!CalculateNormalPath(ctx.query, ctx.filter, *ctx.client, startPosition, endPosition, path, cornerRefs,
                             partial, flags))
    {
        return false;
    }

    ToPointFlags(ctx.query, flags, cornerRefs, path.pointCount, flags);
    Vector3 target;
    endPosition.CopyToRDCoords(target);

    // Zone detours, shortcut and stop distance before the jitter, wall margin and arcs after it.
    if (HasZones(ctx.filter) || (options && (options->shortcut || options->tuning.stopDistance > 0.0f)))
    {
        PathOptions first;

        if (options)
        {
            first.shortcut = options->shortcut;
            first.tuning.stopDistance = options->tuning.stopDistance;
        }

        RefineAll(ctx.query, ctx.filter, path, flags, target, &first, true);

        // The corner refs belong to the plain path.
        for (int i = 0; i < path.pointCount; ++i)
        {
            PolyPosition snapped;
            cornerRefs[i] = FindNearestPoly(ctx.query, ctx.filter, path[i], snapped) ? snapped.poly : 0;
        }
    }

    if (maxRandomDistance > 0.0f)
    {
        // Keep start and end, jitter every corner in between on the navmesh. An offset is only kept if the way to it
        // and on to the next corner stays on the navmesh (no shortcut over a ledge or into a wall). Both ends of an
        // off-mesh connection stay where they are.
        PolyPosition previous{cornerRefs[0], path[0]};

        for (int i = 1; i < path.pointCount - 1; ++i)
        {
            const PolyPosition original{cornerRefs[i], path[i]};

            if (!original.poly || !previous.poly || (flags[i] & PATH_POINT_OFFMESH)
                || (flags[i - 1] & PATH_POINT_OFFMESH))
            {
                previous = original;
                continue;
            }

            PolyPosition jittered;
            const dtStatus status =
                ctx.query->findRandomPointAroundCircle(original.poly, original.pos, maxRandomDistance, ctx.filter,
                                                       GetRandomFloat, &jittered.poly, jittered.pos);
            dtRaycastHit toIt{};
            dtRaycastHit fromIt{};

            if (dtStatusSucceed(status) && jittered.poly
                && Raycast(ctx.query, ctx.filter, previous, jittered.pos, toIt) && toIt.t == FLT_MAX
                && Raycast(ctx.query, ctx.filter, jittered, path[i + 1], fromIt) && fromIt.t == FLT_MAX)
            {
                path[i] = jittered.pos;
                previous = jittered;
            }
            else
            {
                previous = original;
            }
        }
    }

    if (options && (options->avoidWalls || options->roundCorners))
    {
        PathOptions second = *options;
        second.shortcut = false;
        second.tuning.stopDistance = 0.0f;
        RefineAll(ctx.query, ctx.filter, path, flags, target, &second, false);
    }

    if (options && options->clearBody)
    {
        ClearBody(ctx.query, ctx.filter, mapId, path, flags, *options);
    }

    if (pointInfo)
    {
        std::fill(pointInfo, pointInfo + path.maxSize, static_cast<unsigned char>(0));
    }

    if (options && options->detail)
    {
        DetailPath(ctx.query, ctx.filter, path, flags, *options, pointInfo);
        GroundPoints(mapId, path);
    }

    if (pointInfo && pointAreas)
    {
        DescribePoints(ctx.query, ctx.filter, path, pointInfo, pointAreas);
    }

    if (pointFlags)
    {
        std::copy(flags, flags + path.pointCount, pointFlags);
    }

    path.ToWowCoords();
    return true;
}

void AmeisenNavigation::ToPointFlags(const dtNavMeshQuery* query, const unsigned char* detourFlags,
                                     const dtPolyRef* refs, int count, unsigned char* out) noexcept
{
    const dtNavMesh* mesh = query->getAttachedNavMesh();

    for (int i = 0; i < count; ++i)
    {
        const unsigned char d = detourFlags[i];
        unsigned char f = 0;
        f |= (d & DT_STRAIGHTPATH_START) ? PATH_POINT_START : 0;
        f |= (d & DT_STRAIGHTPATH_END) ? PATH_POINT_END : 0;

        if ((d & DT_STRAIGHTPATH_OFFMESH_CONNECTION) && i + 1 < count)
        {
            f |= PATH_POINT_OFFMESH;
            const dtOffMeshConnection* con = mesh ? mesh->getOffMeshConnectionByRef(refs[i]) : nullptr;

            switch (con ? static_cast<OffMeshLinkKind>(con->userId) : OffMeshLinkKind::UNKNOWN)
            {
                case OffMeshLinkKind::JUMP: f |= PATH_POINT_JUMP; break;
                case OffMeshLinkKind::DROP: f |= PATH_POINT_DROP; break;
                default: f |= PATH_POINT_LINK; break;
            }
        }

        out[i] = f;
    }
}

void AmeisenNavigation::RefineAll(const dtNavMeshQuery* query, const dtQueryFilter* filter, Path& path,
                                  unsigned char* flags, const Vector3& target, const PathOptions* options,
                                  bool allowShortcut) const noexcept
{
    const bool zones = HasZones(filter);

    if ((!options && !zones) || path.pointCount < 2)
    {
        return;
    }

    try
    {
        // Off-mesh connections can't be walked: the path is refined in pieces between them, their ends stay.
        std::vector<Vector3> points;
        std::vector<unsigned char> pointFlags;
        points.reserve(static_cast<size_t>(path.pointCount));
        pointFlags.reserve(static_cast<size_t>(path.pointCount));
        Path piece(path.maxSize);
        Path scratch(path.maxSize);

        const auto refinePiece = [&](int from, int to, bool last) {
            piece.Clear();

            for (int i = from; i <= to; ++i)
            {
                piece.TryAppend(path[i]);
            }

            if (piece.pointCount >= 2)
            {
                if (zones)
                {
                    LeadAroundZones(query, filter, piece, scratch);
                }

                if (options)
                {
                    PathOptions o = *options;
                    o.tuning.stopDistance = last ? o.tuning.stopDistance : 0.0f;
                    RefinePath(const_cast<dtNavMeshQuery*>(query), filter, piece, target, o, allowShortcut, scratch);
                }
            }

            for (int k = 0; k < piece.pointCount; ++k)
            {
                unsigned char f = 0;

                if (k == 0)
                {
                    f = flags[from];
                }
                else if (k == piece.pointCount - 1 && piece[k] == path[to])
                {
                    f = flags[to];
                }

                if (k == 0 && !points.empty() && points.back() == piece[0])
                {
                    pointFlags.back() |= f;
                    continue;
                }

                points.push_back(piece[k]);
                pointFlags.push_back(f);
            }
        };

        int from = 0;

        for (int i = 0; i + 1 < path.pointCount; ++i)
        {
            if (flags[i] & PATH_POINT_OFFMESH)
            {
                refinePiece(from, i, false);
                from = i + 1;
            }
        }

        refinePiece(from, path.pointCount - 1, true);

        if (static_cast<int>(points.size()) > path.maxSize)
        {
            return; // the unrefined path fits, keep it
        }

        std::copy(points.begin(), points.end(), path.points);
        std::copy(pointFlags.begin(), pointFlags.end(), flags);
        path.pointCount = static_cast<int>(points.size());
    }
    catch (...)
    {
    }
}

bool AmeisenNavigation::MoveAlongSurface(size_t clientId, int mapId, const Vector3& startPosition,
                                         const Vector3& endPosition, Vector3& positionToGoTo)
{
    const auto ctx = GetQueryContext(clientId, mapId);

    if (!ctx)
    {
        return false;
    }

    LogD("[", clientId, "] MoveAlongSurface (", mapId, ") ", startPosition, " -> ", endPosition);

    PolyPosition start;

    if (!FindNearestPolyWow(ctx.query, ctx.filter, startPosition, start))
    {
        return false;
    }

    Vector3 rdEnd;

    if (!ToValidRdCoords(endPosition, rdEnd))
    {
        return false;
    }

    if (const dtStatus status = MoveAlong(ctx.query, ctx.filter, start, rdEnd); dtStatusFailed(status))
    {
        LogD("[", clientId, "] moveAlongSurface failed: 0x", std::format("{:08X}", status));
        return false;
    }

    positionToGoTo = start.pos.ToWowCoords();
    return true;
}

bool AmeisenNavigation::GetRandomPoint(size_t clientId, int mapId, Vector3& position)
{
    const auto ctx = GetQueryContext(clientId, mapId);

    if (!ctx)
    {
        return false;
    }

    dtPolyRef polyRef = 0;
    const dtStatus status = ctx.query->findRandomPoint(ctx.filter, GetRandomFloat, &polyRef, position);

    if (dtStatusFailed(status) || !polyRef)
    {
        LogD("[", clientId, "] findRandomPoint failed: 0x", std::format("{:08X}", status));
        return false;
    }

    position.ToWowCoords();
    return true;
}

bool AmeisenNavigation::GetRandomPointAround(size_t clientId, int mapId, const Vector3& startPosition, float radius,
                                             Vector3& position)
{
    const auto ctx = GetQueryContext(clientId, mapId);

    if (!ctx || !std::isfinite(radius) || radius < 0.0f || radius > MAX_QUERY_RADIUS)
    {
        return false;
    }

    PolyPosition start;

    if (!FindNearestPolyWow(ctx.query, ctx.filter, startPosition, start))
    {
        return false;
    }

    dtPolyRef polyRef = 0;
    const dtStatus status = ctx.query->findRandomPointAroundCircle(start.poly, start.pos, radius, ctx.filter,
                                                                   GetRandomFloat, &polyRef, position);

    if (dtStatusFailed(status) || !polyRef)
    {
        return false;
    }

    position.ToWowCoords();
    return true;
}

bool AmeisenNavigation::GetHeight(size_t clientId, int mapId, const Vector3& position, Vector3& out)
{
    const auto ctx = GetQueryContext(clientId, mapId);

    Vector3 rdPos;

    if (!ctx || !ToValidRdCoords(position, rdPos))
    {
        return false;
    }

    // Collect every polygon in the vertical column and pick the surface closest to the input height.
    // This handles stacked floors (bridges, buildings) better than a plain nearest poly query.
    constexpr int MAX_COLUMN_POLYS = 64;
    constexpr float COLUMN_EXTENTS[3] = {0.5f, HEIGHT_QUERY_EXTENTS[1], 0.5f};
    dtPolyRef polys[MAX_COLUMN_POLYS]{};
    int polyCount = 0;

    if (dtStatusSucceed(ctx.query->queryPolygons(rdPos, COLUMN_EXTENTS, ctx.filter, polys, &polyCount,
                                                 MAX_COLUMN_POLYS)))
    {
        bool found = false;
        float bestHeight = 0.0f;

        for (int i = 0; i < polyCount; ++i)
        {
            float h = 0.0f;

            if (dtStatusSucceed(ctx.query->getPolyHeight(polys[i], rdPos, &h))
                && (!found || std::fabs(h - rdPos.y) < std::fabs(bestHeight - rdPos.y)))
            {
                bestHeight = h;
                found = true;
            }
        }

        if (found)
        {
            out = Vector3(rdPos.x, bestHeight, rdPos.z).ToWowCoords();
            return true;
        }
    }

    // Not above any polygon (e.g. slightly off-mesh): fall back to the closest point on the mesh.
    PolyPosition nearest;

    if (FindNearestPoly(ctx.query, ctx.filter, rdPos, nearest, HEIGHT_QUERY_EXTENTS))
    {
        out = nearest.pos.ToWowCoords();
        return true;
    }

    return false;
}

bool AmeisenNavigation::CastMovementRay(size_t clientId, int mapId, const Vector3& startPosition,
                                        const Vector3& endPosition, RaycastResult* result)
{
    RaycastResult local;
    RaycastResult& res = result ? *result : local;
    res = RaycastResult{};

    const auto ctx = GetQueryContext(clientId, mapId);

    if (!ctx)
    {
        return false;
    }

    LogD("[", clientId, "] CastMovementRay (", mapId, ") ", startPosition, " -> ", endPosition);

    PolyPosition start;

    if (!FindNearestPolyWow(ctx.query, ctx.filter, startPosition, start))
    {
        return false;
    }

    Vector3 rdEnd;

    if (!ToValidRdCoords(endPosition, rdEnd))
    {
        return false;
    }

    dtRaycastHit hit{};

    if (!Raycast(ctx.query, ctx.filter, start, rdEnd, hit))
    {
        LogD("[", clientId, "] raycast failed");
        return false;
    }

    if (hit.t == FLT_MAX)
    {
        res.hit = false;
        res.t = 1.0f;
        res.hitPosition = endPosition;
        return true;
    }

    Vector3 rdHit;
    dtVlerp(rdHit, start.pos, rdEnd, hit.t);

    res.hit = true;
    res.t = hit.t;
    res.hitPosition = rdHit.ToWowCoords();
    res.hitNormal = Vector3(hit.hitNormal).ToWowCoords();
    return false;
}

bool AmeisenNavigation::Raycast(const dtNavMeshQuery* query, const dtQueryFilter* filter, const PolyPosition& start,
                                const Vector3& rdEnd, dtRaycastHit& hit, dtPolyRef* visited, int maxVisited) noexcept
{
    // Polygons touching the start: the start's own one plus those around it whose surface contains it.
    constexpr int MAX_START_POLYS = 16;
    constexpr float TOUCH_EXTENTS[3]{0.05f, 2.0f, 0.05f};
    dtPolyRef candidates[MAX_START_POLYS]{start.poly};
    int candidateCount = 1;
    dtPolyRef around[MAX_START_POLYS]{};
    int aroundCount = 0;

    if (dtStatusSucceed(query->queryPolygons(start.pos, TOUCH_EXTENTS, filter, around, &aroundCount, MAX_START_POLYS)))
    {
        for (int i = 0; i < aroundCount && candidateCount < MAX_START_POLYS; ++i)
        {
            Vector3 closest;
            bool over = false;

            if (around[i] != start.poly && dtStatusSucceed(query->closestPointOnPoly(around[i], start.pos, closest, &over))
                && dtVdist2DSqr(closest, start.pos) < 1e-4f && std::fabs(closest.y - start.pos.y) < 1.0f)
            {
                candidates[candidateCount++] = around[i];
            }
        }
    }

    const float length = dtVdist(start.pos, rdEnd);
    bool cast = false;

    for (int i = 0; i < candidateCount; ++i)
    {
        // dtRaycastHit has no constructor, path/maxPath MUST be initialized or Detour writes through garbage.
        dtRaycastHit current{};
        current.path = visited;
        current.maxPath = visited ? maxVisited : 0;

        if (dtStatusFailed(query->raycast(candidates[i], start.pos, rdEnd, filter, 0, &current)))
        {
            continue;
        }

        if (current.t != FLT_MAX && length > 0.0f && (1.0f - current.t) * length < RAYCAST_END_TOLERANCE)
        {
            current.t = FLT_MAX;
        }

        if (!cast || current.t > hit.t)
        {
            hit = current;
            cast = true;
        }

        if (hit.t == FLT_MAX)
        {
            break;
        }
    }

    return cast;
}

bool AmeisenNavigation::PostProcessClosestPointOnPoly(size_t clientId, int mapId, const Path& input, Path& output)
{
    output.Clear();
    const auto ctx = GetQueryContext(clientId, mapId);

    if (!ctx)
    {
        return false;
    }

    for (const auto& point : input)
    {
        PolyPosition nearest;

        if (FindNearestPolyWow(ctx.query, ctx.filter, point, nearest))
        {
            if (!output.TryAppendUnique(nearest.pos.ToWowCoords()))
            {
                break;
            }
        }
    }

    return output.pointCount > 0;
}

bool AmeisenNavigation::PostProcessMoveAlongSurface(size_t clientId, int mapId, const Path& input, Path& output)
{
    output.Clear();
    const auto ctx = GetQueryContext(clientId, mapId);

    if (!ctx || input.pointCount == 0)
    {
        return false;
    }

    PolyPosition current;

    if (!FindNearestPolyWow(ctx.query, ctx.filter, input[0], current))
    {
        return false;
    }

    Vector3 wowStart = current.pos;
    output.TryAppend(wowStart.ToWowCoords());

    for (int i = 1; i < input.pointCount && !output.IsFull(); ++i)
    {
        Vector3 target;

        if (!ToValidRdCoords(input[i], target))
        {
            break;
        }

        const Vector3 segmentStart = current.pos;
        const float distance = dtVdist(segmentStart, target);
        const int steps = std::max(1, static_cast<int>(std::ceil(distance / MOVE_ALONG_SURFACE_MAX_CHUNK)));

        for (int s = 1; s <= steps && !output.IsFull(); ++s)
        {
            Vector3 stepTarget;
            dtVlerp(stepTarget, segmentStart, target, static_cast<float>(s) / static_cast<float>(steps));

            if (dtStatusFailed(MoveAlong(ctx.query, ctx.filter, current, stepTarget)))
            {
                return output.pointCount > 0;
            }

            Vector3 wow = current.pos;
            output.TryAppendUnique(wow.ToWowCoords());
        }
    }

    return true;
}

bool AmeisenNavigation::ExplorePolygon(size_t clientId, int mapId, const Vector3& startPosition,
                                       std::span<const Vector3> polygon, float spacing, Path& path,
                                       ExploreResult* result, const PathOptions* options)
{
    path.Clear();
    ExploreResult local;
    ExploreResult& res = result ? *result : local;
    res = ExploreResult{};

    if (polygon.size() < 3 || polygon.size() > static_cast<size_t>(MAX_EXPLORE_POLYGON_POINTS)
        || !IsValidPosition(startPosition) || !std::isfinite(spacing)
        || std::any_of(polygon.begin(), polygon.end(), [](const Vector3& v) { return !IsValidPosition(v); }))
    {
        return false;
    }

    const auto [minX, maxX] = std::minmax_element(polygon.begin(), polygon.end(),
                                                   [](const Vector3& a, const Vector3& b) { return a.x < b.x; });
    const auto [minY, maxY] = std::minmax_element(polygon.begin(), polygon.end(),
                                                   [](const Vector3& a, const Vector3& b) { return a.y < b.y; });

    if (maxX->x - minX->x > MAX_EXPLORE_EXTENT || maxY->y - minY->y > MAX_EXPLORE_EXTENT)
    {
        return false;
    }

    const auto ctx = GetQueryContext(clientId, mapId);

    if (!ctx)
    {
        return false;
    }

    // Grid points inside the polygon, grow the spacing until they fit.
    spacing = std::clamp(spacing, MIN_EXPLORE_SPACING, MAX_EXPLORE_EXTENT);
    std::vector<Vector3> samples;

    for (int attempt = 0; attempt < 32; ++attempt)
    {
        samples = PolygonMath::HexGridSampling(polygon, spacing, MAX_EXPLORE_WAYPOINTS);

        if (!samples.empty())
        {
            break;
        }

        spacing *= 1.5f;
    }

    res.spacing = spacing;

    // The outline's height picks the floor on stacked geometry (caves, buildings, bridges).
    float probeHeight = 0.0f;

    for (const Vector3& v : polygon)
    {
        probeHeight += v.z / static_cast<float>(polygon.size());
    }

    // Snap to the navmesh, keep points still inside the polygon and not too close to another one.
    const float snapDistance = std::min(spacing * 0.5f, MAX_EXPLORE_SNAP_DISTANCE);
    const float snapExtents[3]{snapDistance, HEIGHT_QUERY_EXTENTS[1], snapDistance};
    const float minDistance = spacing * 0.25f;
    std::vector<Vector3> waypoints;
    std::vector<PolyPosition> waypointPolys; // same waypoints on the navmesh (RD coordinates)
    waypoints.reserve(samples.size());
    waypointPolys.reserve(samples.size());

    for (const Vector3& sample : samples)
    {
        PolyPosition snapped;

        if (!FindNearestPoly(ctx.query, ctx.filter, Vector3(sample.y, probeHeight, sample.x), snapped, snapExtents))
        {
            continue;
        }

        Vector3 wow = snapped.pos;
        wow.ToWowCoords();

        if (!PolygonMath::IsInside2D(polygon.data(), static_cast<int>(polygon.size()), wow)
            || std::any_of(waypoints.begin(), waypoints.end(),
                           [&](const Vector3& w) { return w.DistanceTo(wow) < minDistance; }))
        {
            continue;
        }

        waypoints.push_back(wow);
        waypointPolys.push_back(snapped);
    }

    res.waypoints = static_cast<int>(waypoints.size());
    PolyPosition current;

    if (waypoints.empty() || !FindNearestPolyWow(ctx.query, ctx.filter, startPosition, current))
    {
        return false;
    }

    // Connect the waypoints in tour order with navmesh paths. Segments use their own buffer, the client's
    // path buffers may be the output.
    Path segment(Settings.maxPointPath);
    std::vector<unsigned char> routeFlags; // PathPointFlags of the route points
    routeFlags.reserve(static_cast<size_t>(path.maxSize));
    dtPolyRef* segmentRefs = ctx.client->GetStraightPathRefBuffer();
    unsigned char* segmentFlags = ctx.client->GetStraightPathFlagBuffer();
    const std::vector<int> order = Tour::Order(startPosition, waypoints, 8, spacing * 0.5f);
    std::vector<bool> unreachable(waypoints.size(), false);

    for (size_t o = 0; o < order.size(); ++o)
    {
        const auto index = static_cast<size_t>(order[o]);

        if (unreachable[index])
        {
            continue;
        }

        const PolyPosition& target = waypointPolys[index];
        bool partial = false;

        if (!CalculateNormalPath(ctx.query, ctx.filter, *ctx.client, current, target, segment, segmentRefs, &partial,
                                 segmentFlags))
        {
            continue;
        }

        if (partial)
        {
            // If the search neither found the target nor ran out of nodes, A* closed every polygon reachable
            // from here. The route only continues to waypoints reached from here, so everything reachable from
            // them is reachable from here too (transitivity, also with one-way off-mesh links): waypoints outside
            // the closed list can't be reached from anywhere later on the route, skip them without searching.
            const dtNodePool* pool = ctx.query->getNodePool();

            if (pool && pool->getNodeCount() < pool->getMaxNodes()
                && !ctx.query->isInClosedList(target.poly))
            {
                for (size_t k = o + 1; k < order.size(); ++k)
                {
                    const auto other = static_cast<size_t>(order[k]);
                    unreachable[other] = unreachable[other] || !ctx.query->isInClosedList(waypointPolys[other].poly);
                }
            }

            continue; // other island, blocked by the filter, ...
        }

        ToPointFlags(ctx.query, segmentFlags, segmentRefs, segment.pointCount, segmentFlags);

        // The route stays in RD coordinates until it is complete.
        for (int i = path.Empty() ? 0 : 1; i < segment.pointCount; ++i)
        {
            const unsigned char f = segmentFlags[i] & ~(PATH_POINT_START | PATH_POINT_END);

            if (path.pointCount > 0 && path[path.pointCount - 1] == segment[i])
            {
                routeFlags.back() |= f;
                continue;
            }

            if (!path.TryAppend(segment[i]))
            {
                res.truncated = true;
                break;
            }

            routeFlags.push_back(f);
        }

        if (res.truncated)
        {
            break;
        }

        res.reached++;
        current = target;
    }

    if (((options && (options->avoidWalls || options->roundCorners)) || HasZones(ctx.filter)) && path.pointCount > 1)
    {
        // Shortcut, stop distance and follow don't apply: the route has to visit its waypoints.
        PathOptions corners;

        if (options)
        {
            corners.avoidWalls = options->avoidWalls;
            corners.roundCorners = options->roundCorners;
            corners.tuning = options->tuning;
        }

        routeFlags.resize(static_cast<size_t>(path.maxSize), 0);
        RefineAll(ctx.query, ctx.filter, path, routeFlags.data(), Vector3{}, options ? &corners : nullptr, false);
    }

    path.ToWowCoords();

    LogD("[", clientId, "] ExplorePolygon (", mapId, ") ", polygon.size(), " vertices, spacing ", spacing, ": ",
         res.reached, "/", res.waypoints, " waypoints, ", path.pointCount, " points",
         res.truncated ? " (truncated)" : "");
    return res.reached > 0 && path.pointCount > 0;
}

void AmeisenNavigation::SmoothPathChaikinCurve(const Path& input, Path& output) const noexcept
{
    ChaikinCurve::SmoothPath(input.points, input.pointCount, output);
}

void AmeisenNavigation::SmoothPathCatmullRom(const Path& input, Path& output, int points, float alpha) const noexcept
{
    CatmullRomSpline::SmoothPath(input.points, input.pointCount, output, points, alpha);
}

void AmeisenNavigation::SmoothPathBezier(const Path& input, Path& output, int points) const noexcept
{
    BezierCurve::SmoothPath(input.points, input.pointCount, output, points);
}

AmeisenNavigation::QueryContext AmeisenNavigation::GetQueryContext(size_t clientId, int mapId)
{
    QueryContext ctx;
    ctx.client = GetClient(clientId);

    if (!ctx.client)
    {
        LogE("Unknown client: ", clientId);
        return {};
    }

    ctx.filter = ctx.client->QueryFilter(mapId, GlobalZones.Get(mapId));

    if (!ctx.filter)
    {
        return {};
    }

    NavMeshQueryPool* pool = GetQueryPool(mapId);

    if (!pool)
    {
        LogD("[", clientId, "] No navmesh available for map ", mapId); // the nav source warned once
        return {};
    }

    ctx.meshLock = NavMeshLock::Shared(pool->Lock());
    ctx.lease = pool->Acquire();

    if (!ctx.lease)
    {
        LogE("[", clientId, "] Failed to create a dtNavMeshQuery for map ", mapId);
        return {};
    }

    ctx.query = ctx.lease.get();
    return ctx;
}

NavMeshQueryPool* AmeisenNavigation::GetQueryPool(int mapId)
{
    {
        std::shared_lock lock(QueryPoolsMutex);
        const auto it = QueryPools.find(mapId);

        if (it != QueryPools.end())
        {
            return it->second.get();
        }
    }

    // Load outside the pool lock, loading a big map mustn't block requests for other maps.
    const dtNavMesh* navMesh = NavSource->Get(mapId);

    if (!navMesh)
    {
        return nullptr;
    }

    std::unique_lock lock(QueryPoolsMutex);
    auto& pool = QueryPools[mapId];

    if (!pool)
    {
        pool = std::make_unique<NavMeshQueryPool>(navMesh, Settings.maxSearchNodes, Settings.maxIdleQueriesPerMap);
    }

    return pool.get();
}

size_t AmeisenNavigation::GetQueryCount(int mapId) const
{
    std::shared_lock lock(QueryPoolsMutex);
    const auto it = QueryPools.find(mapId);
    return it != QueryPools.end() ? it->second->GetCreatedCount() : 0;
}

bool AmeisenNavigation::CalculateNormalPath(dtNavMeshQuery* query, const dtQueryFilter* filter,
                                            AmeisenNavClient& client, const Vector3& startPosition,
                                            const Vector3& endPosition, Path& path, dtPolyRef* straightPathRefs,
                                            bool* partial, unsigned char* straightPathFlags) noexcept
{
    path.Clear();

    if (partial)
    {
        *partial = false;
    }

    PolyPosition start; // FindNearestPoly rejects invalid positions
    PolyPosition end;

    if (!FindNearestPolyWow(query, filter, startPosition, start))
    {
        LogD("No poly near start position ", startPosition);
        return false;
    }

    if (!FindNearestPolyWow(query, filter, endPosition, end))
    {
        LogD("No poly near end position ", endPosition);
        return false;
    }

    return CalculateNormalPath(query, filter, client, start, end, path, straightPathRefs, partial, straightPathFlags);
}

bool AmeisenNavigation::CalculateNormalPath(dtNavMeshQuery* query, const dtQueryFilter* filter,
                                            AmeisenNavClient& client, const PolyPosition& start,
                                            const PolyPosition& end, Path& path, dtPolyRef* straightPathRefs,
                                            bool* partial, unsigned char* straightPathFlags) noexcept
{
    path.Clear();

    if (partial)
    {
        *partial = false;
    }

    dtPolyRef* polyPath = client.GetPolyPathBuffer();
    int polyPathCount = 0;

    const dtStatus polyPathStatus = query->findPath(start.poly, end.poly, start.pos, end.pos, filter, polyPath,
                                                    &polyPathCount, client.GetPolyPathBufferSize());

    if (dtStatusFailed(polyPathStatus) || polyPathCount <= 0)
    {
        LogD("findPath failed: 0x", std::format("{:08X}", polyPathStatus));
        return false;
    }

    if (dtStatusDetail(polyPathStatus, DT_PARTIAL_RESULT))
    {
        LogD("findPath: partial result (end not reachable), ", polyPathCount, " polys");
    }

    // For partial paths, aim at the closest point of the last reached polygon instead of the unreachable end.
    Vector3 endPos = end.pos;

    // DT_PARTIAL_RESULT or a poly path cut at the buffer size: either way the end poly wasn't reached.
    if (polyPath[polyPathCount - 1] != end.poly)
    {
        query->closestPointOnPoly(polyPath[polyPathCount - 1], end.pos, endPos, nullptr);

        if (partial)
        {
            *partial = true;
        }
    }

    // Corner refs/flags go into the client's buffers, never write more corners than they hold.
    const int maxCorners = straightPathRefs || straightPathFlags
                               ? std::min(path.maxSize, client.GetStraightPathRefBufferSize())
                               : path.maxSize;

    const dtStatus straightPathStatus =
        SafeFindStraightPath(query, start.pos, endPos, polyPath, polyPathCount, reinterpret_cast<float*>(path.points),
                             straightPathFlags, straightPathRefs, &path.pointCount, maxCorners);

    if (dtStatusFailed(straightPathStatus) || path.pointCount <= 0)
    {
        LogD("findStraightPath failed: 0x", std::format("{:08X}", straightPathStatus));
        path.Clear();
        return false;
    }

    return true;
}

bool AmeisenNavigation::LineOfSight(int mapId, const Vector3& start, const Vector3& end, uint32_t ignore,
                                    World::Hit& hit)
{
    World::StaticWorld* world = Worlds.Get(mapId);
    return world && world->Raycast(start, end, ignore | World::IGNORE_LIQUIDS, hit);
}

bool AmeisenNavigation::GetSurfaces(int mapId, float x, float y, float zTop, float zBottom, uint32_t ignore,
                                    std::vector<World::Surface>& surfaces, uint32_t& areaId)
{
    World::StaticWorld* world = Worlds.Get(mapId);
    return world && world->Surfaces(x, y, zTop, zBottom, ignore, surfaces, areaId);
}

bool AmeisenNavigation::SweepBody(int mapId, const Vector3& start, const Vector3& end, float radius, float height,
                                  float stepHeight, uint32_t ignore, World::Hit& hit)
{
    World::StaticWorld* world = Worlds.Get(mapId);
    return world && world->Sweep(start, end, radius, height, stepHeight, ignore, hit);
}

int AmeisenNavigation::ReplaceTiles(int mapId, const std::vector<std::vector<unsigned char>>& tiles)
{
    NavMeshQueryPool* pool = GetQueryPool(mapId);
    dtNavMesh* navMesh = pool ? NavSource->Get(mapId) : nullptr;

    if (!navMesh)
    {
        return 0;
    }

    // File links are read before taking the lock (the file can be slow, requests wait while it is held).
    std::vector<OffMeshLink> fileLinks;

    if (!Settings.offMeshFile.empty())
    {
        unsigned short groundFlags = NAV_GROUND;

        if (!Settings.useAnp)
        {
            groundFlags = Settings.mmapFormat == MmapFormat::SF548 ? static_cast<unsigned short>(NavArea548::GROUND)
                                                                   : static_cast<unsigned short>(NavFlag335a::GROUND);
        }

        fileLinks = OffMeshLinks::ReadFile(Settings.offMeshFile, mapId, groundFlags);
    }

    struct Exclusive
    {
        NavMeshLock& lock;
        explicit Exclusive(NavMeshLock& l) : lock(l) { lock.Lock(); }
        ~Exclusive() { lock.Unlock(); }
    } exclusive(pool->Lock());

    int replaced = 0;

    for (const auto& data : tiles)
    {
        if (data.empty() || !ValidateTileData(data.data(), data.size()))
        {
            continue;
        }

        dtMeshHeader header;
        std::memcpy(&header, data.data(), sizeof(header));

        if (const dtTileRef existing = navMesh->getTileRefAt(header.x, header.y, header.layer); existing)
        {
            navMesh->removeTile(existing, nullptr, nullptr);
        }

        auto* copy = static_cast<unsigned char*>(dtAlloc(static_cast<int>(data.size()), DT_ALLOC_PERM));

        if (!copy)
        {
            continue;
        }

        std::memcpy(copy, data.data(), data.size());

        if (dtStatusFailed(navMesh->addTile(copy, static_cast<int>(data.size()), DT_TILE_FREE_DATA, 0, nullptr)))
        {
            dtFree(copy);
            LogE("Map ", mapId, ": failed to add rebuilt tile ", header.x, ",", header.y);
            continue;
        }

        replaced++;

        // Links starting or ending in the new tile (the neighbours still have theirs).
        std::vector<const OffMeshLink*> links;

        for (const OffMeshLink& link : fileLinks)
        {
            int sx = 0, sy = 0, ex = 0, ey = 0;
            navMesh->calcTileLoc(link.start, &sx, &sy);
            navMesh->calcTileLoc(link.end, &ex, &ey);

            if ((sx == header.x && sy == header.y) || (ex == header.x && ey == header.y))
            {
                links.push_back(&link);
            }
        }

        if (!links.empty())
        {
            TileRebuilder::Rebuild(*navMesh, navMesh->getTileAt(header.x, header.y, header.layer), links);
        }
    }

    return replaced;
}
