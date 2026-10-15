#include "NavServer.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <format>
#include <algorithm>
#include <functional>
#include <span>
#include <thread>
#include <vector>

namespace {
/// Copy a request out of the (possibly unaligned) receive buffer. Returns false if the packet is too small.
template <typename T>
bool ReadRequest(const void* data, int size, T& out) noexcept
{
    static_assert(std::is_trivially_copyable_v<T>);

    if (!data || size < static_cast<int>(sizeof(T)))
    {
        return false;
    }

    std::memcpy(&out, data, sizeof(T));
    return true;
}

bool SendVector(ClientHandler* handler, AnTcpMessageType type, const Vector3& v) noexcept
{
    return handler->SendData(type, &v, sizeof(Vector3));
}

bool SendZero(ClientHandler* handler, AnTcpMessageType type) noexcept { return SendVector(handler, type, Vector3{}); }

const char* MessageName(AnTcpMessageType type) noexcept
{
    switch (static_cast<MessageType>(type))
    {
        case MessageType::PATH: return "Path";
        case MessageType::MOVE_ALONG_SURFACE: return "MoveAlongSurface";
        case MessageType::RANDOM_POINT: return "RandomPoint";
        case MessageType::RANDOM_POINT_AROUND: return "RandomPointAround";
        case MessageType::CAST_RAY: return "CastRay";
        case MessageType::RANDOM_PATH: return "RandomPath";
        case MessageType::EXPLORE_POLY: return "ExplorePoly";
        case MessageType::CONFIGURE_FILTER: return "ConfigureFilter";
        case MessageType::GET_HEIGHT: return "GetHeight";
        case MessageType::GET_CONFIG: return "GetConfig";
        case MessageType::CAST_RAY_EX: return "CastRayEx";
        case MessageType::SET_AVOID_ZONES: return "SetAvoidZones";
        case MessageType::STEER: return "Steer";
        case MessageType::REPORT_STUCK: return "ReportStuck";
        case MessageType::PATH_EX: return "PathEx";
        case MessageType::LINE_OF_SIGHT: return "LineOfSight";
        case MessageType::SURFACES: return "Surfaces";
        case MessageType::SWEEP: return "Sweep";
        case MessageType::SET_OBJECTS: return "SetObjects";
        default: return "Unknown";
    }
}

void LogTooSmall(ClientHandler* handler, AnTcpMessageType type, int size, size_t expected)
{
    // Debug level: a misbehaving client could otherwise flood the log.
    LogD("[", handler->GetId(), "] ", MessageName(type), ": packet too small (", size, " < ", expected, ")");
}
} // namespace

NavServer::NavServer(const AmeisenNavConfig& config) : Cfg(config)
{
    AmeisenNavigationSettings settings;
    settings.meshFolder = std::filesystem::path(Cfg.mmapsPath);
    settings.useAnp = Cfg.useAnpFileFormat;
    settings.mmapFormat = ToMmapFormat(Cfg.mmapFormat);
    settings.customMmapPatterns = {Cfg.customMmapPattern, Cfg.customMmtilePattern};
    AmeisenNavConfig::ParseMapPairs(Cfg.parentMaps, &settings.parentMaps);
    settings.maxPolyPath = Cfg.maxPolyPath;
    settings.maxPointPath = Cfg.maxPointPath;
    settings.maxSearchNodes = Cfg.maxSearchNodes;
    settings.factionDangerCost = Cfg.factionDangerCost;
    settings.waterCost = Cfg.waterCost;
    settings.badLiquidCost = Cfg.badLiquidCost;
    settings.roadCost = Cfg.roadCost;
    settings.avoidZonesFile = Cfg.avoidZonesFile;
    settings.stuckZoneRadius = Cfg.stuckZoneRadius;
    settings.stuckZoneCost = Cfg.stuckZoneCost;
    settings.offMeshFile = Cfg.offMeshFile;
    settings.autoDropHeight = Cfg.autoDropHeight;
    settings.autoDropReach = Cfg.autoDropReach;
    settings.worldFolder = Cfg.worldPath;
    settings.worldCacheTiles = static_cast<size_t>(std::max(1, Cfg.worldCacheTiles));

    Navigation = std::make_unique<AmeisenNavigation>(settings);
    Dynamic = std::make_unique<DynamicWorld>(*Navigation,
                                             settings.worldFolder.empty() ? settings.meshFolder : settings.worldFolder);
    TcpServer = std::make_unique<AnTcpServer>(Cfg.ip, static_cast<unsigned short>(Cfg.port));
    TcpServer->SetMaxClients(static_cast<size_t>(Cfg.maxClients));
    TcpServer->SetIdleTimeout(std::chrono::seconds(Cfg.clientIdleTimeoutSec));

    RegisterCallbacks();
}

MmapFormat NavServer::ToMmapFormat(int configValue) noexcept
{
    switch (configValue)
    {
        case -1: return MmapFormat::CUSTOM;
        case 1: return MmapFormat::TC335A;
        case 2: return MmapFormat::SF548;
        case 3: return MmapFormat::TC_MASTER;
        default: return MmapFormat::UNKNOWN;
    }
}

void NavServer::ValidateConfig(AmeisenNavConfig& config, std::vector<std::string>& errors,
                               std::vector<std::string>& warnings)
{
    std::error_code ec;

    if (config.mmapsPath.empty() || !std::filesystem::is_directory(config.mmapsPath, ec))
    {
        errors.push_back(std::format("sMmapsPath is not a directory: \"{}\"", config.mmapsPath));
    }

    if (config.port < 0 || config.port > 65535)
    {
        errors.push_back("iPort has to be a value between 0 and 65535");
    }

    if (config.maxPolyPath <= 0)
    {
        errors.push_back("iMaxPolyPath has to be a value > 0");
    }

    if (config.maxSearchNodes <= 0 || config.maxSearchNodes > 65535)
    {
        errors.push_back("iMaxSearchNodes has to be a value between 1 and 65535");
    }

    if (config.maxClients < 0)
    {
        errors.push_back("iMaxClients has to be >= 0 (0 = unlimited)");
    }

    if (config.statsIntervalSec < 0)
    {
        errors.push_back("iStatsIntervalSec has to be >= 0 (0 = off)");
    }

    if (!(config.bodyRadius >= 0.0f && config.bodyRadius <= 5.0f) || !(config.bodyHeight >= 0.5f && config.bodyHeight <= 10.0f)
        || !(config.bodyStepHeight >= 0.0f && config.bodyStepHeight < config.bodyHeight))
    {
        errors.push_back("fBodyRadius has to be 0-5, fBodyHeight 0.5-10, fBodyStepHeight below fBodyHeight");
    }

    if (!(config.waypointSpacing >= MIN_WAYPOINT_SPACING && config.waypointSpacing <= MAX_WAYPOINT_SPACING)
        || !(config.waypointHeightError >= MIN_WAYPOINT_HEIGHT_ERROR && config.waypointHeightError <= 10.0f))
    {
        errors.push_back("fWaypointSpacing has to be 1-100, fWaypointHeightError 0.05-10");
    }

    if (!(config.edgeMargin >= 0.0f && config.edgeMargin <= MAX_WALL_MARGIN))
    {
        errors.push_back("fEdgeMargin has to be 0-10");
    }

    if (config.worldCacheTiles < 1 || config.worldCacheTiles > 4096)
    {
        errors.push_back("iWorldCacheTiles has to be between 1 and 4096");
    }

    if (config.clientIdleTimeoutSec < 0)
    {
        errors.push_back("iClientIdleTimeoutSec has to be >= 0 (0 = never)");
    }

    if (config.maxPointPath < 2)
    {
        errors.push_back("iMaxPointPath has to be a value >= 2");
    }
    else if (sizeof(PathExResponseHeader) + static_cast<size_t>(config.maxPointPath) * (sizeof(Vector3) + 7) + 1 >
             ANTCP_MAX_RESPONSE_SIZE)
    {
        errors.push_back("iMaxPointPath is too large for a single response");
    }

    if (config.mmapFormat < -1 || config.mmapFormat > 3)
    {
        errors.push_back(
            "iMmapFormat has to be -1 (custom), 0 (auto), 1 (TC 3.3.5a), 2 (SkyFire 5.4.8) or 3 (TC master)");
    }

    if (std::string error; !AmeisenNavConfig::ParseMapPairs(config.parentMaps, nullptr, &error))
    {
        errors.push_back("sParentMaps: " + error);
    }

    if (config.mmapFormat == -1 && (config.customMmapPattern.empty() || config.customMmtilePattern.empty()))
    {
        errors.push_back("iMmapFormat=-1 requires sCustomMmapPattern and sCustomMmtilePattern");
    }

    if (config.bezierCurvePoints < 2)
    {
        warnings.push_back("iBezierCurvePoints too low, clamping to 2");
        config.bezierCurvePoints = 2;
    }

    if (config.catmullRomSplinePoints < 1)
    {
        warnings.push_back("iCatmullRomSplinePoints too low, clamping to 1");
        config.catmullRomSplinePoints = 1;
    }

    if (config.catmullRomSplineAlpha < 0.0f || config.catmullRomSplineAlpha > 1.0f)
    {
        warnings.push_back("fCatmullRomSplineAlpha has to be within [0, 1], clamping");
        config.catmullRomSplineAlpha = std::clamp(config.catmullRomSplineAlpha, 0.0f, 1.0f);
    }

    if (config.factionDangerCost < 0.01f)
    {
        warnings.push_back("fFactionDangerCost too low, clamping to 0.01");
        config.factionDangerCost = 0.01f;
    }

    const std::pair<const char*, float*> costs[]{
        {"fWaterCost", &config.waterCost}, {"fBadLiquidCost", &config.badLiquidCost}, {"fRoadCost", &config.roadCost}};

    for (const auto& cost : costs)
    {
        if (*cost.second < 0.01f)
        {
            warnings.push_back(std::format("{} too low, clamping to 0.01", cost.first));
            *cost.second = 0.01f;
        }
    }

    const std::pair<const char*, float*> distances[]{{"fWallMargin", &config.wallMargin},
                                                      {"fCornerRadius", &config.cornerRadius}};

    for (const auto& distance : distances)
    {
        const float limit = distance.second == &config.wallMargin ? MAX_WALL_MARGIN : MAX_CORNER_RADIUS;

        if (*distance.second < 0.0f || *distance.second > limit)
        {
            warnings.push_back(std::format("{} has to be within [0, {}], clamping", distance.first, limit));
            *distance.second = std::clamp(*distance.second, 0.0f, limit);
        }
    }

    if (config.cornerPoints < 1 || config.cornerPoints > MAX_CORNER_POINTS)
    {
        warnings.push_back(std::format("iCornerPoints has to be within [1, {}], clamping", MAX_CORNER_POINTS));
        config.cornerPoints = std::clamp(config.cornerPoints, 1, MAX_CORNER_POINTS);
    }

    if (config.stuckZoneRadius <= 0.0f || config.stuckZoneRadius > MAX_AVOID_ZONE_RADIUS)
    {
        warnings.push_back(std::format("fStuckZoneRadius has to be within (0, {}], clamping", MAX_AVOID_ZONE_RADIUS));
        config.stuckZoneRadius = std::clamp(config.stuckZoneRadius, 0.5f, MAX_AVOID_ZONE_RADIUS);
    }

    if (config.stuckZoneCost < 1.0f || config.stuckZoneCost > MAX_AVOID_ZONE_COST)
    {
        warnings.push_back(std::format("fStuckZoneCost has to be within [1, {}], clamping", MAX_AVOID_ZONE_COST));
        config.stuckZoneCost = std::clamp(config.stuckZoneCost, 1.0f, MAX_AVOID_ZONE_COST);
    }

    if (!config.offMeshFile.empty() && !std::filesystem::is_regular_file(config.offMeshFile, ec))
    {
        warnings.push_back(std::format("sOffMeshFile not found: \"{}\"", config.offMeshFile));
    }

    if (config.autoDropHeight < 0.0f || config.autoDropHeight > 50.0f)
    {
        warnings.push_back("fAutoDropHeight has to be within [0, 50] (0 = off), clamping");
        config.autoDropHeight = std::clamp(config.autoDropHeight, 0.0f, 50.0f);
    }

    if (config.autoDropReach < 0.5f || config.autoDropReach > 10.0f)
    {
        warnings.push_back("fAutoDropReach has to be within [0.5, 10], clamping");
        config.autoDropReach = std::clamp(config.autoDropReach, 0.5f, 10.0f);
    }

    if (config.randomPathMaxDistance < 0.0f)
    {
        warnings.push_back("fRandomPathMaxDistance negative, clamping to 0");
        config.randomPathMaxDistance = 0.0f;
    }
}

void NavServer::PreloadMaps()
{
    const auto load = [this](int mapId) {
        if (!Navigation->PreloadMap(mapId))
        {
            LogW("Preload: no navmesh for map ", mapId);
        }
    };

    // Maps load concurrently: every load has a single-threaded phase (dtNavMesh::addTile) that would add up.
    const auto maps = Cfg.GetPreloadMaps();
    std::vector<std::thread> loaders;
    loaders.reserve(maps.size());

    for (const int mapId : maps)
    {
        try
        {
            loaders.emplace_back(load, mapId);
        }
        catch (...)
        {
            load(mapId); // no thread available
        }
    }

    for (auto& loader : loaders)
    {
        loader.join();
    }
}

void NavServer::RegisterCallbacks()
{
    auto& s = *TcpServer;

    s.SetOnClientConnected([this](ClientHandler* h) { OnClientConnect(h); });
    s.SetOnClientDisconnected([this](ClientHandler* h) { OnClientDisconnect(h); });

    // A connection flood would otherwise log one line per refused connection: at most one warning per 10s.
    s.SetOnClientRejected([this](size_t active) {
        using namespace std::chrono;
        const long long now = duration_cast<seconds>(steady_clock::now().time_since_epoch()).count();
        long long last = LastRejectLog.load(std::memory_order_relaxed);

        if ((last == 0 || now - last >= 10) && LastRejectLog.compare_exchange_strong(last, now))
        {
            LogW("Client limit reached (", active, " connected, iMaxClients=", Cfg.maxClients,
                 "), refusing new connections (", TcpServer->GetRejectedCount(), " refused so far)");
        }
    });

    // Every handler is timed for the stats and the debug log.
    const auto add = [&](MessageType type, auto handler) {
        s.AddCallback(static_cast<AnTcpMessageType>(type),
                      [this, handler](ClientHandler* h, AnTcpMessageType t, const void* d, int sz) {
                          const auto start = std::chrono::steady_clock::now();
                          handler(h, t, d, sz);
                          const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
                                                  std::chrono::steady_clock::now() - start)
                                                  .count();
                          RecordRequest(t, static_cast<uint64_t>(micros));
                          LogD("[", h->GetId(), "] ", MessageName(t), " took ", micros, "us");
                      });
    };

    add(MessageType::PATH, [this](ClientHandler* h, AnTcpMessageType t, const void* d, int sz) {
        HandlePath(h, t, d, sz, PathType::STRAIGHT);
    });
    add(MessageType::RANDOM_PATH, [this](ClientHandler* h, AnTcpMessageType t, const void* d, int sz) {
        HandlePath(h, t, d, sz, PathType::RANDOM);
    });
    add(MessageType::MOVE_ALONG_SURFACE, std::bind_front(&NavServer::HandleMoveAlongSurface, this));
    add(MessageType::CAST_RAY, std::bind_front(&NavServer::HandleCastRay, this));
    add(MessageType::CAST_RAY_EX, std::bind_front(&NavServer::HandleCastRayEx, this));
    add(MessageType::RANDOM_POINT, std::bind_front(&NavServer::HandleRandomPoint, this));
    add(MessageType::RANDOM_POINT_AROUND, std::bind_front(&NavServer::HandleRandomPointAround, this));
    add(MessageType::CONFIGURE_FILTER, std::bind_front(&NavServer::HandleConfigureFilter, this));
    add(MessageType::GET_HEIGHT, std::bind_front(&NavServer::HandleGetHeight, this));
    add(MessageType::GET_CONFIG, std::bind_front(&NavServer::HandleGetConfig, this));
    add(MessageType::EXPLORE_POLY, std::bind_front(&NavServer::HandleExplorePoly, this));
    add(MessageType::SET_AVOID_ZONES, std::bind_front(&NavServer::HandleSetAvoidZones, this));
    add(MessageType::STEER, std::bind_front(&NavServer::HandleSteer, this));
    add(MessageType::REPORT_STUCK, std::bind_front(&NavServer::HandleReportStuck, this));
    add(MessageType::PATH_EX, std::bind_front(&NavServer::HandlePathEx, this));
    add(MessageType::LINE_OF_SIGHT, std::bind_front(&NavServer::HandleLineOfSight, this));
    add(MessageType::SURFACES, std::bind_front(&NavServer::HandleSurfaces, this));
    add(MessageType::SWEEP, std::bind_front(&NavServer::HandleSweep, this));
    add(MessageType::SET_OBJECTS, std::bind_front(&NavServer::HandleSetObjects, this));
}

AnTcpError NavServer::Run() noexcept
{
    std::thread reporter;

    {
        const std::lock_guard lock(StatsMutex);
        StatsStop = false;
    }

    if (Cfg.statsIntervalSec > 0)
    {
        try
        {
            reporter = std::thread([this] { StatsLoop(); });
        }
        catch (...)
        {
            LogW("Failed to start the stats thread, request stats are disabled");
        }
    }

    const AnTcpError result = TcpServer->Run();

    {
        const std::lock_guard lock(StatsMutex);
        StatsStop = true;
    }

    StatsCv.notify_all();

    if (reporter.joinable())
    {
        reporter.join();
    }

    return result;
}

void NavServer::RecordRequest(AnTcpMessageType type, uint64_t micros) noexcept
{
    auto& stats = Stats[type % STATS_SLOTS];
    stats.count.fetch_add(1, std::memory_order_relaxed);
    stats.totalMicros.fetch_add(micros, std::memory_order_relaxed);

    uint64_t max = stats.maxMicros.load(std::memory_order_relaxed);

    while (micros > max && !stats.maxMicros.compare_exchange_weak(max, micros, std::memory_order_relaxed))
    {
    }
}

std::string NavServer::TakeStatsSummary(double seconds)
{
    std::string details;
    uint64_t total = 0;

    for (size_t i = 0; i < STATS_SLOTS; ++i)
    {
        const uint64_t count = Stats[i].count.exchange(0, std::memory_order_relaxed);
        const uint64_t micros = Stats[i].totalMicros.exchange(0, std::memory_order_relaxed);
        const uint64_t max = Stats[i].maxMicros.exchange(0, std::memory_order_relaxed);

        if (count == 0)
        {
            continue;
        }

        total += count;
        details += std::format(" | {} {} avg {:.2f}ms max {:.2f}ms", MessageName(static_cast<AnTcpMessageType>(i)),
                               count, static_cast<double>(micros) / static_cast<double>(count) / 1000.0,
                               static_cast<double>(max) / 1000.0);
    }

    if (total == 0)
    {
        return {};
    }

    return std::format("Stats {:.0f}s: {} requests ({:.1f}/s), {} clients, {} refused{}", seconds, total,
                       seconds > 0.0 ? static_cast<double>(total) / seconds : 0.0, TcpServer->GetClientCount(),
                       TcpServer->GetRejectedCount(), details);
}

void NavServer::StatsLoop()
{
    auto last = std::chrono::steady_clock::now();
    std::unique_lock lock(StatsMutex);

    while (!StatsCv.wait_for(lock, std::chrono::seconds(Cfg.statsIntervalSec), [this] { return StatsStop; }))
    {
        const auto now = std::chrono::steady_clock::now();
        lock.unlock();

        const std::string summary = TakeStatsSummary(std::chrono::duration<double>(now - last).count());
        last = now;

        if (!summary.empty())
        {
            LogI(summary);
        }

        lock.lock();
    }
}

void NavServer::OnClientConnect(ClientHandler* handler)
{
    LogI("Client connected: ", handler->GetIpAddress(), ":", handler->GetPort(), " (id ", handler->GetId(), ")");
    Navigation->NewClient(handler->GetId());
}

void NavServer::OnClientDisconnect(ClientHandler* handler)
{
    Navigation->FreeClient(handler->GetId());
    LogI("Client disconnected: ", handler->GetIpAddress(), ":", handler->GetPort(), " (id ", handler->GetId(), ")");
}

PathOptions NavServer::MakePathOptions(int flags, const PathTuningData* tuning,
                                       const PathDetailData* detail) const noexcept
{
    PathOptions options;
    options.avoidWalls = HasFlag(flags, PathRequestFlags::AVOID_WALLS);
    options.shortcut = HasFlag(flags, PathRequestFlags::SHORTCUT);
    options.roundCorners = HasFlag(flags, PathRequestFlags::ROUND_CORNERS);
    options.follow = HasFlag(flags, PathRequestFlags::FOLLOW);
    options.clearBody = HasFlag(flags, PathRequestFlags::CLEAR_BODY);
    options.detail = HasFlag(flags, PathRequestFlags::DETAIL);
    options.waypointSpacing = Cfg.waypointSpacing;
    options.waypointHeightError = Cfg.waypointHeightError;

    if (detail)
    {
        if (std::isfinite(detail->spacing) && detail->spacing > 0.0f)
        {
            options.waypointSpacing = std::clamp(detail->spacing, MIN_WAYPOINT_SPACING, MAX_WAYPOINT_SPACING);
        }

        if (std::isfinite(detail->heightError) && detail->heightError > 0.0f)
        {
            options.waypointHeightError = std::clamp(detail->heightError, MIN_WAYPOINT_HEIGHT_ERROR, 10.0f);
        }
    }
    options.edgeMargin = Cfg.edgeMargin;
    options.bodyRadius = Cfg.bodyRadius;
    options.bodyHeight = Cfg.bodyHeight;
    options.bodyStepHeight = Cfg.bodyStepHeight;
    options.tuning.wallMargin = Cfg.wallMargin;
    options.tuning.cornerRadius = Cfg.cornerRadius;
    options.tuning.cornerPoints = Cfg.cornerPoints;

    if (tuning)
    {
        const auto use = [](float value, float& field, float limit) {
            if (std::isfinite(value) && value >= 0.0f)
            {
                field = std::min(value, limit);
            }
        };

        use(tuning->wallMargin, options.tuning.wallMargin, MAX_WALL_MARGIN);
        use(tuning->cornerRadius, options.tuning.cornerRadius, MAX_CORNER_RADIUS);
        use(tuning->stopDistance, options.tuning.stopDistance, MAX_STOP_DISTANCE);

        if (tuning->cornerPoints >= 0)
        {
            options.tuning.cornerPoints = std::clamp(tuning->cornerPoints, 1, MAX_CORNER_POINTS);
        }
    }

    return options;
}

Path* NavServer::ApplyPathFlags(size_t clientId, int mapId, int flags, PathType pathType, Path& path, Path& scratch)
{
    Path* current = &path;
    Path* spare = &scratch;
    bool smoothed = false;

    if (HasFlag(flags, PathRequestFlags::SMOOTH_CHAIKIN) && current->pointCount >= 3)
    {
        Navigation->SmoothPathChaikinCurve(*current, *spare);
        smoothed = true;
    }
    else if (HasFlag(flags, PathRequestFlags::SMOOTH_CATMULLROM) && current->pointCount >= 3)
    {
        Navigation->SmoothPathCatmullRom(*current, *spare, Cfg.catmullRomSplinePoints, Cfg.catmullRomSplineAlpha);
        smoothed = true;
    }
    else if (HasFlag(flags, PathRequestFlags::SMOOTH_BEZIERCURVE) && current->pointCount >= 4)
    {
        Navigation->SmoothPathBezier(*current, *spare, Cfg.bezierCurvePoints);
        smoothed = true;
    }

    if (smoothed && spare->pointCount > 0)
    {
        std::swap(current, spare);
    }

    // Smoothed and randomized paths may leave the navmesh, validate them if requested.
    if ((smoothed || pathType == PathType::RANDOM) && current->pointCount > 0)
    {
        bool validated = false;

        if (HasFlag(flags, PathRequestFlags::VALIDATE_CPOP))
        {
            validated = Navigation->PostProcessClosestPointOnPoly(clientId, mapId, *current, *spare);
        }
        else if (HasFlag(flags, PathRequestFlags::VALIDATE_MAS))
        {
            validated = Navigation->PostProcessMoveAlongSurface(clientId, mapId, *current, *spare);
        }

        if (validated && spare->pointCount > 0)
        {
            std::swap(current, spare);
        }
    }

    return current;
}

void NavServer::HandlePath(ClientHandler* handler, AnTcpMessageType type, const void* data, int size,
                           PathType pathType)
{
    PathRequestData request{};

    if (!ReadRequest(data, size, request))
    {
        LogTooSmall(handler, type, size, sizeof(request));
        SendZero(handler, type);
        return;
    }

    const auto client = Navigation->GetClient(handler->GetId());

    if (!client)
    {
        SendZero(handler, type);
        return;
    }

    Path& path = client->GetPathBuffer();
    Path& scratch = client->GetScratchPathBuffer();

    PathTuningData tuning{};
    const bool hasTuning = static_cast<size_t>(size) >= sizeof(request) + sizeof(tuning);

    if (hasTuning)
    {
        std::memcpy(&tuning, static_cast<const char*>(data) + sizeof(request), sizeof(tuning));
    }

    PathDetailData detail{};
    const bool hasDetail = static_cast<size_t>(size) >= sizeof(request) + sizeof(tuning) + sizeof(detail);

    if (hasDetail)
    {
        std::memcpy(&detail, static_cast<const char*>(data) + sizeof(request) + sizeof(tuning), sizeof(detail));
    }

    PathOptions options =
        MakePathOptions(request.flags, hasTuning ? &tuning : nullptr, hasDetail ? &detail : nullptr);
    options.follow = options.follow && pathType == PathType::STRAIGHT;

    bool partial = false;
    bool ok = pathType == PathType::RANDOM
                  ? Navigation->GetRandomPath(handler->GetId(), request.mapId, request.start, request.end, path,
                                              Cfg.randomPathMaxDistance, &partial, &options)
                  : Navigation->GetPath(handler->GetId(), request.mapId, request.start, request.end, path, &partial,
                                        &options);

    if (ok && partial && HasFlag(request.flags, PathRequestFlags::REQUIRE_COMPLETE))
    {
        LogD("[", handler->GetId(), "] ", MessageName(type), ": end not reachable, partial path rejected");
        ok = false;
    }

    int pointCount = 0;

    if (ok && path.pointCount > 0)
    {
        const Path* result = ApplyPathFlags(handler->GetId(), request.mapId, request.flags, pathType, path, scratch);
        pointCount = result->pointCount;
        handler->SendData(type, result->points, static_cast<size_t>(result->pointCount) * sizeof(Vector3));
    }
    else
    {
        SendZero(handler, type);
    }

    LogD("[", handler->GetId(), "] ", MessageName(type), " map=", request.mapId, ok ? " ok" : " FAIL",
         " pts=", pointCount);
}

void NavServer::HandleMoveAlongSurface(ClientHandler* handler, AnTcpMessageType type, const void* data, int size)
{
    MoveRequestData request{};

    if (!ReadRequest(data, size, request))
    {
        LogTooSmall(handler, type, size, sizeof(request));
        SendZero(handler, type);
        return;
    }

    Vector3 point;
    const bool ok =
        Navigation->MoveAlongSurface(handler->GetId(), request.mapId, request.start, request.end, point);
    SendVector(handler, type, ok ? point : Vector3{});
    LogD("[", handler->GetId(), "] MoveAlongSurface map=", request.mapId, ok ? " ok" : " FAIL");
}

void NavServer::HandleCastRay(ClientHandler* handler, AnTcpMessageType type, const void* data, int size)
{
    CastRayData request{};

    if (!ReadRequest(data, size, request))
    {
        LogTooSmall(handler, type, size, sizeof(request));
        SendZero(handler, type);
        return;
    }

    // Legacy semantics: the end position if the way is clear, zero if blocked.
    const bool clear = Navigation->CastMovementRay(handler->GetId(), request.mapId, request.start, request.end);
    SendVector(handler, type, clear ? request.end : Vector3{});
    LogD("[", handler->GetId(), "] CastRay map=", request.mapId, clear ? " clear" : " blocked");
}

void NavServer::HandleCastRayEx(ClientHandler* handler, AnTcpMessageType type, const void* data, int size)
{
    CastRayData request{};
    CastRayExResponse response{-1, 0.0f, Vector3{}, Vector3{}};

    if (!ReadRequest(data, size, request))
    {
        LogTooSmall(handler, type, size, sizeof(request));
        handler->SendDataPtr(type, &response);
        return;
    }

    RaycastResult result;
    const bool clear =
        Navigation->CastMovementRay(handler->GetId(), request.mapId, request.start, request.end, &result);

    if (clear || result.hit)
    {
        response.hit = result.hit ? 1 : 0;
        response.t = result.t;
        response.position = result.hitPosition;
        response.normal = result.hitNormal;
    }

    handler->SendDataPtr(type, &response);
}

void NavServer::HandleRandomPoint(ClientHandler* handler, AnTcpMessageType type, const void* data, int size)
{
    int mapId = 0;

    if (!ReadRequest(data, size, mapId))
    {
        LogTooSmall(handler, type, size, sizeof(mapId));
        SendZero(handler, type);
        return;
    }

    Vector3 point;
    const bool ok = Navigation->GetRandomPoint(handler->GetId(), mapId, point);
    SendVector(handler, type, ok ? point : Vector3{});
}

void NavServer::HandleRandomPointAround(ClientHandler* handler, AnTcpMessageType type, const void* data, int size)
{
    RandomPointAroundData request{};

    if (!ReadRequest(data, size, request))
    {
        LogTooSmall(handler, type, size, sizeof(request));
        SendZero(handler, type);
        return;
    }

    Vector3 point;
    const bool ok =
        Navigation->GetRandomPointAround(handler->GetId(), request.mapId, request.start, request.radius, point);
    SendVector(handler, type, ok ? point : Vector3{});
}

void NavServer::HandleConfigureFilter(ClientHandler* handler, AnTcpMessageType type, const void* data, int size)
{
    const auto reply = [&](bool result) { handler->SendDataVar(type, result); };

    ConfigureFilterHeader header{};

    if (!ReadRequest(data, size, header))
    {
        LogTooSmall(handler, type, size, sizeof(header));
        reply(false);
        return;
    }

    const int count = header.filterConfigCount;
    const size_t expectedSize = sizeof(ConfigureFilterHeader) + static_cast<size_t>(std::max(count, 0)) * sizeof(FilterConfig);

    if (count < 0 || count > MAX_FILTER_CONFIGS || static_cast<size_t>(size) < expectedSize)
    {
        LogD("[", handler->GetId(), "] ConfigureFilter: invalid entry count ", count);
        reply(false);
        return;
    }

    const auto client = Navigation->GetClient(handler->GetId());

    if (!client)
    {
        reply(false);
        return;
    }

    AreaCost costs[MAX_FILTER_CONFIGS]{};
    const auto* entries = static_cast<const unsigned char*>(data) + sizeof(ConfigureFilterHeader);

    for (int i = 0; i < count; ++i)
    {
        FilterConfig entry{};
        std::memcpy(&entry, entries + static_cast<size_t>(i) * sizeof(FilterConfig), sizeof(FilterConfig));
        costs[i] = {entry.areaId, entry.cost};
    }

    const bool ok = client->ConfigureQueryFilter(header.state, std::span<const AreaCost>(costs, count));

    if (!ok)
    {
        LogD("[", handler->GetId(), "] ConfigureFilter: rejected (state=", static_cast<int>(header.state),
             ", entries=", count, ")");
    }

    LogD("[", handler->GetId(), "] ConfigureFilter state=", static_cast<int>(header.state), " entries=", count);
    reply(ok);
}

void NavServer::HandleGetHeight(ClientHandler* handler, AnTcpMessageType type, const void* data, int size)
{
    GetHeightData request{};

    if (!ReadRequest(data, size, request))
    {
        LogTooSmall(handler, type, size, sizeof(request));
        SendZero(handler, type);
        return;
    }

    Vector3 point;
    const bool ok = Navigation->GetHeight(handler->GetId(), request.mapId, request.position, point);
    SendVector(handler, type, ok ? point : Vector3{});
}

void NavServer::HandleGetConfig(ClientHandler* handler, AnTcpMessageType type, const void* /*data*/, int /*size*/)
{
    const auto& path = Cfg.mmapsPath;

    GetConfigResponseHeader header{};
    // Report the effective (detected) format using the config's numbering.
    switch (Navigation->GetMmapFormat())
    {
        case MmapFormat::TC335A: header.mmapFormat = 1; break;
        case MmapFormat::SF548: header.mmapFormat = 2; break;
        case MmapFormat::CUSTOM: header.mmapFormat = -1; break;
        case MmapFormat::TC_MASTER: header.mmapFormat = 3; break;
        default: header.mmapFormat = 0; break;
    }

    header.useAnpFileFormat = Cfg.useAnpFileFormat ? 1 : 0;
    header.pathLength = static_cast<int>(path.size());

    const std::string_view version = AMEISENNAV_VERSION;
    GetConfigResponseTrailer trailer{};
    trailer.protocolVersion = PROTOCOL_VERSION;
    trailer.maxPointPath = Cfg.maxPointPath;
    trailer.versionLength = static_cast<int>(version.size());

    std::vector<char> buffer;
    buffer.reserve(sizeof(header) + path.size() + sizeof(trailer) + version.size());

    const auto append = [&](const void* bytes, size_t count) {
        buffer.insert(buffer.end(), static_cast<const char*>(bytes), static_cast<const char*>(bytes) + count);
    };

    append(&header, sizeof(header));
    append(path.data(), path.size());
    append(&trailer, sizeof(trailer));
    append(version.data(), version.size());

    handler->SendData(type, buffer.data(), buffer.size());
}

void NavServer::HandleExplorePoly(ClientHandler* handler, AnTcpMessageType type, const void* data, int size)
{
    ExplorePolyRequestHeader request{};

    if (!ReadRequest(data, size, request))
    {
        LogTooSmall(handler, type, size, sizeof(request));
        SendZero(handler, type);
        return;
    }

    const int count = request.pointCount;

    if (count < 3 || count > MAX_EXPLORE_POLYGON_POINTS
        || static_cast<size_t>(size) < sizeof(request) + static_cast<size_t>(count) * sizeof(Vector3))
    {
        LogD("[", handler->GetId(), "] ExplorePoly: invalid polygon (", count, " points, ", size, " bytes)");
        SendZero(handler, type);
        return;
    }

    const auto client = Navigation->GetClient(handler->GetId());

    if (!client)
    {
        SendZero(handler, type);
        return;
    }

    std::vector<Vector3> polygon(static_cast<size_t>(count));
    std::memcpy(polygon.data(), static_cast<const char*>(data) + sizeof(request), polygon.size() * sizeof(Vector3));

    const size_t tuningOffset = sizeof(request) + polygon.size() * sizeof(Vector3);
    PathTuningData tuning{};
    const bool hasTuning = static_cast<size_t>(size) >= tuningOffset + sizeof(tuning);

    if (hasTuning)
    {
        std::memcpy(&tuning, static_cast<const char*>(data) + tuningOffset, sizeof(tuning));
    }

    const PathOptions options = MakePathOptions(request.flags, hasTuning ? &tuning : nullptr);

    Path& path = client->GetPathBuffer();
    Path& scratch = client->GetScratchPathBuffer();
    ExploreResult result;

    const bool ok = Navigation->ExplorePolygon(handler->GetId(), request.mapId, request.start, polygon,
                                               request.spacing, path, &result, &options);

    if (ok)
    {
        const Path* route =
            ApplyPathFlags(handler->GetId(), request.mapId, request.flags, PathType::STRAIGHT, path, scratch);
        handler->SendData(type, route->points, static_cast<size_t>(route->pointCount) * sizeof(Vector3));
    }
    else
    {
        SendZero(handler, type);
    }

    LogD("[", handler->GetId(), "] ExplorePoly map=", request.mapId, ok ? " ok" : " FAIL", " waypoints=",
         result.reached, "/", result.waypoints, " pts=", path.pointCount);
}

void NavServer::HandleSetAvoidZones(ClientHandler* handler, AnTcpMessageType type, const void* data, int size)
{
    const auto reply = [&](bool result) { handler->SendDataVar(type, result); };
    AvoidZonesHeader header{};

    if (!ReadRequest(data, size, header))
    {
        LogTooSmall(handler, type, size, sizeof(header));
        reply(false);
        return;
    }

    if (header.zoneCount < 0 || header.zoneCount > MAX_AVOID_ZONES
        || static_cast<size_t>(size) < sizeof(header) + static_cast<size_t>(header.zoneCount) * sizeof(AvoidZoneData))
    {
        LogD("[", handler->GetId(), "] SetAvoidZones: invalid zone count ", header.zoneCount);
        reply(false);
        return;
    }

    std::vector<AvoidZone> zones(static_cast<size_t>(header.zoneCount));
    const auto* entries = static_cast<const char*>(data) + sizeof(header);

    for (size_t i = 0; i < zones.size(); ++i)
    {
        AvoidZoneData entry{};
        std::memcpy(&entry, entries + i * sizeof(AvoidZoneData), sizeof(entry));
        zones[i] = AvoidZone{entry.center, entry.radius, entry.cost};
    }

    const bool ok = Navigation->SetAvoidZones(handler->GetId(), header.mapId, zones);
    LogD("[", handler->GetId(), "] SetAvoidZones map=", header.mapId, " zones=", zones.size(), ok ? " ok" : " FAIL");
    reply(ok);
}

void NavServer::HandleSteer(ClientHandler* handler, AnTcpMessageType type, const void* data, int size)
{
    SteerRequestHeader header{};
    SteerResponse response{0, Vector3{}, Vector3{}};

    if (!ReadRequest(data, size, header))
    {
        LogTooSmall(handler, type, size, sizeof(header));
        handler->SendDataPtr(type, &response);
        return;
    }

    if (header.unitCount < 0 || header.unitCount > MAX_STEER_UNITS
        || static_cast<size_t>(size) < sizeof(header) + static_cast<size_t>(header.unitCount) * sizeof(SteerUnitData))
    {
        LogD("[", handler->GetId(), "] Steer: invalid unit count ", header.unitCount);
        handler->SendDataPtr(type, &response);
        return;
    }

    std::vector<SteerUnit> units(static_cast<size_t>(header.unitCount));
    const auto* entries = static_cast<const char*>(data) + sizeof(header);

    for (size_t i = 0; i < units.size(); ++i)
    {
        SteerUnitData entry{};
        std::memcpy(&entry, entries + i * sizeof(SteerUnitData), sizeof(entry));
        units[i] = SteerUnit{entry.position, entry.velocity, entry.radius};
    }

    SteerResult result;

    if (Navigation->Steer(handler->GetId(), header.mapId, header.position, header.velocity, header.target,
                          header.maxSpeed, header.radius, header.deltaTime, units, result))
    {
        response = SteerResponse{1, result.velocity, result.position};
    }

    handler->SendDataPtr(type, &response);
}

void NavServer::HandleReportStuck(ClientHandler* handler, AnTcpMessageType type, const void* data, int size)
{
    ReportStuckData request{};

    if (!ReadRequest(data, size, request))
    {
        LogTooSmall(handler, type, size, sizeof(request));
        handler->SendDataVar(type, false);
        return;
    }

    handler->SendDataVar(type, Navigation->ReportStuck(request.mapId, request.position, request.radius));
}

void NavServer::HandlePathEx(ClientHandler* handler, AnTcpMessageType type, const void* data, int size)
{
    PathRequestData request{};
    const PathExResponseHeader failed{0, 0};

    if (!ReadRequest(data, size, request))
    {
        LogTooSmall(handler, type, size, sizeof(request));
        handler->SendDataPtr(type, &failed);
        return;
    }

    const auto client = Navigation->GetClient(handler->GetId());

    if (!client)
    {
        handler->SendDataPtr(type, &failed);
        return;
    }

    PathTuningData tuning{};
    const bool hasTuning = static_cast<size_t>(size) >= sizeof(request) + sizeof(tuning);

    if (hasTuning)
    {
        std::memcpy(&tuning, static_cast<const char*>(data) + sizeof(request), sizeof(tuning));
    }

    PathDetailData detail{};
    const bool hasDetail = static_cast<size_t>(size) >= sizeof(request) + sizeof(tuning) + sizeof(detail);

    if (hasDetail)
    {
        std::memcpy(&detail, static_cast<const char*>(data) + sizeof(request) + sizeof(tuning), sizeof(detail));
    }

    const bool randomize = HasFlag(request.flags, PathRequestFlags::RANDOMIZE);
    PathOptions options =
        MakePathOptions(request.flags, hasTuning ? &tuning : nullptr, hasDetail ? &detail : nullptr);
    options.follow = options.follow && !randomize;

    Path& path = client->GetPathBuffer();
    Path& scratch = client->GetScratchPathBuffer();
    std::vector<unsigned char> flags(static_cast<size_t>(path.maxSize), 0);
    const bool surfaceInfo = HasFlag(request.flags, PathRequestFlags::SURFACE_INFO);
    std::vector<unsigned char> info(surfaceInfo ? static_cast<size_t>(path.maxSize) : 0, 0);
    std::vector<unsigned char> areas(info.size(), 0);
    unsigned char* infoPtr = surfaceInfo ? info.data() : nullptr;
    unsigned char* areasPtr = surfaceInfo ? areas.data() : nullptr;
    bool partial = false;
    PathPartialReason reason = PathPartialReason::NONE;
    const bool wantLinkIds = HasFlag(request.flags, PathRequestFlags::LINK_IDS);
    std::vector<unsigned int> linkIds;
    std::vector<unsigned int>* linkIdsPtr = wantLinkIds ? &linkIds : nullptr;

    bool ok = randomize ? Navigation->GetRandomPath(handler->GetId(), request.mapId, request.start, request.end, path,
                                                    Cfg.randomPathMaxDistance, &partial, &options, flags.data(),
                                                    infoPtr, areasPtr, &reason, linkIdsPtr)
                        : Navigation->GetPath(handler->GetId(), request.mapId, request.start, request.end, path,
                                              &partial, &options, flags.data(), infoPtr, areasPtr, &reason, linkIdsPtr);

    if (ok && partial && HasFlag(request.flags, PathRequestFlags::REQUIRE_COMPLETE))
    {
        ok = false;
    }

    if (!ok || path.pointCount <= 0)
    {
        handler->SendDataPtr(type, &failed);
        return;
    }

    // Legacy smoothing moves every point: only for paths without off-mesh connections (their points must stay).
    const Path* result = &path;
    const bool offMesh = std::any_of(flags.begin(), flags.begin() + path.pointCount,
                                     [](unsigned char f) { return (f & PATH_POINT_OFFMESH) != 0; });

    if (!offMesh)
    {
        result = ApplyPathFlags(handler->GetId(), request.mapId, request.flags,
                                randomize ? PathType::RANDOM : PathType::STRAIGHT, path, scratch);

        if (result != &path)
        {
            std::fill(flags.begin(), flags.end(), 0);
            flags[0] = PATH_POINT_START;
            flags[static_cast<size_t>(result->pointCount - 1)] |= partial ? 0 : PATH_POINT_END;
        }
    }

    // Legacy smoothing moved the points: their areas and info are unknown.
    const bool withInfo = surfaceInfo && result == &path;
    const auto count = static_cast<size_t>(result->pointCount);
    const bool withLinks = wantLinkIds && result == &path;
    int resultFlags = (partial ? PATH_RESULT_PARTIAL : 0) | (withInfo ? PATH_RESULT_SURFACE_INFO : 0) |
                      (withLinks ? PATH_RESULT_LINK_IDS : 0);

    if (partial && reason == PathPartialReason::UNREACHABLE)
    {
        resultFlags |= PATH_RESULT_UNREACHABLE;
    }
    else if (partial && reason == PathPartialReason::SEARCH_LIMIT)
    {
        resultFlags |= PATH_RESULT_SEARCH_LIMIT;
    }

    const PathExResponseHeader header{result->pointCount, resultFlags};
    std::vector<char> buffer(sizeof(header) + count * (sizeof(Vector3) + (withInfo ? 3 : 1) + (withLinks ? 4 : 0)));
    char* out = buffer.data();
    std::memcpy(out, &header, sizeof(header));
    out += sizeof(header);
    std::memcpy(out, result->points, count * sizeof(Vector3));
    out += count * sizeof(Vector3);
    std::memcpy(out, flags.data(), count);

    if (withInfo)
    {
        std::memcpy(out + count, areas.data(), count);
        std::memcpy(out + 2 * count, info.data(), count);
    }

    if (withLinks)
    {
        // One id per point: the k-th point flagged OFFMESH gets the k-th id (refinement keeps them all, in order).
        std::vector<unsigned int> perPoint(count, 0u);
        size_t next = 0;

        for (size_t i = 0; i < count; ++i)
        {
            if ((flags[i] & PATH_POINT_OFFMESH) && next < linkIds.size())
            {
                perPoint[i] = linkIds[next++];
            }
        }

        std::memcpy(out + (withInfo ? 3 : 1) * count, perPoint.data(), count * sizeof(unsigned int));
    }

    handler->SendData(type, buffer.data(), buffer.size());
}

namespace {
WorldHitResponse ToResponse(bool known, const World::Hit& hit)
{
    WorldHitResponse response{-1, 0.0f, Vector3{}, Vector3{}, 0u, 0};

    if (known)
    {
        response.result = hit.hit ? 1 : 0;
        response.t = hit.t;
        response.position = hit.position;
        response.normal = hit.normal;
        response.source = hit.source;
        response.area = hit.area;
    }

    return response;
}
} // namespace

void NavServer::HandleLineOfSight(ClientHandler* handler, AnTcpMessageType type, const void* data, int size)
{
    LineOfSightData request{};
    World::Hit hit;
    bool known = false;

    if (ReadRequest(data, size, request))
    {
        known = Navigation->LineOfSight(request.mapId, request.start, request.end,
                                        static_cast<uint32_t>(request.flags), hit);
    }
    else
    {
        LogTooSmall(handler, type, size, sizeof(request));
    }

    const WorldHitResponse response = ToResponse(known, hit);
    handler->SendDataPtr(type, &response);
}

void NavServer::HandleSweep(ClientHandler* handler, AnTcpMessageType type, const void* data, int size)
{
    SweepData request{};
    World::Hit hit;
    bool known = false;

    if (ReadRequest(data, size, request))
    {
        known = Navigation->SweepBody(request.mapId, request.start, request.end, request.radius, request.height,
                                      request.stepHeight, static_cast<uint32_t>(request.flags), hit);
    }
    else
    {
        LogTooSmall(handler, type, size, sizeof(request));
    }

    const WorldHitResponse response = ToResponse(known, hit);
    handler->SendDataPtr(type, &response);
}

void NavServer::HandleSurfaces(ClientHandler* handler, AnTcpMessageType type, const void* data, int size)
{
    SurfacesRequestData request{};
    std::vector<World::Surface> surfaces;
    uint32_t areaId = 0;
    SurfacesResponseHeader header{-1, 0u};

    if (!ReadRequest(data, size, request))
    {
        LogTooSmall(handler, type, size, sizeof(request));
    }
    else if (Navigation->GetSurfaces(request.mapId, request.x, request.y, request.zTop, request.zBottom,
                                     static_cast<uint32_t>(request.flags), surfaces, areaId))
    {
        header.count = static_cast<int>(std::min<size_t>(surfaces.size(), MAX_SURFACES));
        header.areaId = areaId;
    }

    std::vector<unsigned char> response(sizeof(header) + std::max(header.count, 0) * sizeof(SurfaceData));
    std::memcpy(response.data(), &header, sizeof(header));

    for (int i = 0; i < header.count; ++i)
    {
        const World::Surface& surface = surfaces[static_cast<size_t>(i)];
        SurfaceData out{};
        out.z = surface.z;
        out.normal = surface.normal;
        out.source = surface.source;
        out.area = surface.area;
        out.flags = static_cast<unsigned char>((surface.standable ? SURFACE_STANDABLE : 0)
                                               | (surface.liquid ? SURFACE_LIQUID : 0));
        std::memcpy(response.data() + sizeof(header) + i * sizeof(SurfaceData), &out, sizeof(out));
    }

    handler->SendData(type, response.data(), response.size());
}

void NavServer::HandleSetObjects(ClientHandler* handler, AnTcpMessageType type, const void* data, int size)
{
    SetObjectsHeader header{};
    SetObjectsResponse response{-1, 0};

    const bool valid = ReadRequest(data, size, header) && header.count >= 0 && header.removeCount >= 0
                       && header.count + header.removeCount <= MAX_OBJECTS_PER_REQUEST
                       && static_cast<size_t>(size) == sizeof(header) + header.count * sizeof(ObjectData)
                                                          + header.removeCount * sizeof(unsigned long long);

    if (!valid)
    {
        LogD("[", handler->GetId(), "] Invalid SET_OBJECTS request (", size, " bytes)");
        handler->SendDataPtr(type, &response);
        return;
    }

    const auto* bytes = static_cast<const unsigned char*>(data) + sizeof(header);
    std::vector<WorldObject> objects(static_cast<size_t>(header.count));
    std::vector<uint64_t> removed(static_cast<size_t>(header.removeCount));

    for (auto& object : objects)
    {
        ObjectData in{};
        std::memcpy(&in, bytes, sizeof(in));
        bytes += sizeof(in);
        object.guid = in.guid;
        object.displayId = in.displayId;
        object.blocking = (in.flags & OBJECT_BLOCKING) != 0;
        object.position = in.position;
        std::copy(std::begin(in.rotation), std::end(in.rotation), object.rotation);
        object.scale = in.scale;
    }

    if (!removed.empty())
    {
        std::memcpy(removed.data(), bytes, removed.size() * sizeof(uint64_t));
    }

    response.accepted = Dynamic->SetObjects(header.mapId, objects, removed);
    response.pendingTiles = static_cast<int>(Dynamic->PendingTiles());
    handler->SendDataPtr(type, &response);
}
