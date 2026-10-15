using AnTCP.Client;
using System;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace AmeisenNavigation.Client
{
    /// <summary>
    /// Client for the AmeisenNavigation pathfinding server.
    ///
    /// <para>Usage:</para>
    /// <code>
    /// using var nav = new AmeisenNavClient("127.0.0.1", 47110);
    /// nav.Connected += () => Console.WriteLine("Connected!");
    /// nav.Disconnected += () => Console.WriteLine("Lost connection");
    /// nav.TryConnect();
    /// nav.SetClientState(ClientState.NormalAlliance);
    /// nav.SetAreaCosts(ground: 1f, road: 0.75f, water: 1.6f, badLiquid: 4f, hordeMult: 3f);
    /// nav.SetCanSwim(false);
    /// nav.ApplyFilter();
    ///
    /// var path = nav.GetPath(0, start, end, PathFlags.Fluid, new PathTuning(StopDistance: 4f));
    /// var detailed = nav.GetPathEx(0, start, end); // where to jump or drop
    /// </code>
    /// </summary>
    public sealed class AmeisenNavClient : IDisposable
    {
        private enum MessageType : byte
        {
            Path,
            MoveAlongSurface,
            RandomPoint,
            RandomPointAround,
            CastRay,
            RandomPath,
            ExplorePoly,
            ConfigureFilter,
            GetHeight,
            GetConfig,
            CastRayEx,
            SetAvoidZones,
            Steer,
            ReportStuck,
            PathEx,
            LineOfSight,
            Surfaces,
            Sweep,
            SetObjects,
        }

        private readonly AnTcpClient _client;
        private readonly object _lock = new();

        private ClientState _state;
        private readonly float[] _areaCosts = new float[28]; // index 0 unused, 1-27 = area costs
        private float _allyMult = 1f;
        private float _hordeMult = 1f;
        private readonly Dictionary<TerrainType, float> _terrainCosts = new();
        private readonly Dictionary<int, AvoidZone[]> _avoidZones = new();
        private bool _wasConnected;
        private ServerConfig? _serverConfig;
        private bool _disposed;

        // ── Events ────────────────────────────────────────────────────────

        /// <summary>Raised when a connection (or reconnection) to the server succeeds.</summary>
        public event Action? Connected;

        /// <summary>Raised when the server connection is lost.</summary>
        public event Action? Disconnected;

        /// <summary>Raised before each reconnect attempt. The int parameter is the attempt number (1-based).</summary>
        public event Action<int>? Reconnecting;

        // ── Configuration ─────────────────────────────────────────────────

        /// <summary>
        /// When true, the client will automatically attempt to reconnect when a
        /// network error is detected during a pathfinding call. Default: true.
        /// </summary>
        public bool AutoReconnect { get; set; } = true;

        /// <summary>
        /// Milliseconds to wait between reconnect attempts. Default: 1000.
        /// </summary>
        public int ReconnectDelayMs { get; set; } = 1000;

        /// <summary>
        /// Maximum number of reconnect attempts before giving up.
        /// Set to 0 for infinite retries. Default: 5.
        /// </summary>
        public int MaxReconnectAttempts { get; set; } = 5;

        /// <summary>
        /// True when area costs or client state have changed since the last <see cref="ApplyFilter"/> call.
        /// </summary>
        public bool IsFilterDirty { get; private set; }

        public bool IsConnected => _client.IsConnected;

        /// <summary>
        /// Send/receive timeout in milliseconds (0 = infinite). A timeout counts as a network error
        /// and triggers the auto-reconnect logic. Default: 30 seconds.
        /// </summary>
        public int TimeoutMs
        {
            get => _client.TimeoutMs;
            set => _client.TimeoutMs = value;
        }

        public AmeisenNavClient(string ip = "127.0.0.1", int port = 47110)
        {
            _client = new AnTcpClient(ip, port);

            // Initialize with server defaults
            _state = ClientState.Normal;
            SetAreaCosts(ground: 1f, road: 0.75f, water: 1.6f, badLiquid: 4f);
        }

        /// <summary>
        /// Connect to the server. Returns true on success.
        /// </summary>
        public bool TryConnect()
        {
            if (_client.IsConnected) return true;

            try
            {
                _client.Connect();

                if (_client.IsConnected)
                {
                    _wasConnected = true;
                    Connected?.Invoke();
                    return true;
                }

                return false;
            }
            catch
            {
                return false;
            }
        }

        // ── Pathfinding ─────────────────────────────────────────────────

        /// <summary>
        /// Find a straight path from start to end. Returns null on failure.
        /// <paramref name="tuning"/> (protocol 3) adjusts the refinement flags and the stop distance.
        /// </summary>
        public Vector3[]? GetPath(int mapId, Vector3 start, Vector3 end, PathFlags flags = PathFlags.None,
                                  PathTuning? tuning = null)
        {
            return SendPathRequest(MessageType.Path, mapId, start, end, flags, tuning);
        }

        /// <summary>
        /// Find a path with randomized waypoints (human-like movement). Returns null on failure.
        /// </summary>
        public Vector3[]? GetRandomPath(int mapId, Vector3 start, Vector3 end, PathFlags flags = PathFlags.None,
                                        PathTuning? tuning = null)
        {
            return SendPathRequest(MessageType.RandomPath, mapId, start, end, flags, tuning);
        }

        /// <summary>
        /// Find a path and get what each point is: where to jump or drop (off-mesh connections) and whether the end
        /// is reachable. <see cref="PathFlags.Randomize"/> adds random offsets, <see cref="PathFlags.SurfaceInfo"/>
        /// the ground under every point. Returns null on failure.
        /// Requires protocol version 3 (server 1.10+).
        /// </summary>
        public PathResult? GetPathEx(int mapId, Vector3 start, Vector3 end, PathFlags flags = PathFlags.None,
                                     PathTuning? tuning = null)
        {
            lock (_lock)
            {
                return SendWithReconnect<PathResult?>(() =>
                {
                    var response = Send(MessageType.PathEx, MakePathRequest(mapId, start, end, flags, tuning));
                    return ParsePathEx(response.Data);
                }, null);
            }
        }

        /// <summary>
        /// Replace this connection's avoid zones on a map (empty list = none). The search prefers other ways and
        /// paths are led around them. The zones are sent again after a reconnect. Up to 64 zones per map.
        /// Requires protocol version 3.
        /// </summary>
        public bool SetAvoidZones(int mapId, IReadOnlyList<AvoidZone> zones)
        {
            if (zones == null || zones.Count > MaxAvoidZones)
                return false;

            var copy = new AvoidZone[zones.Count];

            for (int i = 0; i < copy.Length; i++)
                copy[i] = zones[i];

            lock (_lock)
            {
                if (copy.Length == 0)
                    _avoidZones.Remove(mapId);
                else
                    _avoidZones[mapId] = copy;

                return SendWithReconnect(() => SendAvoidZonesInternal(mapId, copy), false);
            }
        }

        /// <summary>Remove this connection's avoid zones on a map.</summary>
        public bool ClearAvoidZones(int mapId) => SetAvoidZones(mapId, Array.Empty<AvoidZone>());

        /// <summary>Maximum number of avoid zones per map.</summary>
        public const int MaxAvoidZones = 64;

        /// <summary>Maximum number of units passed to <see cref="Steer"/>.</summary>
        public const int MaxSteerUnits = 32;

        /// <summary>
        /// Local steering for one tick: the velocity towards <paramref name="target"/> (along the navmesh path) that
        /// avoids the <paramref name="units"/> (up to 32), the avoid zones and the navmesh borders, and the position
        /// it leads to after <paramref name="deltaTime"/> seconds. Requires protocol version 3.
        /// </summary>
        public SteerResult Steer(int mapId, Vector3 position, Vector3 velocity, Vector3 target, float maxSpeed,
                                 float radius = 0.5f, float deltaTime = 0.1f, IReadOnlyList<SteerUnit>? units = null)
        {
            int unitCount = units?.Count ?? 0;

            if (unitCount > MaxSteerUnits)
                return default;

            var header = new SteerRequestHeader
            {
                MapId = mapId,
                Position = position,
                Velocity = velocity,
                Target = target,
                MaxSpeed = maxSpeed,
                Radius = radius,
                DeltaTime = deltaTime,
                UnitCount = unitCount,
            };

            int headerSize = Marshal.SizeOf<SteerRequestHeader>();
            int unitSize = Marshal.SizeOf<SteerUnitData>();
            byte[] buffer = new byte[headerSize + unitCount * unitSize];
            MemoryMarshal.Write(buffer.AsSpan(0, headerSize), in header);

            for (int i = 0; i < unitCount; i++)
            {
                var unit = new SteerUnitData { Position = units![i].Position, Velocity = units[i].Velocity, Radius = units[i].Radius };
                MemoryMarshal.Write(buffer.AsSpan(headerSize + i * unitSize, unitSize), in unit);
            }

            lock (_lock)
            {
                return SendWithReconnect(() =>
                {
                    var response = _client.SendBytes((byte)MessageType.Steer, buffer);

                    if (!response.TryAs(out SteerResponse result) || result.Ok == 0)
                        return default;

                    return new SteerResult(true, result.Velocity, result.Position);
                }, default(SteerResult));
            }
        }

        /// <summary>
        /// Report a position where the bot got stuck: the server adds a zone every client avoids from now on (and
        /// keeps it if it has an avoid zone file). <paramref name="radius"/> 0 uses the server's default.
        /// Requires protocol version 3.
        /// </summary>
        public bool ReportStuck(int mapId, Vector3 position, float radius = 0f)
        {
            lock (_lock)
            {
                var request = new ReportStuckData { MapId = mapId, Position = position, Radius = radius };
                return SendWithReconnect(() => _client.Send((byte)MessageType.ReportStuck, request).As<bool>(), false);
            }
        }

        /// <summary>
        /// Line of sight against the collision geometry the navmesh was built from (static world), with what blocks
        /// it. Pass eye positions (feet + about 1.7 yards). Liquids never block. Requires protocol version 4 and a
        /// .anw file for the map (exporter 1.12+).
        /// </summary>
        public WorldHit LineOfSight(int mapId, Vector3 start, Vector3 end, WorldIgnore ignore = WorldIgnore.None)
        {
            var request = new LineOfSightData { MapId = mapId, Flags = (int)ignore, Start = start, End = end };

            lock (_lock)
            {
                return SendWithReconnect(() => ToWorldHit(_client.Send((byte)MessageType.LineOfSight, request)), default);
            }
        }

        /// <summary>
        /// Move a standing body (radius, height, feet at the positions) from start to end: the first geometry it runs
        /// into above <paramref name="stepHeight"/> (steps and slopes below it are walked over). Catches what the
        /// navmesh rounds away (thin posts, branches at head height). Requires protocol version 4 and a .anw file.
        /// </summary>
        public WorldHit Sweep(int mapId, Vector3 start, Vector3 end, float radius = 0.5f, float height = 2f,
                              float stepHeight = 0.5f, WorldIgnore ignore = WorldIgnore.None)
        {
            var request = new SweepData
            {
                MapId = mapId,
                Flags = (int)ignore,
                Start = start,
                End = end,
                Radius = radius,
                Height = height,
                StepHeight = stepHeight,
            };

            lock (_lock)
            {
                return SendWithReconnect(() => ToWorldHit(_client.Send((byte)MessageType.Sweep, request)), default);
            }
        }

        /// <summary>
        /// Every surface under (x, y) between zTop and zBottom (bridges, decks, floors of a building, the ground,
        /// water) with where it comes from, highest first, and the AreaTable id there. Null if the server has no
        /// static world there. Requires protocol version 4 and a .anw file.
        /// </summary>
        public WorldSurfaces? GetSurfaces(int mapId, float x, float y, float zTop, float zBottom,
                                          WorldIgnore ignore = WorldIgnore.None)
        {
            var request = new SurfacesRequestData
            {
                MapId = mapId,
                Flags = (int)ignore,
                X = x,
                Y = y,
                ZTop = zTop,
                ZBottom = zBottom,
            };

            lock (_lock)
            {
                return SendWithReconnect<WorldSurfaces?>(() =>
                {
                    var response = _client.Send((byte)MessageType.Surfaces, request);
                    return ParseSurfaces(response.Data);
                }, null);
            }
        }

        /// <summary>Most objects (set + removed) per <see cref="SetObjects"/> call.</summary>
        public const int MaxObjectsPerRequest = 512;

        /// <summary>
        /// Tell the server about game objects (doors, gates, elevators' platforms, chests...) and remove others by
        /// guid. Their collision is in line of sight, surface and sweep queries at once; the navmesh tiles around them
        /// are rebuilt in the background (<see cref="SetObjectsResult.PendingTiles"/>). Objects are shared by every
        /// client of the server (they are world state). Requires protocol version 4, a .anw file for the map and
        /// gameobjects.ano (exporter 1.12+).
        /// </summary>
        public SetObjectsResult SetObjects(int mapId, IReadOnlyList<WorldObject> objects,
                                           IReadOnlyList<ulong>? removed = null)
        {
            int count = objects?.Count ?? 0;
            int removeCount = removed?.Count ?? 0;

            if (count + removeCount > MaxObjectsPerRequest)
                return new SetObjectsResult(-1, 0);

            int headerSize = Marshal.SizeOf<SetObjectsHeader>();
            int objectSize = Marshal.SizeOf<ObjectData>();
            byte[] buffer = new byte[headerSize + count * objectSize + removeCount * 8];
            var header = new SetObjectsHeader { MapId = mapId, Count = count, RemoveCount = removeCount };
            MemoryMarshal.Write(buffer.AsSpan(0, headerSize), in header);

            for (int i = 0; i < count; i++)
            {
                var o = objects![i];
                var data = new ObjectData
                {
                    Guid = o.Guid,
                    DisplayId = o.DisplayId,
                    Flags = o.Blocking ? 1u : 0u,
                    Position = o.Position,
                    RotationX = o.Rotation.X,
                    RotationY = o.Rotation.Y,
                    RotationZ = o.Rotation.Z,
                    RotationW = o.Rotation.W,
                    Scale = o.Scale,
                };
                MemoryMarshal.Write(buffer.AsSpan(headerSize + i * objectSize, objectSize), in data);
            }

            for (int i = 0; i < removeCount; i++)
                BitConverter.TryWriteBytes(buffer.AsSpan(headerSize + count * objectSize + i * 8, 8), removed![i]);

            lock (_lock)
            {
                return SendWithReconnect(() =>
                {
                    var response = _client.SendBytes((byte)MessageType.SetObjects, buffer);
                    return response.TryAs(out SetObjectsResponse result)
                        ? new SetObjectsResult(result.Accepted, result.PendingTiles)
                        : new SetObjectsResult(-1, 0);
                }, new SetObjectsResult(-1, 0));
            }
        }

        private static WorldHit ToWorldHit(AnTCP.Client.Objects.AnTcpResponse response)
        {
            if (!response.TryAs(out WorldHitResponse hit) || hit.Result < 0)
                return default;

            return new WorldHit(true, hit.Result == 1, hit.T, hit.Position, hit.Normal, new WorldSource(hit.Source),
                                hit.Area);
        }

        private static WorldSurfaces? ParseSurfaces(ReadOnlySpan<byte> data)
        {
            int headerSize = Marshal.SizeOf<SurfacesResponseHeader>();
            int surfaceSize = Marshal.SizeOf<SurfaceData>();

            if (data.Length < headerSize)
                return null;

            var header = MemoryMarshal.Read<SurfacesResponseHeader>(data);

            if (header.Count < 0 || data.Length != headerSize + (long)header.Count * surfaceSize)
                return null;

            var surfaces = new WorldSurface[header.Count];

            for (int i = 0; i < header.Count; i++)
            {
                var s = MemoryMarshal.Read<SurfaceData>(data.Slice(headerSize + i * surfaceSize, surfaceSize));
                surfaces[i] = new WorldSurface(s.Z, s.Normal, new WorldSource(s.Source), s.Area, (s.Flags & 1) != 0,
                                               (s.Flags & 2) != 0);
            }

            return new WorldSurfaces(surfaces, header.AreaId);
        }

        /// <summary>Maximum number of polygon vertices accepted by <see cref="ExplorePolygon"/>.</summary>
        public const int MaxExplorePolygonPoints = 256;

        /// <summary>
        /// Generate a route that explores an area: waypoints on a hexagonal grid <paramref name="spacing"/>
        /// apart inside the polygon (x/y outline, its z picks the floor on stacked geometry), visited in a short
        /// tour from <paramref name="start"/> and connected by navmesh paths. Unreachable waypoints are skipped,
        /// the route is cut at the server's maximum path length. Returns null on failure.
        /// Requires protocol version 2 (server 1.9+).
        /// </summary>
        /// <param name="spacing">Distance between waypoints, e.g. twice the sight/gather range (min 2).</param>
        public Vector3[]? ExplorePolygon(int mapId, Vector3 start, IReadOnlyList<Vector3> polygon, float spacing,
                                         PathFlags flags = PathFlags.None, PathTuning? tuning = null)
        {
            if (polygon == null || polygon.Count < 3 || polygon.Count > MaxExplorePolygonPoints
                || !float.IsFinite(spacing))
                return null;

            var header = new ExplorePolyRequestHeader
            {
                MapId = mapId,
                Flags = (int)flags,
                Start = start,
                Spacing = spacing,
                PointCount = polygon.Count,
            };

            int headerSize = Marshal.SizeOf<ExplorePolyRequestHeader>();
            int pointSize = Marshal.SizeOf<Vector3>();
            int tuningSize = tuning.HasValue ? Marshal.SizeOf<PathTuningData>() : 0;
            byte[] buffer = new byte[headerSize + polygon.Count * pointSize + tuningSize];

            MemoryMarshal.Write(buffer.AsSpan(0, headerSize), in header);

            for (int i = 0; i < polygon.Count; i++)
            {
                Vector3 point = polygon[i];
                MemoryMarshal.Write(buffer.AsSpan(headerSize + i * pointSize, pointSize), in point);
            }

            if (tuning.HasValue)
            {
                PathTuningData trailer = ToWire(tuning.Value);
                MemoryMarshal.Write(buffer.AsSpan(headerSize + polygon.Count * pointSize, tuningSize), in trailer);
            }

            lock (_lock)
            {
                return SendWithReconnect(() =>
                {
                    var points = _client.SendBytes((byte)MessageType.ExplorePoly, buffer).AsArray<Vector3>();
                    return points.Length == 0 || (points.Length == 1 && points[0].IsZero) ? null : points;
                }, null);
            }
        }

        /// <summary>
        /// Move along the navmesh surface by a small delta. Useful for preventing
        /// falling off edges during incremental movement.
        /// Returns the clamped target position, or a zero vector on failure.
        /// </summary>
        public Vector3 MoveAlongSurface(int mapId, Vector3 start, Vector3 end)
        {
            lock (_lock)
            {
                var request = new MoveRequestData { MapId = mapId, Start = start, End = end };
                return SendWithReconnect(
                    () => _client.Send((byte)MessageType.MoveAlongSurface, request).As<Vector3>(),
                    default
                );
            }
        }

        /// <summary>
        /// Cast a movement ray to test for obstacles between start and end.
        /// Returns true if the ray is clear (no wall hit), hitPoint is then the end position.
        /// If the ray is blocked (or the request failed) hitPoint is a zero vector,
        /// use <see cref="CastRayEx"/> to get the actual hit position.
        /// </summary>
        public bool CastRay(int mapId, Vector3 start, Vector3 end, out Vector3 hitPoint)
        {
            lock (_lock)
            {
                var request = new CastRayData { MapId = mapId, Start = start, End = end };
                var result = SendWithReconnect(
                    () => _client.Send((byte)MessageType.CastRay, request).As<Vector3>(),
                    default
                );

                hitPoint = result;
                return !result.IsZero;
            }
        }

        /// <summary>
        /// Cast a movement ray along the navmesh surface and return where it hit a wall,
        /// the wall normal and how far it got. Requires server version 1.9+.
        /// </summary>
        public RaycastHit CastRayEx(int mapId, Vector3 start, Vector3 end)
        {
            lock (_lock)
            {
                var request = new CastRayData { MapId = mapId, Start = start, End = end };
                return SendWithReconnect(() =>
                {
                    var response = _client.Send((byte)MessageType.CastRayEx, request);

                    if (!response.TryAs(out CastRayExResponse result) || result.Hit < 0)
                        return default;

                    return new RaycastHit(true, result.Hit != 0, result.Fraction, result.Position, result.Normal);
                }, default(RaycastHit));
            }
        }

        /// <summary>
        /// Get a random point anywhere on the navmesh for the given map.
        /// </summary>
        public Vector3 GetRandomPoint(int mapId)
        {
            lock (_lock)
            {
                return SendWithReconnect(
                    () => _client.Send((byte)MessageType.RandomPoint, mapId).As<Vector3>(),
                    default
                );
            }
        }

        /// <summary>
        /// Get a random point on the navmesh within a radius of the given center.
        /// </summary>
        public Vector3 GetRandomPointAround(int mapId, Vector3 center, float radius)
        {
            lock (_lock)
            {
                var request = new RandomPointAroundData { MapId = mapId, Start = center, Radius = radius };
                return SendWithReconnect(
                    () => _client.Send((byte)MessageType.RandomPointAround, request).As<Vector3>(),
                    default
                );
            }
        }

        /// <summary>
        /// Get the navmesh terrain height at a position. Returns the position
        /// snapped to the navmesh surface, or a zero vector if the position is off-mesh.
        /// </summary>
        public Vector3 GetHeight(int mapId, Vector3 position)
        {
            lock (_lock)
            {
                var request = new GetHeightData { MapId = mapId, Position = position };
                return SendWithReconnect(
                    () => _client.Send((byte)MessageType.GetHeight, request).As<Vector3>(),
                    default
                );
            }
        }

        // ── Server config ────────────────────────────────────────────────

        /// <summary>
        /// Request the server's configuration. Returns null on failure.
        /// </summary>
        public ServerConfig? GetConfig()
        {
            lock (_lock)
            {
                return SendWithReconnect<ServerConfig?>(() =>
                {
                    var response = _client.SendBytes((byte)MessageType.GetConfig, ReadOnlySpan<byte>.Empty);
                    _serverConfig = ParseConfig(response.Data);
                    return _serverConfig;
                }, null);
            }
        }

        // ── Filter configuration ────────────────────────────────────────

        /// <summary>
        /// Set the client state (faction). Call <see cref="ApplyFilter"/> to send to server.
        /// </summary>
        public void SetClientState(ClientState state)
        {
            lock (_lock)
            {
                _state = state;
                IsFilterDirty = true;
            }
        }

        /// <summary>
        /// Set the cost for a single area ID (1-27). Call <see cref="ApplyFilter"/> to send.
        /// </summary>
        public void SetAreaCost(byte areaId, float cost)
        {
            if (areaId >= 1 && areaId <= 27 && float.IsFinite(cost) && cost > 0f)
            {
                lock (_lock)
                {
                    _areaCosts[areaId] = cost;
                    IsFilterDirty = true;
                }
            }
        }

        /// <summary>
        /// Set costs for all 27 areas at once using category-based costs.
        /// Faction multipliers scale alliance/horde area variants.
        /// Call <see cref="ApplyFilter"/> to send to the server.
        /// </summary>
        /// <param name="ground">Base cost for ground, city, WMO, doodad surfaces.</param>
        /// <param name="road">Cost for road surfaces (typically &lt; 1 to prefer roads).</param>
        /// <param name="water">Cost for water and ocean surfaces.</param>
        /// <param name="badLiquid">Cost for lava and slime surfaces.</param>
        /// <param name="allyMult">Multiplier for Alliance faction areas (raise to avoid).</param>
        /// <param name="hordeMult">Multiplier for Horde faction areas (raise to avoid).</param>
        public void SetAreaCosts(float ground, float road, float water, float badLiquid,
                                 float allyMult = 1f, float hordeMult = 1f)
        {
            lock (_lock)
            {
                SetAreaCostsUnlocked(ground, road, water, badLiquid, allyMult, hordeMult);
            }
        }

        private void SetAreaCostsUnlocked(float ground, float road, float water, float badLiquid,
                                          float allyMult, float hordeMult)
        {
            _allyMult = allyMult;
            _hordeMult = hordeMult;

            // Ground
            _areaCosts[AnpArea.TERRAIN_GROUND] = ground;
            _areaCosts[AnpArea.ALLIANCE_TERRAIN_GROUND] = ground * allyMult;
            _areaCosts[AnpArea.HORDE_TERRAIN_GROUND] = ground * hordeMult;

            // Road
            _areaCosts[AnpArea.TERRAIN_ROAD] = road;
            _areaCosts[AnpArea.ALLIANCE_TERRAIN_ROAD] = road * allyMult;
            _areaCosts[AnpArea.HORDE_TERRAIN_ROAD] = road * hordeMult;

            // City
            _areaCosts[AnpArea.TERRAIN_CITY] = ground;
            _areaCosts[AnpArea.ALLIANCE_TERRAIN_CITY] = ground * allyMult;
            _areaCosts[AnpArea.HORDE_TERRAIN_CITY] = ground * hordeMult;

            // WMO
            _areaCosts[AnpArea.WMO] = ground;
            _areaCosts[AnpArea.ALLIANCE_WMO] = ground * allyMult;
            _areaCosts[AnpArea.HORDE_WMO] = ground * hordeMult;

            // Doodad
            _areaCosts[AnpArea.DOODAD] = ground;
            _areaCosts[AnpArea.ALLIANCE_DOODAD] = ground * allyMult;
            _areaCosts[AnpArea.HORDE_DOODAD] = ground * hordeMult;

            // Water
            _areaCosts[AnpArea.LIQUID_WATER] = water;
            _areaCosts[AnpArea.ALLIANCE_LIQUID_WATER] = water * allyMult;
            _areaCosts[AnpArea.HORDE_LIQUID_WATER] = water * hordeMult;

            // Ocean
            _areaCosts[AnpArea.LIQUID_OCEAN] = water;
            _areaCosts[AnpArea.ALLIANCE_LIQUID_OCEAN] = water * allyMult;
            _areaCosts[AnpArea.HORDE_LIQUID_OCEAN] = water * hordeMult;

            // Lava (always expensive, faction doesn't matter)
            _areaCosts[AnpArea.LIQUID_LAVA] = badLiquid;
            _areaCosts[AnpArea.ALLIANCE_LIQUID_LAVA] = badLiquid;
            _areaCosts[AnpArea.HORDE_LIQUID_LAVA] = badLiquid;

            // Slime
            _areaCosts[AnpArea.LIQUID_SLIME] = badLiquid;
            _areaCosts[AnpArea.ALLIANCE_LIQUID_SLIME] = badLiquid;
            _areaCosts[AnpArea.HORDE_LIQUID_SLIME] = badLiquid;

            IsFilterDirty = true;
        }

        /// <summary>
        /// Cost of a surface category, translated to the area ids of the server's navmesh format. 0 forbids it
        /// (e.g. no water for a bot that can't swim), a cost above 0 also allows a category the server excludes by
        /// default (steep slopes on TrinityCore MMAPs, jumps). On ANP servers Ground/Road/Water/BadLiquid replace the
        /// category's cost of <see cref="SetAreaCosts"/> (faction multipliers still apply). Call
        /// <see cref="ApplyFilter"/> to send. Requires protocol version 3, older servers only get the ANP costs.
        /// </summary>
        public void SetTerrainCost(TerrainType type, float cost)
        {
            if (!float.IsFinite(cost) || cost < 0f)
                return;

            lock (_lock)
            {
                _terrainCosts[type] = cost;
                IsFilterDirty = true;
            }
        }

        /// <summary>Back to the server's default for a category. Call <see cref="ApplyFilter"/> to send.</summary>
        public void ClearTerrainCost(TerrainType type)
        {
            lock (_lock)
            {
                IsFilterDirty |= _terrainCosts.Remove(type);
            }
        }

        /// <summary>Back to the server's defaults for all categories. Call <see cref="ApplyFilter"/> to send.</summary>
        public void ClearTerrainCosts()
        {
            lock (_lock)
            {
                IsFilterDirty |= _terrainCosts.Count > 0;
                _terrainCosts.Clear();
            }
        }

        /// <summary>
        /// A bot that can't swim never gets a path through water (water cost 0), true goes back to the default.
        /// Call <see cref="ApplyFilter"/> to send.
        /// </summary>
        public void SetCanSwim(bool canSwim)
        {
            if (canSwim)
                ClearTerrainCost(TerrainType.Water);
            else
                SetTerrainCost(TerrainType.Water, 0f);
        }

        /// <summary>
        /// Send the current filter configuration to the server.
        /// Returns true on success. Call this after changing state or area costs.
        /// The ANP area costs are only sent to servers using ANP navmeshes, MMAP servers
        /// (different area ids) only receive the client state.
        /// </summary>
        public bool ApplyFilter()
        {
            lock (_lock)
            {
                return SendWithReconnect(ApplyFilterInternal, false);
            }
        }

        // ── Internals ───────────────────────────────────────────────────

        private Vector3[]? SendPathRequest(MessageType type, int mapId, Vector3 start, Vector3 end, PathFlags flags,
                                           PathTuning? tuning)
        {
            lock (_lock)
            {
                return SendWithReconnect(() =>
                {
                    var points = Send(type, MakePathRequest(mapId, start, end, flags, tuning)).AsArray<Vector3>();
                    if (points.Length == 0) return null;

                    // Server returns a single zero vector on failure
                    if (points.Length == 1 && points[0].IsZero) return null;

                    return points;
                }, null);
            }
        }

        private static PathTuningData ToWire(PathTuning tuning) => new()
        {
            WallMargin = tuning.WallMargin,
            CornerRadius = tuning.CornerRadius,
            CornerPoints = tuning.CornerPoints,
            StopDistance = tuning.StopDistance,
        };

        /// <summary>PathRequestData, with the tuning trailer if given (older servers ignore extra bytes).</summary>
        private static byte[] MakePathRequest(int mapId, Vector3 start, Vector3 end, PathFlags flags, PathTuning? tuning)
        {
            var request = new PathRequestData { MapId = mapId, Flags = (int)flags, Start = start, End = end };

            if (!tuning.HasValue)
            {
                byte[] plain = new byte[Marshal.SizeOf<PathRequestData>()];
                MemoryMarshal.Write(plain, in request);
                return plain;
            }

            if (tuning.Value.WaypointSpacing > 0f || tuning.Value.WaypointHeightError > 0f)
            {
                var withDetail = new PathRequestWithDetail
                {
                    Request = request,
                    Tuning = ToWire(tuning.Value),
                    Detail = new PathDetailData
                    {
                        Spacing = tuning.Value.WaypointSpacing,
                        HeightError = tuning.Value.WaypointHeightError,
                    },
                };
                byte[] detailed = new byte[Marshal.SizeOf<PathRequestWithDetail>()];
                MemoryMarshal.Write(detailed, in withDetail);
                return detailed;
            }

            var withTuning = new PathRequestWithTuning { Request = request, Tuning = ToWire(tuning.Value) };
            byte[] buffer = new byte[Marshal.SizeOf<PathRequestWithTuning>()];
            MemoryMarshal.Write(buffer, in withTuning);
            return buffer;
        }

        private AnTCP.Client.Objects.AnTcpResponse Send(MessageType type, byte[] payload)
            => _client.SendBytes((byte)type, payload);

        /// <summary>PATH_EX: header(pointCount, resultFlags), pointCount Vector3, pointCount flag bytes.</summary>
        private static PathResult? ParsePathEx(ReadOnlySpan<byte> data)
        {
            int headerSize = Marshal.SizeOf<PathExResponseHeader>();

            if (data.Length < headerSize)
                return null;

            var header = MemoryMarshal.Read<PathExResponseHeader>(data);
            int pointSize = Marshal.SizeOf<Vector3>();

            bool withInfo = (header.ResultFlags & 2) != 0;
            bool withLinks = (header.ResultFlags & 16) != 0;

            if (header.PointCount <= 0
                || data.Length != headerSize + (long)header.PointCount * (pointSize + (withInfo ? 3 : 1) + (withLinks ? 4 : 0)))
                return null;

            var points = MemoryMarshal.Cast<byte, Vector3>(data.Slice(headerSize, header.PointCount * pointSize)).ToArray();
            var flagBytes = data.Slice(headerSize + header.PointCount * pointSize, header.PointCount);
            var flags = new PathPointFlags[header.PointCount];

            for (int i = 0; i < flags.Length; i++)
                flags[i] = (PathPointFlags)flagBytes[i];

            byte[]? areas = null;
            PathPointInfo[]? info = null;

            if (withInfo)
            {
                int offset = headerSize + header.PointCount * (pointSize + 1);
                areas = data.Slice(offset, header.PointCount).ToArray();
                var infoBytes = data.Slice(offset + header.PointCount, header.PointCount);
                info = new PathPointInfo[header.PointCount];

                for (int i = 0; i < info.Length; i++)
                    info[i] = (PathPointInfo)infoBytes[i];
            }

            uint[]? linkIds = null;

            if (withLinks)
            {
                int offset = headerSize + header.PointCount * (pointSize + (withInfo ? 3 : 1));
                linkIds = MemoryMarshal.Cast<byte, uint>(data.Slice(offset, header.PointCount * 4)).ToArray();
            }

            return new PathResult(points, flags, (header.ResultFlags & 1) != 0, areas, info,
                                  (header.ResultFlags & 4) != 0, (header.ResultFlags & 8) != 0, linkIds);
        }

        private bool SendAvoidZonesInternal(int mapId, AvoidZone[] zones)
        {
            int headerSize = Marshal.SizeOf<AvoidZonesHeader>();
            int zoneSize = Marshal.SizeOf<AvoidZoneData>();
            byte[] buffer = new byte[headerSize + zones.Length * zoneSize];
            var header = new AvoidZonesHeader { MapId = mapId, ZoneCount = zones.Length };
            MemoryMarshal.Write(buffer.AsSpan(0, headerSize), in header);

            for (int i = 0; i < zones.Length; i++)
            {
                var zone = new AvoidZoneData { Center = zones[i].Center, Radius = zones[i].Radius, Cost = zones[i].Cost };
                MemoryMarshal.Write(buffer.AsSpan(headerSize + i * zoneSize, zoneSize), in zone);
            }

            return _client.SendBytes((byte)MessageType.SetAvoidZones, buffer).As<bool>();
        }

        /// <summary>Area ids of a terrain category for the server's navmesh format.</summary>
        private static byte[] TerrainAreas(TerrainType type, ServerConfig config)
        {
            switch (type)
            {
                case TerrainType.Jump: return [AnpArea.OFFMESH_JUMP];
                case TerrainType.Drop: return [AnpArea.OFFMESH_DROP];
                case TerrainType.Link: return [AnpArea.OFFMESH_LINK];
            }

            if (config.UseAnpFileFormat)
                return []; // ANP categories go through the cost table

            if (config.MmapFormat == 2) // SkyFire 5.4.8: ground 1, magma 2, slime 4, water 8
            {
                return type switch
                {
                    TerrainType.Ground => [1],
                    TerrainType.BadLiquid => [2, 4],
                    TerrainType.Water => [8],
                    _ => [],
                };
            }

            // TrinityCore: ground 11, steep 10, water 9, magma/slime 8
            return type switch
            {
                TerrainType.Ground => [11],
                TerrainType.Steep => [10],
                TerrainType.Water => [9],
                TerrainType.BadLiquid => [8],
                _ => [],
            };
        }

        /// <summary>
        /// Execute a server call. On network failure, attempt auto-reconnect
        /// and retry once. Returns <paramref name="fallback"/> on total failure.
        /// </summary>
        private T SendWithReconnect<T>(Func<T> action, T fallback)
        {
            try
            {
                return action();
            }
            catch (Exception ex) when (IsNetworkError(ex))
            {
                if (!AutoReconnect)
                {
                    OnDisconnected();
                    return fallback;
                }

                if (!TryAutoReconnect())
                    return fallback;

                // Retry once after successful reconnect
                try
                {
                    return action();
                }
                catch
                {
                    return fallback;
                }
            }
            catch
            {
                return fallback;
            }
        }

        private bool TryAutoReconnect()
        {
            OnDisconnected();

            int maxAttempts = MaxReconnectAttempts;
            int attempt = 0;

            while (maxAttempts == 0 || attempt < maxAttempts)
            {
                attempt++;
                Reconnecting?.Invoke(attempt);

                Thread.Sleep(ReconnectDelayMs);

                try
                {
                    if (_client.TryReconnect())
                    {
                        _wasConnected = true;
                        Connected?.Invoke();

                        // The server may have been restarted with a different config, re-query it and
                        // re-apply the filter so the server knows our faction/costs.
                        _serverConfig = null;
                        try { ApplyFilterInternal(); } catch { }

                        // Avoid zones are per connection on the server.
                        foreach (var (mapId, zones) in _avoidZones)
                        {
                            try { SendAvoidZonesInternal(mapId, zones); } catch { }
                        }

                        return true;
                    }
                }
                catch { }
            }

            return false;
        }

        /// <summary>
        /// Filter apply without the lock or reconnect wrapper (callers hold the lock).
        /// </summary>
        private bool ApplyFilterInternal()
        {
            // ANP area ids mean different areas on MMAP servers (e.g. 8/9 are lava/water in TrinityCore meshes),
            // only send the costs when the server confirmed it uses ANP navmeshes, otherwise just the state.
            _serverConfig ??= QueryConfigInternal();
            var entries = new SortedDictionary<byte, float>();

            if (_serverConfig is { UseAnpFileFormat: true })
            {
                var costs = (float[])_areaCosts.Clone();

                // Terrain categories replace the category's base cost (faction multipliers still apply), 0 forbids.
                if (_serverConfig.ProtocolVersion >= 3)
                {
                    void Category(TerrainType type, byte first, bool factions)
                    {
                        if (!_terrainCosts.TryGetValue(type, out float cost))
                            return;

                        costs[first] = cost;
                        costs[first + 1] = factions ? cost * _allyMult : cost;
                        costs[first + 2] = factions ? cost * _hordeMult : cost;
                    }

                    Category(TerrainType.Ground, AnpArea.TERRAIN_GROUND, true);
                    Category(TerrainType.Ground, AnpArea.TERRAIN_CITY, true);
                    Category(TerrainType.Ground, AnpArea.WMO, true);
                    Category(TerrainType.Ground, AnpArea.DOODAD, true);
                    Category(TerrainType.Road, AnpArea.TERRAIN_ROAD, true);
                    Category(TerrainType.Water, AnpArea.LIQUID_WATER, true);
                    Category(TerrainType.Water, AnpArea.LIQUID_OCEAN, true);
                    Category(TerrainType.BadLiquid, AnpArea.LIQUID_LAVA, false);
                    Category(TerrainType.BadLiquid, AnpArea.LIQUID_SLIME, false);
                }

                for (byte area = 1; area <= 27; area++)
                    entries[area] = costs[area];
            }

            // Old servers reject cost 0 and don't know the categories.
            if (_serverConfig is { ProtocolVersion: >= 3 })
            {
                foreach (var (type, cost) in _terrainCosts)
                {
                    foreach (byte area in TerrainAreas(type, _serverConfig))
                        entries[area] = cost;
                }
            }

            // Wire format: [state(1)+pad(3)][count(4)][entries: {areaId(1)+pad(3)+cost(4)} x N]
            const int headerSize = 8;
            const int entrySize = 8;
            byte[] buffer = new byte[headerSize + entries.Count * entrySize];

            buffer[0] = (byte)_state;
            BitConverter.GetBytes(entries.Count).CopyTo(buffer, 4);
            int index = 0;

            foreach (var (areaId, cost) in entries)
            {
                int off = headerSize + index++ * entrySize;
                buffer[off] = areaId;
                BitConverter.GetBytes(cost).CopyTo(buffer, off + 4);
            }

            bool result = _client.SendBytes((byte)MessageType.ConfigureFilter, buffer).As<bool>();

            if (result)
                IsFilterDirty = false;

            return result;
        }

        private ServerConfig? QueryConfigInternal()
        {
            try
            {
                var response = _client.SendBytes((byte)MessageType.GetConfig, ReadOnlySpan<byte>.Empty);
                return ParseConfig(response.Data);
            }
            catch (Exception ex) when (!IsNetworkError(ex))
            {
                return null;
            }
        }

        /// <summary>
        /// GET_CONFIG response: mmapFormat(4) useAnp(4) pathLength(4) path[pathLength], then (protocol 2+)
        /// protocolVersion(4) maxPointPath(4) versionLength(4) version[versionLength].
        /// </summary>
        private static ServerConfig? ParseConfig(ReadOnlySpan<byte> data)
        {
            if (data.Length < 12)
                return null;

            int mmapFormat = BitConverter.ToInt32(data.Slice(0, 4));
            bool useAnp = BitConverter.ToInt32(data.Slice(4, 4)) != 0;
            int pathLen = BitConverter.ToInt32(data.Slice(8, 4));

            // long: 12 + pathLen must not wrap for a corrupt length.
            if (pathLen < 0 || data.Length < 12L + pathLen)
                return new ServerConfig(mmapFormat, useAnp, "");

            string meshesPath = Encoding.UTF8.GetString(data.Slice(12, pathLen));
            var trailer = data.Slice(12 + pathLen);

            if (trailer.Length < 12)
                return new ServerConfig(mmapFormat, useAnp, meshesPath);

            int protocolVersion = BitConverter.ToInt32(trailer.Slice(0, 4));
            int maxPointPath = BitConverter.ToInt32(trailer.Slice(4, 4));
            int versionLen = BitConverter.ToInt32(trailer.Slice(8, 4));
            string version = versionLen > 0 && trailer.Length >= 12L + versionLen
                ? Encoding.UTF8.GetString(trailer.Slice(12, versionLen))
                : "";

            return new ServerConfig(mmapFormat, useAnp, meshesPath, protocolVersion, maxPointPath, version);
        }

        private void OnDisconnected()
        {
            if (_wasConnected)
            {
                _wasConnected = false;
                Disconnected?.Invoke();
            }
        }

        private static bool IsNetworkError(Exception ex)
        {
            return ex is IOException
                || ex is SocketException
                || ex is ObjectDisposedException
                || (ex.InnerException != null && IsNetworkError(ex.InnerException));
        }

        public void Dispose()
        {
            if (_disposed)
                return;

            _disposed = true;

            lock (_lock)
            {
                try { _client.Dispose(); } catch { }
            }
        }
    }
}
