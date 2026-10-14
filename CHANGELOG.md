# Changelog

## 1.10.0.0 (protocol 3)

Older clients keep working: every addition is a new message type, a new flag or an optional trailer.

### Added

- **TrinityCore master MMAPs** (MMAP v16, retail and classic clients): `iMmapFormat=3`, also picked by auto
  detection (`iMmapFormat=0`) from the magic in front of the `.mmap` files. The header in front of the navmesh
  parameters is read and its version checked. `sParentMaps` (`child:parent` pairs from Map.db2's ParentMapID) lets
  phased child maps without their own files use their parent's, like the worldserver does.
- **Path refinement on the navmesh** (PATH, RANDOM_PATH, EXPLORE_POLY, PATH_EX):
  - `128` shortcut: skip corners wherever the navmesh allows a straight line
  - `64` wall margin: move corners `fWallMargin` out of the turn, away from the wall they bend around
  - `256` round corners: replace corners by arcs of `fCornerRadius` around them (`iCornerPoints` points)

  Every changed segment is checked with a navmesh raycast that has to arrive on the end point's surface, a step that
  fails is skipped (half the distance is tried first): the result is never worse than the plain path.
- **Per request tuning**: optional trailer `float wallMargin, float cornerRadius, int cornerPoints, float
  stopDistance`.
- **Stop distance**: end a path a given distance before the target (melee or casting range), at a point that sees the
  target over the navmesh (around a wall if needed).
- **Follow** (flag `512`): reuse the connection's corridor while walking along it, corners stay the same between
  requests and the request is cheap.
- **Avoid zones**, message `11` SET_AVOID_ZONES: up to 64 circles per connection and map that cost more to cross
  (height aware). The search prefers other ways and paths are led around them.
- **Stuck reports**, message `13` REPORT_STUCK: a zone every client avoids, appended to `sAvoidZonesFile` and loaded
  again at startup. Repeated reports at a spot raise its cost.
- **Local steering**, message `12` STEER: the velocity for one tick that avoids up to 32 nearby units, the avoid zones
  and the navmesh borders (Detour's obstacle avoidance), plus the resulting position on the navmesh.
- **Forbidden and allowed terrain**: a CONFIGURE_FILTER cost of `0` forbids an area (no water for a bot that can't
  swim), a cost above `0` allows one the server excludes by default (steep slopes on TrinityCore MMAPs, jumps).
- **Off-mesh connections** without regenerating navmeshes: from a file in TrinityCore's `offmesh.txt` format
  (`sOffMeshFile`) and automatic one way drop-down links from ledges (`fAutoDropHeight`, `fAutoDropReach`, off by
  default). The tiles concerned are rebuilt when a map loads (`TileRebuilder`). Area ids `40` jump, `41` drop, `42`
  link and flags `NAV_JUMP`/`NAV_DROP` for ANP and MMAP navmeshes; jumps are excluded by default.
- **PATH_EX** (message `14`): the path with a flag per point (start, end, where an off-mesh connection starts and
  whether it is a jump, a drop or a link) and a partial flag. `1024` RANDOMIZE adds random offsets.
- **C# client**: `PathFlags.AvoidWalls`, `Shortcut`, `RoundCorners`, `Follow`, `Randomize` and `Fluid` (the first
  three), `PathTuning`, `GetPathEx` (`PathResult`, `PathPointFlags`), `SetAvoidZones`/`ClearAvoidZones`
  (`AvoidZone`, sent again after a reconnect), `Steer` (`SteerUnit`, `SteerResult`), `ReportStuck`,
  `SetTerrainCost`/`ClearTerrainCost(s)`/`SetCanSwim` (`TerrainType`, translated to the area ids of the server's
  navmesh format). The .NET assemblies take their version from `Version.hpp`.
- Config keys: `fWallMargin`, `fCornerRadius`, `iCornerPoints`, `sOffMeshFile`, `fAutoDropHeight`, `fAutoDropReach`,
  `sAvoidZonesFile`, `fStuckZoneRadius`, `fStuckZoneCost`, `sParentMaps`.

### Changed

- Start and end positions are looked up within 6 yards, then within 6 yards horizontally and 50 vertically: a
  character in the air still gets a path, starting on the navmesh below it (matters in tiles with tall terrain,
  Detour clamps the vertical search to the tile's height range).
- RANDOM_PATH only keeps a random offset when the way to it and on to the next corner stays on the navmesh, and never
  moves the ends of off-mesh connections.
- EXPLORE_POLY orders its waypoints with a penalty on turns: routes walk rows instead of zigzags.
- Navmesh parameters with a `maxPolys` that is negative as a signed integer are accepted (TrinityCore master writes
  `1u << DT_POLY_BITS`, unused with 64 bit poly refs).
- Vendored Detour: `DT_VIRTUAL_QUERYFILTER` enabled (client filters derive from `dtQueryFilter`).

### Fixed

- CAST_RAY and CAST_RAY_EX reported rays as blocked that start or end exactly on a polygon corner or edge (like the
  points of a path). The ray is now cast from every polygon touching the start and a hit within 5 cm of the end counts
  as clear.

### Tested

Everything above runs in the CI suites on the synthetic test world: unit, end-to-end (pipeline, MMAP v16 files
written from the test world, off-mesh links, refinement, avoidance, TCP server) and the C# client against a real
server (`tests/run_client_integration.sh`), plus ASan/UBSan. Not tested on real client data in this change (none was
available): the MMAP v16 reader follows TrinityCore master's `MMapDefines.h` (commit of 2026-10-12).
