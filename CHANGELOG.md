# Changelog

## 1.12.0.0 (protocol 4)

The world beyond the navmesh, after studying namigator (static geometry next to the navmesh, game objects rebuilt at
runtime) and a comparison of runtime topologies: what is where, where it comes from, and what changes. Protocol 3
clients keep working (new messages and flags only); servers without `.anw` files answer the new queries with "no
static world". Re-export to get the `.anw` and `gameobjects.ano` files.

### Static world

- **`.anw` files** next to the `.anp` (exporter, `--no-world` to skip): the collision geometry the navmesh was built
  from with the provenance of every triangle: terrain, WMO (placement unique id), M2 doodad (unique id), liquid, game
  object (display id), the navmesh area (0 = too steep to stand on) and the AreaTable id of every chunk. MPQ and
  `--tc-data` exports. About 350 KB per ADT of terrain plus the models.
- **Server queries** (tiles read on demand, `iWorldCacheTiles` per map, bounding volume hierarchy per tile):
  - `15` LINE_OF_SIGHT against the geometry, with what blocks it (doodads, game objects, WMOs or terrain can be
    ignored; liquids never block)
  - `16` SURFACES: every surface under a point, highest first (decks, bridges, floors, water, the ground) with source,
    normal, standable/liquid flags and the zone's AreaTable id
  - `17` SWEEP: a standing body (radius, height) moved along a segment, the first geometry it runs into above step
    height (rays at three heights on the center and both sides)
- C# client: `LineOfSight`, `GetSurfaces`, `Sweep` (`WorldHit`, `WorldSurface(s)`, `WorldSource`, `WorldIgnore`).
- Config: `sWorldPath`, `iWorldCacheTiles`.

### Game objects at runtime

- `18` SET_OBJECTS: add, move, open/close or remove game objects (guid, display id, position, rotation, scale). Their
  collision is in the static world queries at once; the navmesh tiles around them are rebuilt in the background with
  the export's settings and area rects (stored in the `.anw`) and swapped in while requests on the map wait. A
  closed gate makes paths go around, opened or removed they go through again (tests). C#: `SetObjects`
  (`WorldObject`, `Quaternion`, `SetObjectsResult`).
- Exporter: `gameobjects.ano`, the collision models of every game object display (GameObjectDisplayInfo.dbc, or
  `GameObjectModels.dtree` with `--tc-data`), and the build settings and area rects per ADT in the `.anw`.
- Path flag `2048` CLEAR_BODY (PATH, RANDOM_PATH, PATH_EX): a body (`fBodyRadius`, `fBodyHeight`,
  `fBodyStepHeight`) is swept along the path through the static world and the game objects; a segment it can't walk
  gets a point beside the obstacle (on the navmesh, both new segments clear, point flag `64` BYPASS) or the point flag
  `128` BLOCKED. Catches what the navmesh doesn't know yet (an object whose tile is still being rebuilt) and fences
  on navmeshes that walk over them (TrinityCore's, with a `.anw` from `--tc-data`). C#: `PathFlags.ClearBody`,
  `PathPointFlags.Bypass`/`Blocked`.
- Navmesh requests hold a per map readers/writer lock (re-entrant for a thread, writers wait for running requests);
  replacing tiles takes it exclusively.

### Detailed paths

- Path flag `4096` DETAIL (PATH, RANDOM_PATH, PATH_EX): dense waypoints on the navmesh surface. Each segment's height
  profile is sampled every half yard on the polygons the navmesh raycast walks (the right floor under bridges and in
  buildings); waypoints go on both sides of every change of ground (bisection to the polygon border: water, road,
  building floor, faction border), where the profile bends (Douglas-Peucker on the heights, `fWaypointHeightError`)
  and at most `fWaypointSpacing` apart. Corners and off-mesh connections keep their place and flags; a path that
  wouldn't fit gets a larger spacing. Optional `PathDetailData` trailer (spacing, height error) per request; C#:
  `PathFlags.Detail`, `PathTuning.WaypointSpacing`/`WaypointHeightError`.

### Fixed

- A data race in glibc's `tzset` when several threads read archives (miniz converted ZIP entry times nobody uses):
  miniz is built with `MINIZ_NO_TIME`.
- A racy check in the stats test (the last request is recorded after its answer went out).

### Tested

Unit, end-to-end and exporter suites in Release and with ASan+UBSan, the end-to-end suite with TSan (requests on
four threads while a gate is opened and closed and its tile replaced), the C# client against the real server, the
fuzzers (new `FuzzWorld` for `.anw` tiles and their bounding volume hierarchy, protocol fuzzing with the new
messages). TSan found a use after free in the first version of the dynamic object queries and fuzzing a `memcpy` with
a null pointer for empty tiles, both fixed before release. Not tested on real client data (no client available):
`.anw` sizes are measured on synthetic ADTs, tile rebuild times on the test world (~50 ms) and the benchmark ADT
(~0.7 s).

### Limits and leads

- SWEEP and CLEAR_BODY approximate the body with 9 rays: a thin horizontal deck between the rays isn't seen.
- Rebuilt tiles drop the server's automatic drop links (`fAutoDropHeight`); the export's jump/drop links and the
  `sOffMeshFile` connections are rebuilt.
- Objects are world state shared by all clients of a navigation server: bots on different realms (where the same
  door can be open on one and closed on the other) need one navigation server per realm.
- Leads: a native CASC reader (wow.export's readers document the formats: split ADTs, FileDataIDs, WDC DB2) to skip
  the TrinityCore extraction; per client object sets; step transitions (door thresholds) from the surfaces along a
  path.

## 1.11.0.0

Navmeshes for recent clients and obstacles that block. The ANP format, area ids and the protocol (3) are unchanged:
navmeshes of 1.10 load as before, re-export to get the obstacle changes.

### Exporter: obstacles

- **Climb 1.0 yard** (was 1.2): obstacles from about 1.05 yards on block the way. With 1.2 the navmesh led over
  1.1 yard fences and bots ran into them (`tests/E2EObstacleTests.cpp` reproduces it). Stairs (0.45 yard risers),
  narrow passages and low ceilings behave as before. `--climb`, `--radius`, `--height`, `--slope` tune the agent.
- **Fences never pass as steps**: Recast rounds span tops up to the next cell, so a step between two spans is off by up
  to one cell. The climb is now rounded down to cells and the cell height is half the cell size where the tile's
  height range allows it (13 bit spans): obstacles from climb + one cell height (~1.1 yards) always block. A 1.1 yard
  fence on ground at an unlucky height passed before (test included).
- **Game objects** (`--gameobjects <file>`): the collision of server spawned objects (fences, gates, stalls...) from a
  spawn file (TrinityCore/AzerothCore world database export, or what a bot logged), models from
  GameObjectDisplayInfo.dbc or a model path, scale and quaternion/orientation. Doors, transports, destructible
  buildings and trapdoors are left out by default (`--gameobject-exclude`).
- **WMO collision faces like the client** (TrinityCore's vmap extractor rule): a triangle collides if it is flagged
  as collision, or rendered and not a detail. Before, faces with neither flag (portals, hints) were phantom walls
  and solid detail faces (railings) were dropped.
- **Jump and drop links baked into the tiles**, checked against the source geometry (vertical profile between
  takeoff and landing): jumps over obstacles up to 1.5 yards (`--jump-height`), over gaps and up onto things up to
  1.25 yards, drops down ledges up to 10 yards (`--drop-height`). Walls, ceilings and taller obstacles in the way
  reject a link. Jumps are excluded by the server's default filters, PATH_EX tells a bot where to jump.
- **M2 collision of every version**: vanilla and TBC models (version < 264) have 20 more header bytes in front of the
  collision fields, they were read with the WotLK layout (collision missing or garbage). Chunked MD21 models
  (Legion+) are read too.

### Exporter: recent clients

- **`--tc-data`**: navmeshes from TrinityCore master's extracted data (`maps/*.map` from `mapextractor`, `vmaps/` from
  `vmap4extractor` + `vmap4assembler`), the way to export CASC clients (Legion and later, the classic flavours) with
  our pipeline instead of TrinityCore's `mmaps_generator`: fences, climb, jump and drop links, game objects (display
  ids from `GameObjectModels.dtree`). Terrain with holes (float, uint16 and uint8 heights, flat tiles), liquids per
  square with deep ocean left out like TrinityCore does, WMO and M2 models with TrinityCore's transforms, WMO liquids.
  Faction and city areas from an AreaTable CSV export (`--area-table`), WMO liquid kinds from a LiquidType CSV
  (`--liquid-types`, else the classic ids). No road areas (not in TrinityCore's files).
- Exporter refactored: extraction (MPQ or TrinityCore files) and the navmesh build/save step are separate.


- **`anav_navmesh2ply`**: a navmesh (ANP or any MMAP format) as a binary PLY file colored by slope or area, with
  off-mesh connections as ribbons, optionally cut to a box around a spot.
- **`anav_db2info`**: header of DB2 files (WDC3-5) checked against TrinityCore's `ExtractorDB2LoadInfo.h`: which
  tables a client build changed (layout hash, field count) when the extractors refuse it.
- README: step by step guide for recent clients (extraction, DB2 layouts, export, server settings, checks on a real
  client).

### Fixed

- A constant named `MAX_PATH` in the jump/drop link code collided with the `MAX_PATH` macro of `windows.h` and
  StormLib (compile error depending on the include order, e.g. MSVC builds of the server).

### Tested

Unit, end-to-end and exporter suites (Release, ASan+UBSan), the C# client against the real server, the fuzzers (2
minutes each, the exporter's new TrinityCore formats 5 more minutes; `FuzzExporter` has 5 new formats: `.map`, `.vmo`,
`.vmtile`, `GameObjectModels.dtree`, DB2 headers). `anav_db2info` was run against TrinityCore master's real
`ExtractorDB2LoadInfo.h` (it reports the `Map.db2` change of build 70245).

### Limits and leads

- **No real client data was available** while writing this (cloud container, no WoW client; the CDN hosts
  `us.patch.battle.net`, `*.version.battle.net` and `wago.tools` were blocked by the network policy). Everything is
  tested on synthetic data written like the client or TrinityCore's tools do. Not verified on real files: the
  `--tc-data` reader (`.map` v10, vmaps `VMAP_4.E`, TrinityCore master of October 2026), the MMAP v16 reader, the
  model axes of game objects (x forward, z up, turned around z), the modern AreaTable flag used for capitals (0x08).
  README "Checking a real client" lists what to look at first.
- Jump and drop links are only built inside a tile: none across ADT borders (the landing must be in the same tile).
- The server's automatic drops (`fAutoDropHeight`) only see the navmesh, not the geometry: they can't tell a ledge
  from a wall top and are off by default; the exporter's links are checked against the geometry.
- `--tc-data` has no road areas (TrinityCore's files keep no textures) and needs CSV exports for faction and city
  areas. A CASC reader in the exporter (CascLib) would remove the TrinityCore extraction step and give roads back.
- Game objects need a spawn file (server world database or what a bot logs); doors are assumed open.
- Leads: links across tile borders (build them per pair of neighbouring tiles), per map agent settings (e.g. a
  higher climb for maps without fences), swimming links between water and shore, a CASC reader.

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
