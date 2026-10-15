# AmeisenNavigation 🐜

[![CI](https://github.com/Jnnshschl/AmeisenNavigation/actions/workflows/ci.yml/badge.svg)](https://github.com/Jnnshschl/AmeisenNavigation/actions/workflows/ci.yml)

TCP navigation server and navmesh exporter for World of Warcraft bots (built for the AmeisenBotX), based on
[recastnavigation](https://github.com/recastnavigation/recastnavigation). Runs on **Windows and Linux**.

- **AmeisenNavigation.Server**: answers path, raycast, height, steering and random point queries over a tiny binary
  TCP protocol. Serves TrinityCore (3.3.5a and **master**: retail and classic clients) and SkyFire **MMAPs** or its own
  **ANP** navmeshes.
- **AmeisenNavigation.Exporter**: builds ANP navmeshes straight from the game's MPQ archives, with area types for
  terrain, roads, cities, WMOs, doodads, water/ocean/lava/slime and Alliance/Horde territory.
- **AmeisenNavigation.Client** (C#): client library with auto reconnect, used by the bot and the WPF tester.

## Features 🚀

- Straight and randomized paths, path smoothing (Chaikin, Catmull-Rom, Bezier) with optional navmesh validation
- Path refinement on the navmesh: shortcuts, distance to walls, round corners, stop in range of a target (melee or
  casting range, with line of sight), stable corners while following a path; every changed segment is verified
- Off-mesh connections: where to jump, drop down or take a custom link (`PATH_EX`), from TrinityCore's `offmesh.txt`
  format and automatic drop-down links from ledges
- Avoid zones per client (danger areas), shared stuck reports (places bots got stuck are avoided by everyone), terrain
  rules (forbid water for a bot that can't swim, allow steep slopes), local steering around other units
- Game objects at runtime: doors and gates the bot sees change the navmesh (tiles rebuilt in the background) and the
  line of sight
- Static world: the collision geometry next to the navmesh with the provenance of every triangle (terrain, WMO, M2,
  liquid, game object, with their ids): line of sight for spells (trees optional), every surface under a point
  (decks, bridges, floors, water) with the zone's AreaTable id, a standing body swept along a segment (what it runs
  into above step height)
- Move small deltas along the navmesh surface, movement raycasts (with hit position and wall normal)
- Ground height queries that pick the right floor (bridges, buildings, caves)
- Random points on the mesh / around a position
- Area exploration: a short route that covers a polygon (grinding/gathering areas), see `EXPLORE_POLY`
- Partial path detection: optionally fail instead of walking to the closest point of an unreachable target
- Per client query filters: faction aware costs (avoid enemy territory), custom area costs, ghost mode
- Thread per connection, concurrent clients never block each other, maps loaded lazily or at startup (`sPreloadMaps`)
- Docker image and a hardened systemd unit, configuration through a file and/or `ANAV_<key>` environment variables

## Quick start 📝

1. Download the latest [release](https://github.com/Jnnshschl/AmeisenNavigation/releases) (or build it, see below).
2. Get navmeshes:
   - **ANP** (recommended): export them with the exporter (see [Exporter](#exporter)).
   - **MMAP**: generate them with the TrinityCore `mmaps_generator` (or SkyFire for 5.4.8). For recent clients
     (retail/classic, CASC) see [Recent clients](#recent-clients-casc).
3. Start the server once, it creates `config.cfg` next to the executable. Set `sMmapsPath` (and `bUseAnpFileFormat=1`
   for ANP navmeshes), then start it again. A different config file can be passed as first argument.

```
AmeisenNavigation.Server [config.cfg]
```

Or with Docker (see [Deployment](#deployment-)):

```bash
docker build -t ameisennav .
docker run -d -p 47110:47110 -v /srv/meshes:/meshes:ro ameisennav
```

### Server configuration

| Key | Default | Description |
| --- | --- | --- |
| `sIp` / `iPort` | `127.0.0.1` / `47110` | Listen address and port |
| `sMmapsPath` | `C:\meshes\` (`./meshes/` on Linux) | Folder with the `.anp` or `.mmap`/`.mmtile` files |
| `bUseAnpFileFormat` | `0` | `1` = ANP navmeshes from the exporter, `0` = MMAPs |
| `iMmapFormat` | `0` | `0` auto detect, `1` TrinityCore 3.3.5a, `2` SkyFire 5.4.8, `3` TrinityCore master (MMAP v16), `-1` custom patterns |
| `sWorldPath` | empty | Folder of the static worlds (`.anw`), empty = `sMmapsPath` |
| `iWorldCacheTiles` | `48` | Static world tiles kept in memory per map |
| `fWaypointSpacing`, `fWaypointHeightError` | `4`, `0.3` | DETAIL: longest distance between waypoints, height deviation that adds a waypoint |
| `fEdgeMargin` | `1` | DETAIL: distance of the added waypoints from the navmesh edge (cliffs, docks, forbidden water), `0` = off |
| `fBodyRadius`, `fBodyHeight`, `fBodyStepHeight` | `0.4`, `2`, `0.75` | Body swept by the clear body path flag; obstacles up to the step height are walked over |
| `sParentMaps` | empty | MMAPs: `child:parent` map pairs (Map.db2 ParentMapID), a child map without its own files uses the parent's, e.g. `2552:0` |
| `sCustomMmapPattern` / `sCustomMmtilePattern` | TC 3.3.5a names | `std::format` patterns for `iMmapFormat=-1` (args: mapId, x, y) |
| `sPreloadMaps` | empty | Comma separated map ids loaded at startup, e.g. `0,1,530,571` |
| `iMaxPolyPath` | `2048` | Max polygons of a path corridor |
| `iMaxPointPath` | `2048` | Max points of a returned path |
| `iMaxSearchNodes` | `65535` | A* node pool size per client and map |
| `fFactionDangerCost` | `3.0` | Cost multiplier for enemy faction areas (ANP) |
| `fWaterCost` / `fBadLiquidCost` | `1.6` / `4.0` | Default costs of water/ocean and lava/slime (ANP and MMAP) |
| `fRoadCost` | `0.75` | Default cost of roads (ANP, < 1 prefers roads) |
| `fRandomPathMaxDistance` | `1.0` | Max offset of randomized path corners |
| `fWallMargin` | `1.0` | Distance of corners from the walls they bend around (flag `64`) |
| `fCornerRadius` / `iCornerPoints` | `2.0` / `4` | Arcs replacing corners (flag `256`) |
| `sOffMeshFile` | empty | Off-mesh connections in TrinityCore's `offmesh.txt` format, added when a map loads |
| `fAutoDropHeight` / `fAutoDropReach` | `0` / `2.0` | One way drop-down links from ledges up to this height (`0` = off), landing up to this far beyond the ledge (see below) |
| `sAvoidZonesFile` | empty | Stuck reports (`REPORT_STUCK`) are kept in this file and loaded again at startup |
| `fStuckZoneRadius` / `fStuckZoneCost` | `3.0` / `8.0` | Radius (when the report has none) and cost multiplier of stuck zones |
| `iCatmullRomSplinePoints` / `fCatmullRomSplineAlpha` | `4` / `0.5` | Catmull-Rom samples per segment / parametrization |
| `iBezierCurvePoints` | `8` | Bezier samples per curve |
| `iMaxClients` | `1024` | Simultaneous connections (one thread each), further ones are refused, `0` = unlimited |
| `iClientIdleTimeoutSec` | `0` | Disconnect clients that send nothing for this long, `0` = never (dead peers are detected by TCP keepalive after ~2 min anyway) |
| `iStatsIntervalSec` | `300` | Log request counts, rates and latencies per request type this often, `0` = off |
| `bDebugLogging` | `0` | Log every request (with timings) |

Invalid values are reported and replaced by defaults, `#`/`;` start comments. Every key can be overridden by an
environment variable named `ANAV_<key>` (e.g. `ANAV_sMmapsPath=/meshes`, `ANAV_iPort=47111`), the server then also
runs without a config file. `CTRL+C` (or `SIGTERM`) shuts the server down gracefully.

## Exporter

```
AmeisenNavigation.Exporter --wow <WoW folder> --output <mesh folder> [options]       # MPQ clients (up to WotLK)
AmeisenNavigation.Exporter --tc-data <folder> --output <mesh folder> [options]       # recent clients, see below

  -m, --map <ids>      Only export these map ids (comma separated, e.g. 0,1,530,571)
  -t, --tile <x,y>     Only export a single ADT (debugging)
  -j, --threads <n>    Worker threads (default: all cores)
  -s, --skip-existing  Don't rebuild maps whose .anp file already exists (resume an interrupted export)
  -d, --debug          Debug logging + area debug images (<output>/debug/area_<map>_<x>_<y>.bmp)
  -l, --list-maps      List the maps in Map.dbc (or the maps with files in --tc-data)
      --no-world       Don't write the static world (.anw: collision geometry with provenance, ~350 KB per ADT of
                       terrain plus the models, needed for LINE_OF_SIGHT, SURFACES, SWEEP and SET_OBJECTS) and
                       gameobjects.ano (collision models of every game object display)
  -g, --gameobjects <file> Add the collision of game object spawns (see Game objects)
      --area-table <csv>   With --tc-data: AreaTable as CSV, for faction and city areas
      --liquid-types <csv> With --tc-data: LiquidType as CSV, for the kind of WMO liquids
      --climb <h>      Highest step walked up without jumping (default 1.0 yards), obstacles above it block
      --radius <r>     Agent radius, distance kept from walls (default 0.6)
      --height <h>     Agent height, lower ceilings block (default 2.0)
      --slope <deg>    Steepest walkable slope (default 55)
      --jump-height <h> Highest obstacle jumped over, 0 = no jump links (default 1.5)
      --drop-height <h> Highest ledge dropped down, 0 = no drop links (default 10)
```

The exit code is `0` on success, `1` if the client data couldn't be read and `2` if maps failed to build.

The exporter reads the MPQs in the client's load order (custom `patch-X.MPQ` files override the base archives),
extracts terrain, liquids, WMOs and doodads, and builds one Detour tile per ADT:

- Triangles are bucketed per tile and sub-tile, so the build time grows linearly with the map size.
- WMO/doodad placements are extracted once (by unique id) and are never clipped at ADT borders.
- WMO-only maps (most dungeons and raids) are built from the WDT's global WMO.
- Liquids come from MH2O (WotLK, sloped surfaces included) with an MCLQ fallback for older data.
- Tiles that would exceed Detour's 65535 vertex limit are automatically rebuilt with coarser settings.
- Obstacles: WMO triangles collide like in the client (collision flag, or rendered and not a detail), M2 collision
  meshes are read for every model version (vanilla/TBC headers, WotLK, chunked MD21). The default climb (1.0 yard)
  makes fences from about 1.05 yards on block the way: a path never leads over an obstacle the bot can't step over
  (see `tests/E2EObstacleTests.cpp` for fences, stairs, ledges, narrow passages and low ceilings).
- Heights are quantized finely enough for obstacles: the cell height is half the cell size where the tile's height
  range allows it, and the climb is rounded down to cells, so an obstacle from `--climb` + one cell height on always
  blocks wherever the ground lies in a cell (Recast rounds span tops up, a fence used to pass as a step).
- Jump and drop links (off-mesh connections) where the navmesh ends at an obstacle or a ledge, checked against the
  geometry: a jump goes over an obstacle no higher than `--jump-height` above the takeoff (or a gap, or up onto
  something up to 1.25 yards higher) with nothing else in the way up to head height, a drop goes down a ledge with
  nothing in between. A wall between a balcony and the street never becomes a link. Links that would land in the
  neighbouring tile are left out. The server reports them per point (`PATH_EX`); jumps are only used by clients that
  enable them.
- Area ids (see `AmeisenNavigation.Pack/src/AnpFormat.hpp`) come in {neutral, Alliance, Horde} triplets for ground,
  road, city, WMO, doodad, water, ocean, lava and slime.

### Game objects

Fences, gates, stalls, braziers and the like are often game objects spawned by the server, not part of the ADTs:
without them the navmesh leads straight through them. `--gameobjects <file>` adds their collision (M2 or WMO model,
scale, rotation) to the maps being exported. The file is delimited text (comma, semicolon or tab), with a header
line naming the columns or in this order without one:

```
map, displayId, x, y, z, orientation, rotation0, rotation1, rotation2, rotation3, scale, type
```

`displayId` is resolved through `GameObjectDisplayInfo.dbc`, a `model` column (a model path) can be given instead.
The quaternion (`rotation0..3`) wins over `orientation` when it isn't zero. Types `0` (doors, assumed open: a bot opens
them), `11`/`15` (transports), `33` (destructible buildings) and `35` (trapdoors) are left out by default
(`--gameobject-exclude <types|none>`). From a TrinityCore/AzerothCore world database:

```bash
mysql -B world -e "SELECT g.map, t.displayId, g.position_x, g.position_y, g.position_z, g.orientation,
  g.rotation0, g.rotation1, g.rotation2, g.rotation3, t.size, t.type
  FROM gameobject g JOIN gameobject_template t ON t.entry = g.id" > gameobjects.tsv
```

A bot can also write the objects it sees in the game in the same format. Spawns are only added on the ADTs being
exported. The model axes follow the client (x forward, z up, turned by the orientation around z): check a few spawns
in game after an export (`anav_navmesh2ply`, below, shows the navmesh around them).

## Recent clients (CASC)

Clients since Legion (7.0, retail and the classic flavours) store their data in CASC instead of MPQ archives. The
exporter doesn't read CASC itself: TrinityCore's extractors do, and they follow every client build. Two ways from
there, both start with TrinityCore master's tools:

1. Build TrinityCore master's tools only (`cmake -DTOOLS=1 -DSERVERS=0`, see TrinityCore's install guide):
   `mapextractor`, `vmap4extractor`, `vmap4assembler`, `mmaps_generator`.
2. In the client folder: `mapextractor` (writes `maps/`), `vmap4extractor` then `vmap4assembler Buildings vmaps`
   (writes `vmaps/`). If an extractor fails on a DB2 file, the client build changed its layout: see
   [DB2 layouts](#db2-layouts).
3. Either:
   - **Our exporter on TrinityCore's extracted data** (recommended for bots): `AmeisenNavigation.Exporter --tc-data
     <folder with maps/ and vmaps/> --output <meshes>` builds `.anp` navmeshes with everything the MPQ export has:
     fences from 1.05 yards on block (climb 1.0, fine height cells), jump and drop links, WMO collision faces like
     the client, game objects (`--gameobjects`, display ids resolved through `vmaps/GameObjectModels.dtree`), water,
     ocean, lava and slime areas. Faction and city areas need AreaTable as CSV (`--area-table`, e.g. the CSV export
     of wago.tools or any DB2 tool for the client build: columns `ID`, `ParentAreaID`, `Flags[0]`,
     `FactionGroupMask`; `Flags` 0x08 marks the capitals). WMO liquid kinds come from `--liquid-types` (`ID`,
     `SoundBank`), without it the classic ids are used (most WMO liquids) and other ids are water. Roads aren't in
     TrinityCore's files: no road areas. Then `bUseAnpFiles=1` on the server.
   - **TrinityCore's own navmeshes**: run `mmaps_generator` too and point the server at the `mmaps` folder
     (`sMmapsPath`); `iMmapFormat=0` detects TrinityCore master (MMAP v16) from the magic in front of the `.mmap`
     files (or set `3`). Child maps (phases) without their own files can use their parent's with `sParentMaps`.
     These navmeshes are coarser for bots: TrinityCore's generator climbs 6 cells (~1.6 yards, it walks over most
     fences) and has no jump links.

`--tc-data` reads the files of TrinityCore master as of October 2026 (`.map` version 10, vmaps `VMAP_4.E`); older
extractor output is refused with a message. Tested on synthetic files written like the extractors do
(`tests/E2ETcDataTests.cpp`), not on a real client yet: see [Checking a real client](#checking-a-real-client).

### DB2 layouts

The extractors read `Map.db2`, `LiquidType.db2`, `LiquidMaterial.db2`, `LiquidObject.db2`, `CinematicCamera.db2` and
`GameObjectDisplayInfo.db2` with layouts compiled into `src/tools/extractor_common/ExtractorDB2LoadInfo.h`. A client
build that adds a field changes the layout hash and the field count, the extractor then refuses the file. Example from
build 70245 (classic): `Map.db2` got `OceanLiquidTypeID` before `NavigationMaxDistance`, 25 -> 26 fields, layout hash
`0x75863E23` -> `0xD43AFAC3`.

`anav_db2info` tells which tables changed: export the DB2 files from the client with a CASC tool (CASCExplorer,
wow.tools.local) and run

```
anav_db2info Map.db2 LiquidType.db2 ... --loadinfo TrinityCore/src/tools/extractor_common/ExtractorDB2LoadInfo.h
```

It prints the layout hash and field count of each file and `MISMATCH` with the values to put in the LoadInfo when the
extractors expect another layout (exit code 2). The new field list (names, types, array sizes, order) comes from the
WoWDBDefs definition of the table for the build (`Map.dbd`, github.com/wowdev/WoWDBDefs); rebuild the tools after
the change.

### Checking a real client

What to look at after the first export of a recent client (none was available while writing this):

1. `--list-maps --tc-data <folder>` lists the maps with files. Export one small map first (`-m 1` Kalimdor or a
   starting zone with `-t x,y`), with `--debug` for the area images.
2. The log: no "Invalid or old .map file", few "model spawns without a readable .vmo file", game objects added.
3. `anav_navmesh2ply <meshes> <mapId> out.ply --anp --box ...` around a town: walls and fences cut the navmesh,
   buildings are hollow where doors are, water (`--area`) where the lakes are, jump links (yellow) over low fences.
   If buildings or trees are mirrored or turned, the vmap transform is wrong (`TcData::SpawnToWow`); if game objects
   are, it's `GameObjects::SpawnTransform`.
4. In game: paths from the server around a few fences, through a city gate, along a pier, and a fence spawned
   by the server (game object) - the bot shouldn't run into anything it can't step over.
5. Compare with TrinityCore's own navmeshes of the same data (server with `iMmapFormat=3`): ours should have the same
   walkable areas, minus fences and plus jump links.

## Protocol

Every message (both directions) is framed as `int32 size | uint8 type | payload[size - 1]` (little endian). Every
request gets exactly one response with the same type. Positions are WoW coordinates (`x`, `y`, `z` floats).

| Type | Request | Response |
| --- | --- | --- |
| `0` PATH | `int mapId, int flags, Vector3 start, Vector3 end` | `Vector3[]` (single zero vector on failure) |
| `1` MOVE_ALONG_SURFACE | `int mapId, Vector3 start, Vector3 end` | `Vector3` reachable position (zero on failure) |
| `2` RANDOM_POINT | `int mapId` | `Vector3` |
| `3` RANDOM_POINT_AROUND | `int mapId, Vector3 center, float radius` | `Vector3` |
| `4` CAST_RAY | `int mapId, Vector3 start, Vector3 end` | `Vector3`: end if clear, zero if blocked |
| `5` RANDOM_PATH | same as PATH | `Vector3[]` with randomized corners |
| `6` EXPLORE_POLY | `int mapId, int flags, Vector3 start, float spacing, int count, Vector3 polygon[count]` | `Vector3[]` route (single zero vector on failure) |
| `7` CONFIGURE_FILTER | `uint8 state, pad[3], int count, {uint8 area, pad[3], float cost}[count]` | `bool` |
| `8` GET_HEIGHT | `int mapId, Vector3 position` | `Vector3` on the mesh (zero on failure) |
| `9` GET_CONFIG | - | `int mmapFormat, int useAnp, int pathLength, char path[]`, then `int protocolVersion, int maxPointPath, int versionLength, char version[]` |
| `10` CAST_RAY_EX | `int mapId, Vector3 start, Vector3 end` | `int hit (1/0/-1 error), float t, Vector3 position, Vector3 normal` |
| `11` SET_AVOID_ZONES | `int mapId, int count, {Vector3 center, float radius, float cost}[count]` (count 0 = none) | `bool` |
| `12` STEER | `int mapId, Vector3 position, Vector3 velocity, Vector3 target, float maxSpeed, float radius, float deltaTime, int count, {Vector3 position, Vector3 velocity, float radius}[count]` | `int ok, Vector3 velocity, Vector3 position` |
| `13` REPORT_STUCK | `int mapId, Vector3 position, float radius` (`<= 0` = default) | `bool` |
| `14` PATH_EX | same as PATH | `int count, int resultFlags (1 = partial, 2 = surface info, 4 = unreachable, 8 = search limit, 16 = link ids), Vector3 points[count], uint8 pointFlags[count]` + with result flag `2`: `uint8 areas[count], uint8 pointInfo[count]` + with `16`: `uint32 linkIds[count]` (count 0 on failure) |
| `15` LINE_OF_SIGHT | `int mapId, int ignore, Vector3 start, Vector3 end` | `int result (1 hit, 0 clear, -1 no static world), float t, Vector3 position, Vector3 normal, uint32 source, int area` |
| `16` SURFACES | `int mapId, int ignore, float x, float y, float zTop, float zBottom` | `int count (-1 no static world), uint32 areaId, {float z, Vector3 normal, uint32 source, uint8 area, uint8 flags, uint16}[count]` |
| `17` SWEEP | `int mapId, int ignore, Vector3 start, Vector3 end, float radius, float height, float stepHeight` | same as LINE_OF_SIGHT, `position` = where the body stops |
| `18` SET_OBJECTS | `int mapId, int count, int removeCount, int 0, {uint64 guid, uint32 displayId, uint32 flags (1 = blocking), Vector3 position, float rotation[4], float scale}[count], uint64 removedGuids[removeCount]` | `int accepted (-1 = map can't change), int pendingTiles` |

Path flags: `1` Chaikin, `2` Catmull-Rom, `4` Bezier (one smoothing), `8` validate with closest point on poly,
`16` validate with move along surface, `32` require a complete path (fail if the end is unreachable instead of
returning a path to the closest reachable point), `64` wall margin, `128` shortcut, `256` round corners, `512` follow
(PATH and PATH_EX), `1024` random offsets (PATH_EX), `2048` clear body (static world: sweep a body along the path, go
around what it runs into), `4096` detail (dense waypoints, see below), `8192` surface info (PATH_EX), `16384` link ids (PATH_EX). Client states: `0` normal, `1` Alliance, `2` Horde, `3` dead.
Unknown message types get an empty response. The protocol version (GET_CONFIG) tells clients what is available:
`2` (1.9) EXPLORE_POLY and flag `32`, `3` (1.10) everything else. Older clients simply ignore the GET_CONFIG trailer,
older servers ignore the new flags and the tuning trailer.

**Path tuning trailer** (PATH, RANDOM_PATH, PATH_EX after the request, EXPLORE_POLY after the polygon):
`float wallMargin, float cornerRadius, int cornerPoints, float stopDistance`. Negative values use the server defaults.
`stopDistance` ends the path that far before the target, at a point that sees it over the navmesh.

**Refinement** (`64`, `128`, `256`): shortcuts skip corners where a straight line stays on the navmesh, the wall margin
moves corners out of the turn (away from the wall they bend around), round corners replace them by arcs around them
(at least the wall margin away). Every changed segment is checked with a navmesh raycast that has to arrive on the end
point's surface, a step that fails is skipped (half the distance is tried first): the result is never worse than the
plain path. Off-mesh connections split the path, their ends never move.

**Follow** (`512`): the server keeps the connection's corridor (`dtPathCorridor`) and walks it to the new start while
the start stays within 3 yards of it and the target within 10 yards of the previous one: the corners ahead stay the
same between requests and a request is cheap. Anything else searches again.

**PATH_EX point flags**: `1` start, `2` end, `4` an off-mesh connection starts here (the next point is where it ends),
then how to take it: `8` jump, `16` drop (walk off the ledge), `32` custom link. With `2048` clear body: `64` a point
added to go around an obstacle the navmesh doesn't know, `128` the body runs into something on the way to the next
point and there is no way around it nearby (jump, open the door).

**DETAIL** (`4096`, PATH, RANDOM_PATH, PATH_EX): as many useful waypoints as a bot can use, all on the navmesh
surface (the right floor on stacked geometry): a point on each side of every change of ground (water, road, building
floor, faction border), points where the height profile bends (crests, dips, stairs, bridge humps: the surface never
leaves the straight line between two waypoints by more than `fWaypointHeightError`, 0.3 yards) and never more than
`fWaypointSpacing` (4 yards) apart. Corners, start, end and off-mesh connections stay with their flags, new points have
none. The added waypoints keep `fEdgeMargin` (1 yard) from the edge of the navmesh where the passage allows it,
else they go to its middle: along a cliff, a dock or water the client forbids, the path runs a safe distance from the
drop instead of on the eroded edge (the ground changes stay exactly where the ground changes). With the static world
(`.anw`) every point's height is the true ground's instead of the navmesh's, which lies a little above it. A path
that wouldn't fit `iMaxPointPath` gets a larger spacing (the bends and ground changes stay). Optional trailer after the
tuning trailer: `float spacing, float heightError` (`<= 0` = defaults).

**Unreachable targets**: when a map loads, the server labels the connected parts ("islands") of its navmesh
(4 bytes per polygon, done again when game objects rebuild tiles). A target on another island (a roof, the inside of a
closed building, a rock in the sea) is known to be unreachable without a search: the path leads to the closest point of
the start's island near the target (searched up to 90 yards around it) instead of the best polygon a search that
exhausts its node budget happens to end on, and it takes microseconds instead of the ~14 ms such a search costs. PATH_EX
tells partial paths apart: result flag `4` unreachable (no path exists, for this filter or at all: asking again won't
help), `8` search limit (`iMaxSearchNodes` ran out, the path leads towards the target: follow it and ask again from
further on). The load log reports the islands per map, many small ones point at navmesh problems.

**Surface info** (`8192`, PATH_EX): the response also carries, per point, the navmesh area under it (ANP area id, 0 =
none) and what the ground does there: `1` the ground changes after this point (water, road, building floor, faction
border), `2` crest (up before, down after), `4` dip, `8` the slope changes (stairs, a ramp starts or ends), `16` added
by DETAIL only to keep the spacing. Crests, dips, slope changes and spacing points come from DETAIL, the areas and
ground changes are there for every path. Result flag `2` says they are present: not after the legacy smoothings (`1`,
`2`, `4`), which move the points.

**Static world** (protocol 4): `{mapId:03}.anw` files written by the exporter next to the `.anp` (`sWorldPath`
to keep them elsewhere, `--no-world` to skip them). Tiles are read when a query needs them and kept per map
(`iWorldCacheTiles`, an ADT of terrain takes ~2.7 MB in memory with its bounding volume hierarchy, buildings add to
it). `source` is `kind << 28 | id`: kind `0` terrain, `1`
WMO (placement unique id), `2` doodad (M2 placed by the map, unique id), `3` liquid, `4` game object added at export
(display id). `ignore` flags: `1` terrain, `2` WMOs, `4` doodads, `8` liquids, `16` game objects; liquids never block
LINE_OF_SIGHT. SURFACES flags: `1` standable (walkable, not too steep), `2` liquid, at most 64 surfaces highest first,
`areaId` is the AreaTable id of the chunk (zone/subzone). SWEEP approximates the body with rays at three heights
(above `stepHeight`, middle, head) along the center and both sides, its front included: it catches what the navmesh
rounds away (posts, branches at head height) but not a thin horizontal deck between the rays. Segments are limited
to 1000 yards.

**Game objects at runtime** (SET_OBJECTS): doors, gates, the platforms of elevators, chests... as the bot's object
manager sees them (guid, display id, position, rotation quaternion, scale, blocking = closed). Their collision models
come from `gameobjects.ano` (written by the exporter for every display of GameObjectDisplayInfo.dbc, or of
TrinityCore's `GameObjectModels.dtree` with `--tc-data`). LINE_OF_SIGHT, SURFACES and SWEEP see them at once; the
navmesh tiles around them are rebuilt in the background with the export's settings and areas (stored in the `.anw`)
and swapped in while requests on the map wait (~50 ms for a test tile, about a second for a continent ADT): paths
see them once `pendingTiles` is 0. Objects are world state shared by all clients (up to 4096 per map); an object
sent again unchanged costs nothing, a blocking flag of 0 keeps it but out of the way (an open door). Rebuilt tiles keep
the export's jump/drop links and the `sOffMeshFile` connections, not the automatic drop links.

**Off-mesh connections.** Area ids `40` jump, `41` drop, `42` link, polygon flags `64` jump, `128` drop, the same for
ANP and MMAP navmeshes. Jumps are excluded by the default filters (a bot has to jump there): enable them with a
CONFIGURE_FILTER cost for area `40`. `sOffMeshFile` lines are `mapId tileX,tileY (x y z) (x y z) radius [area [flags
[id]]]` (WoW coordinates, bidirectional, the tile is ignored). The id (1 to 16777215) names the link for the bot: with
path flag `16384` PATH_EX returns it on the point that leaves by the link (`uint32` per point, 0 elsewhere), so the bot
knows which script to run there (an elevator, a boat, a portal) instead of guessing from the position. Automatic drop links (`fAutoDropHeight`) only see the navmesh:
a ledge can't be told apart from a wall with ground behind it (an upper floor next to the street), keep the reach
small. Exports with geometry checks are better.

**Avoid zones** cost more to cross (cost multiplier, they apply to surfaces within max(radius, 5) of the center
height). The search prefers other ways, segments that still cross a zone are led around it. Zones are per connection
and map (64 max). **Stuck reports** add a zone for every client on the map (repeated reports at a spot raise its cost),
kept in `sAvoidZonesFile` if set.

**CONFIGURE_FILTER costs**: `0` forbids an area, a cost above 0 also allows an area the server excludes by default
(steep slopes on TrinityCore MMAPs, jumps). TrinityCore areas: `11` ground, `10` steep, `9` water, `8` magma/slime.
SkyFire: `1` ground, `2` magma, `4` slime, `8` water.

**STEER** returns the velocity for one tick towards the target (along the navmesh path, slowing down on arrival) that
avoids up to 32 units, the avoid zones and the navmesh borders (Detour's obstacle avoidance), and where that velocity
leads on the navmesh after `deltaTime`.

**EXPLORE_POLY** places waypoints on a hexagonal grid (`spacing` apart, e.g. twice the sight or gather range) inside
the polygon (x/y outline, max 256 points, its z selects the floor on stacked geometry), snaps them to the navmesh,
orders them into a short tour from `start` (nearest neighbour + 2-opt) and connects them with navmesh paths.
Unreachable waypoints are skipped, the spacing grows if more than 1024 waypoints would be needed and the route is cut
at `iMaxPointPath` points.

### C# client

```csharp
using var nav = new AmeisenNavClient("127.0.0.1", 47110);
nav.TryConnect();
nav.SetClientState(ClientState.NormalAlliance);
nav.SetCanSwim(false);                         // water forbidden (cost 0)
nav.SetTerrainCost(TerrainType.Jump, 2f);      // this bot jumps where PathEx says
nav.ApplyFilter();

Vector3[]? path = nav.GetPath(0, start, end, PathFlags.SmoothCatmullRom | PathFlags.ValidateMoveAlongSurface);
Vector3[]? complete = nav.GetPath(0, start, end, PathFlags.RequireComplete); // null if unreachable
Vector3[]? fluid = nav.GetPath(0, start, end, PathFlags.Fluid, new PathTuning(StopDistance: 5f));
Vector3[]? chase = nav.GetPath(0, position, target, PathFlags.Follow);     // stable corners while walking
Vector3[]? dense = nav.GetPath(0, start, end, PathFlags.Fluid | PathFlags.Detail,
                               new PathTuning(WaypointSpacing: 3f));      // waypoints every <= 3 yards, on the ground

PathResult? detailed = nav.GetPathEx(0, start, end);
// detailed.Flags[i].HasFlag(PathPointFlags.Jump): jump from Points[i] to Points[i + 1]
PathResult? ground = nav.GetPathEx(0, start, end, PathFlags.Detail | PathFlags.SurfaceInfo);
// ground.Areas[i] (AnpArea), ground.Info[i].HasFlag(PathPointInfo.SurfaceChange): water/road/... starts after Points[i]

nav.SetAvoidZones(0, [new AvoidZone(dangerousMob, 20f, cost: 10f)]);      // resent after a reconnect
SteerResult step = nav.Steer(0, position, velocity, nextWaypoint, maxSpeed: 7f, units: nearbyPlayers);
nav.ReportStuck(0, position);                  // every client avoids this spot from now on

RaycastHit hit = nav.CastRayEx(0, position, target);
WorldHit view = nav.LineOfSight(0, eyes, targetEyes, WorldIgnore.Doodads); // view.Source: Wmo 1234, Terrain...
WorldSurfaces? floors = nav.GetSurfaces(0, x, y, z + 50f, z - 50f);       // decks, bridges, water, AreaId
WorldHit bump = nav.Sweep(0, position, nextWaypoint, radius: 0.5f, height: 2f);
nav.SetObjects(0, [new WorldObject(door.Guid, door.DisplayId, door.Position, door.Rotation, Blocking: !door.IsOpen)]);
Vector3[]? route = nav.ExplorePolygon(0, position, grindArea, spacing: 40f);
ServerConfig? config = nav.GetConfig(); // ProtocolVersion, MaxPointPath, ServerVersion, ...
```

## Deployment 🐳

**Docker**: one image for the server and the exporter (`Dockerfile`, `deploy/docker-compose.yml`). The server
listens on `0.0.0.0:47110` and serves ANP navmeshes from `/meshes`; mount a config file at
`/etc/ameisennav/config.cfg` and/or set `ANAV_<key>` variables.

```bash
docker build -t ameisennav .
docker run -d --name ameisennav -p 47110:47110 -v /srv/meshes:/meshes:ro ameisennav
docker run -d -p 47110:47110 -v /srv/mmaps:/meshes:ro -e ANAV_bUseAnpFileFormat=0 ameisennav   # MMAPs

# Export navmeshes with the same image
docker run --rm -v /srv/wow:/wow:ro -v /srv/meshes:/meshes ameisennav exporter -w /wow -o /meshes -m 0,1 -s
```

Tagged releases (`v*`) also push the image to `ghcr.io/jnnshschl/ameisennavigation`.

**systemd**: `deploy/ameisennav.service` runs the server as a sandboxed dynamic user:

```bash
sudo cmake --install build --prefix /usr/local
sudo install -Dm644 deploy/ameisennav.service /etc/systemd/system/ameisennav.service
sudo install -Dm644 config.cfg /etc/ameisennav/config.cfg
sudo systemctl daemon-reload && sudo systemctl enable --now ameisennav
```

## Performance ⚡

`anav_benchmark 2 20000` (synthetic 2x2 ADT map at production resolution, 4 vCPU cloud VM, GCC 13 Release):

| | |
| --- | --- |
| Navmesh build | 0.62 s per ADT tile (32x32 sub-tiles, parallel) |
| Load `.anp` (4 tiles) | 49 ms |
| Path queries, 1 thread | 3,400 paths/s (~290 us for paths across ADTs) |
| Path queries, 4 threads | 13,000 paths/s (linear scaling, queries are pooled per map) |

## Robustness and security 🛡️

The server is meant to run next to untrusted bot processes and loads mesh files it didn't create:

- **Requests**: positions must be finite and within +-100000 yards, radii <= 5000, explore polygons <= 10000 yards
  and 256 points, map ids 0..65535, area costs 0 < cost <= 1e6. Malformed requests get a zero/empty answer and are
  only logged at debug level. Every request gets exactly one response.
- **Resources**: `dtNavMeshQuery` objects (A* node pools, ~2.7 MB each) are pooled per map and leased per request,
  memory follows the number of concurrently running requests, not clients x maps. `iMaxClients` caps threads,
  TCP keepalive frees threads of vanished peers.
- **Mesh files**: every Detour tile (ANP and MMAP) is validated before Detour sees it: sizes, every vertex/polygon/
  neighbour/detail/BV tree/off-mesh index, finite coordinates, BV quantization range. Navmesh parameters, tile
  sizes and the total decompressed size of an `.anp` are bounded. Broken tiles are skipped and counted.
- **Exporter input**: corrupt client files can't produce out of bounds reads, endless loops or NaN geometry for
  Recast (non-finite/absurd triangles and area rects are dropped).
- **Verification**: unit, end-to-end (tile pipeline, TCP server, full exports of synthetic client data) and C# client
  integration tests run with GCC, Clang, MSVC, ASan+UBSan (incl. float-cast-overflow) and ThreadSanitizer.
  libFuzzer targets cover tile validation + Detour queries, `.anp` loading, the exporter's ADT/WMO/M2/WDT/DBC
  parsers and the request handlers; inputs that ever crashed are kept in `tests/fuzz/regressions`. Fuzzing found
  (and this repo fixes) a null dereference in Detour's `closestPointOnDetailEdges` for tiles without detail edge
  flags, an endless `raycast` on inconsistent neighbour links, UB in miniz and an unbounded MH2O instance loop.
- **Binaries** are built with stack protector, `_FORTIFY_SOURCE=3`, stack clash protection, CET (IBT/SHSTK), PIE and
  full RELRO (`/guard:cf` + `/CETCOMPAT` with MSVC), the Docker image runs as an unprivileged user and the systemd
  unit in a sandbox.

```bash
# Fuzzing (Clang)
CC=clang CXX=clang++ cmake -S . -B build-fuzz -DANAV_BUILD_FUZZERS=ON -DANAV_BUILD_TESTS=OFF -DANAV_ENABLE_OPENMP=OFF
cmake --build build-fuzz
tests/fuzz/run_fuzzers.sh build-fuzz 300   # seconds per target
```

## Building 🛠️

Requirements: a C++20 compiler (MSVC 2022+, GCC 13+, Clang 17+), CMake 3.21+. The .NET projects need the .NET 10 SDK.

```bash
# Linux / Windows (CMake)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release        # unit, end-to-end and exporter tests
tests/run_client_integration.sh build/bin/AmeisenNavigation.Server   # C# client against the real server
```

On Windows the Visual Studio solution (`AmeisenNavigation.sln`) works as well. CMake uses the prebuilt StormLib
from `dep/` with MSVC and fetches StormLib from GitHub everywhere else (`-DANAV_USE_BUNDLED_STORMLIB=OFF` forces that).

| CMake option | Default | |
| --- | --- | --- |
| `ANAV_BUILD_SERVER` / `ANAV_BUILD_EXPORTER` / `ANAV_BUILD_TESTS` | `ON` | Select targets |
| `ANAV_ENABLE_OPENMP` / `ANAV_ENABLE_LTO` | `ON` | Parallel loading/building, link time optimization |
| `ANAV_SANITIZE` | `OFF` | AddressSanitizer + UndefinedBehaviorSanitizer (GCC/Clang) |
| `ANAV_WARNINGS_AS_ERRORS` | `OFF` | `-Werror` / `/WX` for project code |
| `ANAV_HARDENING` | `ON` | Stack protector, FORTIFY_SOURCE, stack clash protection, CET, PIE, full RELRO / CFG |
| `ANAV_BUILD_FUZZERS` | `OFF` | libFuzzer targets (Clang only, instruments everything) |

`anav_benchmark [adtsPerAxis] [queries]` builds a synthetic map at production resolution and measures build and
query throughput.

### Looking at a navmesh

`anav_navmesh2ply <meshes> <mapId> <out.ply> [--anp] [--format n] [--box x0,y0,x1,y1] [--area]` writes a map's
navmesh (ANP with `--anp`, else MMAPs, `--format` like `iMmapFormat`) as a binary PLY file in WoW coordinates for
MeshLab, Blender or CloudCompare. Triangles are colored by slope (blue flat to red at 60 degrees) or by area with
`--area` (ground, road, city, structures, liquids; Alliance areas bluer, Horde areas redder). Off-mesh connections are
narrow arched ribbons: jumps yellow, drops magenta, other links white. `--box` keeps the triangles around a spot (WoW
x/y): holes where a fence stands, a link over it, a gap the navmesh shouldn't cross.

## Project layout

| Folder | |
| --- | --- |
| `AmeisenNavigation/` | Core library: nav sources (ANP, MMAP), query filters, path smoothing, per client state |
| `AmeisenNavigation.Pack/` | ANP file format (miniz), area ids and Detour helpers shared by server and exporter |
| `AmeisenNavigation.Server/` | Server executable: config, protocol, request handlers |
| `AnTCP.Server/` / `AnTCP.Client/` | Minimal cross-platform request/response TCP layer |
| `AmeisenNavigation.Exporter/` | MPQ/ADT/WMO/M2 parsing and the tile build pipeline |
| `AmeisenNavigation.Client/` | C# client library |
| `AmeisenNavigation.Tester/` | WPF tool to visualize maps, navmeshes and paths |
| `tests/` | Unit and end-to-end tests: a synthetic world through the real tile pipeline and TCP server, full exports of synthetic client data (MPQ, DBC, WDT, ADT, WMO, M2), C# client integration, fuzzers (`tests/fuzz`) |
| `deploy/` | Docker entrypoint and compose file, systemd unit |

## Credits 🙌

- TrinityCore - [GitHub](https://github.com/TrinityCore/TrinityCore)
- recastnavigation - [GitHub](https://github.com/recastnavigation/recastnavigation)
- StormLib - [GitHub](https://github.com/ladislav-zezula/StormLib)
- miniz - [GitHub](https://github.com/richgel999/miniz)
