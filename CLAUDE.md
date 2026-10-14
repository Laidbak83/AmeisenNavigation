# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Navmesh TCP server and navmesh exporter for World of Warcraft bots (C++20, Recast/Detour), plus a C# client.
README.md has the user facing details (config keys, protocol table, deployment).

## Commands

```bash
# Build + test (Linux/Windows). StormLib is fetched from GitHub on non-MSVC builds; offline:
#   -DFETCHCONTENT_SOURCE_DIR_STORMLIB=<local StormLib checkout>
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DANAV_WARNINGS_AS_ERRORS=ON
cmake --build build
ctest --test-dir build --output-on-failure        # unit, e2e, exporter

# One test binary / test cases whose name contains a substring
build/bin/anav_unit_tests                          # pure logic
build/bin/anav_e2e_tests Navigation_Explore        # synthetic world, pipeline, TCP server
build/bin/anav_exporter_tests Exporter_Mh2o        # full exports of synthetic client data

# Sanitizers (CI runs all of these)
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DANAV_SANITIZE=ON          # ASan+UBSan+float-cast-overflow
cmake -S . -B build-tsan -DANAV_ENABLE_OPENMP=OFF -DANAV_BUILD_EXPORTER=OFF \
      -DCMAKE_CXX_FLAGS=-fsanitize=thread -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread

# Fuzzing (Clang + libclang-rt-dev)
CC=clang CXX=clang++ cmake -S . -B build-fuzz -DANAV_BUILD_FUZZERS=ON -DANAV_BUILD_TESTS=OFF -DANAV_ENABLE_OPENMP=OFF
cmake --build build-fuzz && tests/fuzz/run_fuzzers.sh build-fuzz 60

# C#
dotnet build AmeisenNavigation.Client/AmeisenNavigation.Client.csproj -c Release -warnaserror
tests/run_client_integration.sh build/bin/AmeisenNavigation.Server   # needs the e2e test world (run ctest first)

anav_benchmark [adtsPerAxis] [queries]             # manual build/query throughput benchmark
anav_navmesh2ply <meshes> <mapId> <out.ply> [--anp] [--box x0,y0,x1,y1] [--area]   # navmesh as colored PLY
anav_db2info <file.db2>... [--loadinfo ExtractorDB2LoadInfo.h]   # why TrinityCore's extractors refuse a client build
```

Formatting: `.clang-format` (4 spaces, 120 columns, braces on their own line). CI: `.github/workflows/ci.yml`.

## Architecture

CMake targets, bottom up: `anav_recast` (vendored Recast/Detour) → `anav_pack` (ANP file format, miniz,
`DetourUtils.hpp`) → `anav_core` (navigation engine) → `anav_server_lib` (+ `antcp_server`) → server executable.
Exporter: `anav_exporter_core` (header-only WoW parsers + tile builder) → `anav_exporter_lib` (`Exporter.cpp`,
`RunExport`) → CLI. The Visual Studio solution builds the same code: add new files to the `.vcxproj` and
`.vcxproj.filters` too. The version lives only in `AmeisenNavigation.Pack/src/Version.hpp` (CMake parses it).

**Coordinates.** The protocol and the exporter's WoW structures use WoW coordinates (x north, y west, z up),
Recast/Detour use RD coordinates: RD = (wowY, wowZ, wowX) (`Vector3::ToRDCoords/ToWowCoords`). Tests build RD
positions with `TestWorld::Wow(rdX, rdY, rdZ)`.

**Request flow (server).** `AnTcpServer` (one thread per connection; frame = `int32 size | uint8 type | payload`)
→ `AnTcpServer::Dispatch` → `NavServer::Handle*` (`Protocol.hpp` structs copied with memcpy; every request gets
exactly one response, a zero vector on failure) → `AmeisenNavigation` methods → `GetQueryContext` (looks up the
client, leases a `dtNavMeshQuery` from the map's `NavMeshQueryPool`; map ids are bounded in `NavMeshCache`) → Detour.
Every position must pass `IsValidPosition` before reaching Detour (its float→int tile math is UB for NaN/huge
values); `FindNearestPoly` and `ToValidRdCoords` enforce it. `AmeisenNavClient` holds per-connection state (query filter, avoid zones,
follow corridor, path buffers) and is used by one thread at a time. `Protocol.hpp` must stay byte-compatible with
`AmeisenNavigation.Client/WireFormat.cs` (static_asserts; `WireSizes` is checked by the C# integration test); bump
`PROTOCOL_VERSION` when adding or extending messages, extensions are optional trailers so old clients keep working.

**Engine files.** `AmeisenNavigation.cpp` (queries, `CalculateNormalPath`, `Raycast` which casts from every polygon
touching the start), `PathRefinement.cpp` (shortcut, wall margin, arcs, stop distance, follow corridor; every changed
segment goes through `SegmentClear`, a step that fails is skipped), `Avoidance.cpp` (avoid zones, stuck reports,
steering). Paths with off-mesh connections are refined in pieces between them (`RefineAll`, PathPointFlags): their
ends never move. Client filters are `NavQueryFilter` (vendored Detour has `DT_VIRTUAL_QUERYFILTER` enabled): area
exclusion (cost 0), allowed areas (cost > 0 bypasses the flag exclusion), avoid zone costs (grid when many).
`TileRebuilder` turns loaded tiles back into `dtNavMeshCreateParams` to add off-mesh connections after loading (nav
sources run a post-load hook: `sOffMeshFile`, automatic drops). Off-mesh area ids 40/41/42 and flags
`NAV_JUMP`/`NAV_DROP` (`AnpFormat.hpp`) are shared by ANP and MMAP navmeshes; jumps are excluded by default.

**Static world.** The exporter also writes `{mapId:03}.anw` (`AnpWorld.hpp`, `Processors/WorldBuilder.hpp`): the
triangles the navmesh was built from, per ADT, quantized, with a nav area and a source (`SourceKind << 28 | id`, from
`Structure::triSources`; tag new geometry with `TagSources` right after adding it) and per chunk AreaTable ids. The
server reads tiles on demand (`World/StaticWorld.cpp`: LRU per map, BVH per tile) for LINE_OF_SIGHT, SURFACES and
SWEEP; positions go through `AnpWorld::AdtOf`/`IsFinite` checks, segments are capped at 1000 yards. Game objects at
runtime (`AmeisenNavigation.Server/src/World/DynamicWorld.cpp`, SET_OBJECTS): models from `gameobjects.ano`, meshes
published to `StaticWorld::SetDynamicMeshes`, dirty ADTs rebuilt on a worker thread with the exporter's
`AdtTileProcessor` (the server lib links `anav_exporter_core`; it must not include StormLib headers such as
`Wow/GameObjects.hpp`, use `Wow/GameObjectSpawn.hpp`) from the `.anw` tiles + "build" + "r_XX_YY" entries, then
`AmeisenNavigation::ReplaceTiles` under the map's `NavMeshLock` (requests hold it shared through `QueryContext`).

**Nav sources.** `AnpNavSource` (own format: zip with `mapId`, `params` and one Detour tile per `XX_YY` entry) and
`MmapNavSource` (TrinityCore/SkyFire `.mmap`/`.mmtile`, format detected from file names, TrinityCore master's MMAP v16
from the magic in front of the `.mmap`; child maps can fall back to their parent's files). Maps load lazily once
(`NavMeshCache`). Every tile goes through `ValidateTileData` (all indices, finite coordinates, BV quantization range,
tile coordinates) before `dtNavMesh::addTile` — Detour itself trusts tile data completely. ANP area ids
(`AnpFormat.hpp`) come in {neutral, Alliance, Horde} triplets and are wire/file format: never renumber them.
Default filters per `ClientState` come from `IQueryFilterProvider`; `CONFIGURE_FILTER` overrides costs per client.

**Exporter pipeline.** MPQs in client load order (`FileSort.hpp`) → `CachedFileReader` → per ADT
`AdtChunkExtractor` (terrain, MH2O/MCLQ liquids, roads from textures, city/faction areas from AreaTable, WMO/M2
placements deduplicated by uniqueId, never clipped) → `Structure::Clean` (drops garbage geometry) →
`AdtTileProcessor` (one Detour tile per ADT, triangles bucketed per tile and sub-tile, Recast per sub-tile, merged,
rebuilt coarser when Detour's 65535 vertex limit is hit) → `AnpWriter`. The Detour tile grid is aligned to the ADT
grid (tile x = maxAdtX - adtX). WMO-only maps (dungeons) come from the WDT's global WMO. Recent (CASC) clients go
through TrinityCore's extractors: `--tc-data` (`Wow/TcData.hpp`) reads their `.map` files and vmaps into the same
`MapData` and `BuildMap` (vmap spawn space: WoW x = 32 * TILESIZE - pos.x, M2 `.vmo` triangles are stored flipped).

**Vendored code.** `recastnavigation/` and `AmeisenNavigation.Pack/src/miniz` carry local fixes marked with
`AmeisenNavigation:` comments (null deref in `closestPointOnDetailEdges`, raycast iteration cap, miniz UB,
`DT_VIRTUAL_QUERYFILTER` enabled); keep them when updating upstream. Detour's `dtLink` array is only 4 byte aligned by format, so UBSan alignment checks are off
for `anav_recast`.

## Tests

In-repo framework (`tests/TestFramework.hpp`: `TEST_CASE`, `CHECK`, `REQUIRE`, `CHECK_EQ`, `CHECK_NEAR`).
`tests/TestWorld.hpp` exports a synthetic two-ADT world (wall, platform island, water, road, faction area) through
the real tile pipeline once per process into `<tmp>/anav_tests/world`. `tests/WowDataBuilder.hpp` writes synthetic
DBC/WDT/ADT/WMO/M2 files into MPQs for full exporter runs. Recast rounds span heights up, navmesh heights lie up to
~2 cell heights above the source surface: compare heights with a one-sided tolerance, not `CHECK_NEAR(h, exact, small)`.
Inputs that ever crashed a fuzzer live in `tests/fuzz/regressions/<target>/` and are replayed by `run_fuzzers.sh`.

## Gotchas

- Clang with OpenMP can't capture structured bindings in lambdas (compile error): copy to named variables first.
- `ANAV_HARDENING` (default on) adds `-D_FORTIFY_SOURCE=3`, stack protector, CET, full RELRO; it is skipped for
  sanitizer/fuzzer builds.
- Server config: `key=value` file plus `ANAV_<key>` environment overrides (`Config.hpp`); the server re-saves the
  file on start to add new keys.
