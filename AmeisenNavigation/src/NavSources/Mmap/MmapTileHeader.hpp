#pragma once

#include <cstdint>

#include <DetourNavMesh.h>

constexpr uint32_t MMAP_MAGIC = 0x4D4D4150; // 'MMAP'
constexpr uint32_t MMAP_VERSION = 15;       // oldest tile version accepted

/// First MMAP version whose .mmap files start with MmapNavMeshHeader instead of the raw dtNavMeshParams.
constexpr uint32_t MMAP_VERSION_NAVMESH_HEADER = 16;

/// Header in front of every TrinityCore/SkyFire .mmtile file.
struct MmapTileHeader
{
    uint32_t mmapMagic;
    uint32_t dtVersion;
    uint32_t mmapVersion;
    uint32_t size;
    char usesLiquids;
    char padding[3];
};

static_assert(sizeof(MmapTileHeader) == 20, "MmapTileHeader must match the on-disk layout");

/// .mmap file of TrinityCore master (MMAP v16): this header, then offmeshConnectionCount MmapOffMeshData. The
/// generator already baked the connections into the tiles, the list is informational.
struct MmapNavMeshHeader
{
    uint32_t mmapMagic;
    uint32_t mmapVersion;
    dtNavMeshParams params;
    uint32_t offmeshConnectionCount;
};

static_assert(sizeof(MmapNavMeshHeader) == 40, "MmapNavMeshHeader must match the on-disk layout");

/// TrinityCore's OffMeshData (positions in WoW coordinates).
struct MmapOffMeshData
{
    uint32_t mapId;
    uint32_t tileX;
    uint32_t tileY;
    float from[3];
    float to[3];
    float radius;
    uint8_t connectionFlags; // 1 = bidirectional
    uint8_t areaId;
    uint16_t flags;
};

static_assert(sizeof(MmapOffMeshData) == 44, "MmapOffMeshData must match the on-disk layout");
