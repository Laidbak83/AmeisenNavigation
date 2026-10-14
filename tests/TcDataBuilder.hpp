#pragma once

// Synthetic files in the formats of TrinityCore master's extractors (maps/*.map, vmaps/*.vmo, *.vmtile,
// GameObjectModels.dtree), written like mapextractor and vmap4assembler do. Used by the TrinityCore data tests and the
// fuzzer seed corpus.

#include <algorithm>
#include <cstring>
#include <functional>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "Wow/TcData.hpp"
#include "WowDataBuilder.hpp"

namespace TcDataBuilder {
using WowData::Bytes;
using WowData::PutBytes;
using namespace TcData;

/// WoW position of a square corner of ADT (adtX, adtY).
inline float RowX(int adtY, float row) { return (32 - adtY) * TILESIZE - row * UNITSIZE; }
inline float ColumnY(int adtX, float column) { return (32 - adtX) * TILESIZE - column * UNITSIZE; }

template <typename T>
inline void PutRaw(Bytes& out, const T& value)
{
    const auto* p = reinterpret_cast<const unsigned char*>(&value);
    out.insert(out.end(), p, p + sizeof(T));
}

template <typename T>
inline void PutVector(Bytes& out, const std::vector<T>& values)
{
    for (const T& v : values)
    {
        PutRaw(out, v);
    }
}

/// A .map file like mapextractor writes it (areas, heights, liquid, holes).
inline Bytes MakeMapFile(int adtX, int adtY, const std::function<float(float, float)>& height, bool packed,
                         const std::vector<std::pair<int, int>>& waterCells, std::pair<int, int> deepCell,
                         const std::vector<std::tuple<int, int, uint16_t>>& areas, std::pair<int, int> holeCell)
{
    std::vector<float> v9(V9 * V9), v8(V8 * V8);

    for (int r = 0; r < V9; ++r)
        for (int c = 0; c < V9; ++c)
            v9[r * V9 + c] = height(RowX(adtY, static_cast<float>(r)), ColumnY(adtX, static_cast<float>(c)));

    for (int r = 0; r < V8; ++r)
        for (int c = 0; c < V8; ++c)
            v8[r * V8 + c] = height(RowX(adtY, r + 0.5f), ColumnY(adtX, c + 0.5f));

    Bytes areaData;
    MapAreaHeader areaHeader{MAP_AREA_MAGIC, 0, 0};
    PutRaw(areaData, areaHeader);
    uint16_t areaIds[16][16]{};

    for (const auto& [r, c, id] : areas)
        areaIds[r][c] = id;

    PutRaw(areaData, areaIds);

    Bytes heightData;
    const float minH = *std::min_element(v9.begin(), v9.end());
    const float maxH = *std::max_element(v9.begin(), v9.end());
    const float lo = std::min(minH, *std::min_element(v8.begin(), v8.end()));
    const float hi = std::max(maxH, *std::max_element(v8.begin(), v8.end()));
    MapHeightHeader heightHeader{MAP_HEIGHT_MAGIC, packed ? HEIGHT_AS_INT8 : 0u, lo, hi};
    PutRaw(heightData, heightHeader);

    if (packed)
    {
        const float step = 255.0f / (hi - lo);

        for (const float h : v9)
            PutRaw(heightData, static_cast<uint8_t>((h - lo) * step + 0.5f));
        for (const float h : v8)
            PutRaw(heightData, static_cast<uint8_t>((h - lo) * step + 0.5f));
    }
    else
    {
        PutVector(heightData, v9);
        PutVector(heightData, v8);
    }

    Bytes liquidData;

    if (!waterCells.empty())
    {
        // Squares with liquid, corner heights, then the extractor's "no liquid" height in the first corner of every
        // other square.
        std::vector<std::vector<bool>> shown(V8, std::vector<bool>(V8, false));
        std::vector<std::vector<float>> heights(V9, std::vector<float>(V9, 0.0f));
        uint8_t flags[16][16]{};
        int minRow = V8, minColumn = V8, maxRow = 0, maxColumn = 0;

        const auto addCell = [&](int cellRow, int cellColumn, uint8_t cellFlags, float level) {
            flags[cellRow][cellColumn] = cellFlags;

            for (int r = cellRow * 8; r < cellRow * 8 + 8; ++r)
            {
                for (int c = cellColumn * 8; c < cellColumn * 8 + 8; ++c)
                {
                    shown[r][c] = true;
                    heights[r][c] = heights[r + 1][c] = heights[r][c + 1] = heights[r + 1][c + 1] = level;
                    minRow = std::min(minRow, r);
                    minColumn = std::min(minColumn, c);
                    maxRow = std::max(maxRow, r);
                    maxColumn = std::max(maxColumn, c);
                }
            }
        };

        for (const auto& [r, c] : waterCells)
            addCell(r, c, LIQUID_FLAG_WATER, 8.0f);

        addCell(deepCell.first, deepCell.second, LIQUID_FLAG_OCEAN | LIQUID_FLAG_DARK_WATER, 0.0f);

        for (int r = 0; r < V8; ++r)
            for (int c = 0; c < V8; ++c)
                if (!shown[r][c])
                    heights[r][c] = INVALID_LIQUID_HEIGHT;

        MapLiquidHeader liquidHeader{MAP_LIQUID_MAGIC,
                                     0,
                                     0,
                                     0,
                                     static_cast<uint8_t>(minColumn),
                                     static_cast<uint8_t>(minRow),
                                     static_cast<uint8_t>(maxColumn - minColumn + 2),
                                     static_cast<uint8_t>(maxRow - minRow + 2),
                                     INVALID_LIQUID_HEIGHT};
        PutRaw(liquidData, liquidHeader);
        uint16_t entries[16][16]{};
        PutRaw(liquidData, entries);
        PutRaw(liquidData, flags);

        for (int r = minRow; r < minRow + liquidHeader.height; ++r)
            for (int c = minColumn; c < minColumn + liquidHeader.width; ++c)
                PutRaw(liquidData, heights[r][c]);
    }

    uint8_t holes[16][16][8]{};
    std::memset(holes[holeCell.first][holeCell.second], 0xFF, 8);

    MapFileHeader header{};
    header.mapMagic = MAP_MAGIC;
    header.versionMagic = MAP_VERSION;
    header.areaMapOffset = sizeof(header);
    header.areaMapSize = static_cast<uint32_t>(areaData.size());
    header.heightMapOffset = header.areaMapOffset + header.areaMapSize;
    header.heightMapSize = static_cast<uint32_t>(heightData.size());
    header.liquidMapOffset = liquidData.empty() ? 0 : header.heightMapOffset + header.heightMapSize;
    header.liquidMapSize = static_cast<uint32_t>(liquidData.size());
    header.holesOffset = header.heightMapOffset + header.heightMapSize + header.liquidMapSize;
    header.holesSize = sizeof(holes);

    Bytes out;
    PutRaw(out, header);
    PutBytes(out, areaData);
    PutBytes(out, heightData);
    PutBytes(out, liquidData);
    PutRaw(out, holes);
    return out;
}

inline void PutBih(Bytes& out)
{
    const float bounds[6]{};
    PutRaw(out, bounds);
    PutRaw(out, uint32_t{0}); // tree nodes
    PutRaw(out, uint32_t{0}); // objects
}

struct VmoGroup
{
    std::vector<Vector3> verts;
    std::vector<uint32_t> indices;
    std::optional<WorldModel::Liquid> liquid;
};

/// A .vmo file like vmap4assembler writes it.
inline Bytes MakeVmo(bool isM2, const std::vector<VmoGroup>& groups)
{
    Bytes out(VMAP_MAGIC, VMAP_MAGIC + 8);
    PutBytes(out, Bytes{'W', 'M', 'O', 'D'});
    PutRaw(out, uint32_t{8});
    PutRaw(out, uint32_t{isM2 ? MODEL_IS_M2 : 0u});
    PutRaw(out, uint32_t{0}); // root WMO id
    PutBytes(out, Bytes{'G', 'M', 'O', 'D'});
    PutRaw(out, static_cast<uint32_t>(groups.size()));

    for (const VmoGroup& group : groups)
    {
        const float bounds[6]{};
        PutRaw(out, bounds);
        PutRaw(out, uint32_t{0}); // MOGP flags
        PutRaw(out, uint32_t{0}); // group WMO id
        PutBytes(out, Bytes{'V', 'E', 'R', 'T'});
        PutRaw(out, static_cast<uint32_t>(4 + 12 * group.verts.size()));
        PutRaw(out, static_cast<uint32_t>(group.verts.size()));

        if (group.verts.empty())
        {
            continue;
        }

        PutVector(out, group.verts);
        PutBytes(out, Bytes{'T', 'R', 'I', 'M'});
        PutRaw(out, static_cast<uint32_t>(4 + 4 * group.indices.size()));
        PutRaw(out, static_cast<uint32_t>(group.indices.size() / 3));
        PutVector(out, group.indices);
        PutBytes(out, Bytes{'M', 'B', 'I', 'H'});
        PutBih(out);
        PutBytes(out, Bytes{'L', 'I', 'Q', 'U'});

        if (!group.liquid)
        {
            PutRaw(out, uint32_t{0});
            continue;
        }

        const auto& liquid = *group.liquid;
        PutRaw(out, static_cast<uint32_t>(24 + 4 * liquid.heights.size() + liquid.flags.size()));
        PutRaw(out, liquid.tilesX);
        PutRaw(out, liquid.tilesY);
        PutRaw(out, liquid.corner);
        PutRaw(out, liquid.type);
        PutVector(out, liquid.heights);
        PutVector(out, liquid.flags);
    }

    PutBytes(out, Bytes{'G', 'B', 'I', 'H'});
    PutBih(out);
    return out;
}

/// Box geometry with the winding the client files use (AddBox), as 32 bit indices; M2s in TrinityCore's vmaps have
/// the second and third index of each triangle swapped.
inline VmoGroup Box(Vector3 min, Vector3 max, bool tcM2Order)
{
    std::vector<uint16_t> indices;
    VmoGroup group;
    WowData::AddBox(group.verts, indices, min, max);
    group.indices.assign(indices.begin(), indices.end());

    if (tcM2Order)
    {
        for (size_t i = 0; i + 2 < group.indices.size(); i += 3)
        {
            std::swap(group.indices[i + 1], group.indices[i + 2]);
        }
    }

    return group;
}

inline Bytes MakeSpawn(uint32_t id, const Vector3& wow, const Vector3& rot, const std::string& name)
{
    Bytes out;
    PutRaw(out, uint8_t{0}); // flags (no bounds)
    PutRaw(out, uint8_t{0}); // adt id
    PutRaw(out, id);
    const float pos[3]{32.0f * TILESIZE - wow.x, 32.0f * TILESIZE - wow.y, wow.z};
    PutRaw(out, pos);
    PutRaw(out, rot);
    PutRaw(out, 1.0f);
    PutRaw(out, static_cast<uint32_t>(name.size()));
    PutBytes(out, Bytes(name.begin(), name.end()));
    return out;
}

inline Bytes MakeVmtile(const std::vector<Bytes>& spawns)
{
    Bytes out(VMAP_MAGIC, VMAP_MAGIC + 8);
    PutRaw(out, static_cast<uint32_t>(spawns.size()));

    for (const Bytes& spawn : spawns)
    {
        PutBytes(out, spawn);
    }

    return out;
}

inline Bytes MakeGameObjectModels(uint32_t displayId, const std::string& model)
{
    Bytes out(VMAP_MAGIC, VMAP_MAGIC + 8);
    PutRaw(out, displayId);
    PutRaw(out, static_cast<uint32_t>(model.size()));
    PutBytes(out, Bytes(model.begin(), model.end()));
    const float bounds[6]{};
    PutRaw(out, bounds);
    return out;
}
} // namespace TcDataBuilder
