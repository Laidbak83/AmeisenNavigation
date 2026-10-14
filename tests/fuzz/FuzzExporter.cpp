// Fuzz the exporter's client file parsers. The first byte selects the format, the rest is the file:
//   0 ADT  (terrain, liquids, roads, areas, WMO/doodad placements)
//   1 WMO  u16 root size | root | u16 group size | group | M2 for the doodads (MODN name "d.mdx")
//   2 M2   collision mesh
//   3 WDT  (incl. the global WMO of WMO-only maps)
//   4 DBC  records and strings
//   5 TrinityCore .map (terrain, liquids, holes, areas)
//   6 TrinityCore .vmo collision model (added with a rotated spawn transform)
//   7 TrinityCore .vmtile spawns
//   8 TrinityCore GameObjectModels.dtree
//   9 DB2 header (WDC3-5)
// Every extracted structure is cleaned afterwards (Clean() guards the tile builder against garbage).

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "Dbc/Db2Header.hpp"
#include "Dbc/Dbc.hpp"
#include "Mpq/CachedFileReader.hpp"
#include "Utils/CityMap.hpp"
#include "Utils/FactionMap.hpp"
#include "Utils/Logger.hpp"
#include "Utils/RoadMap.hpp"
#include "Utils/WaterMap.hpp"
#include "Wow/Adt.hpp"
#include "Wow/AdtChunkExtractor.hpp"
#include "Wow/RoadDetector.hpp"
#include "Wow/TcData.hpp"
#include "Wow/Wdt.hpp"

namespace {
struct Environment
{
    std::unique_ptr<MpqManager> mpq;
    std::unique_ptr<CachedFileReader> reader;
    std::unordered_map<unsigned int, LiquidType> liquidTypes{
        {1, LiquidType::WATER}, {2, LiquidType::OCEAN}, {3, LiquidType::MAGMA}, {4, LiquidType::SLIME}};
    std::unordered_set<unsigned int> cities{1, 2};
    std::unordered_map<unsigned int, unsigned char> factions{{1, 1}, {3, 2}};
};

Environment& Env()
{
    static Environment env = [] {
        Environment e;
        const auto empty = std::filesystem::temp_directory_path() / "anav_fuzz_no_mpq";
        std::filesystem::create_directories(empty);
        e.mpq = std::make_unique<MpqManager>(empty);
        e.reader = std::make_unique<CachedFileReader>(e.mpq.get());
        return e;
    }();
    return env;
}

/// Same layout as the wrapper classes (Adt, Wmo, M2, ...): { unsigned char* Data; unsigned int Size; }.
template <typename T>
struct View
{
    CachedFileEntry entry;

    View(const uint8_t* data, size_t size)
        : entry{const_cast<unsigned char*>(reinterpret_cast<const unsigned char*>(data)),
                static_cast<unsigned int>(size)}
    {
    }

    T* Get() { return entry.Data && entry.Size ? reinterpret_cast<T*>(&entry) : nullptr; }
};

bool TakeChunk(const uint8_t*& data, size_t& size, const uint8_t*& chunk, size_t& chunkSize)
{
    if (size < 2)
    {
        return false;
    }

    chunkSize = std::min<size_t>(static_cast<size_t>(data[0]) | (static_cast<size_t>(data[1]) << 8), size - 2);
    chunk = data + 2;
    data += 2 + chunkSize;
    size -= 2 + chunkSize;
    return true;
}

void FuzzAdt(const uint8_t* data, size_t size)
{
    // Parsers read straight from the buffer: keep an owned copy so ASan sees the exact bounds.
    std::vector<uint8_t> copy(data, data + size);
    View<Adt> view(copy.data(), copy.size());
    Adt* adt = view.Get();

    if (!adt || !adt->IsValid())
    {
        return;
    }

    auto& env = Env();
    Structure geometry;
    WaterMap water;
    RoadMap roads;
    CityMap cities;
    FactionMap factions;
    PlacementSet wmos;
    PlacementSet doodads;

    const auto roadTextures = FindRoadTextureIds(adt->Mtex());

    for (unsigned int y = 0; y < 16; ++y)
    {
        for (unsigned int x = 0; x < 16; ++x)
        {
            ExtractTerrain(adt, x, y, &geometry);
            ExtractLiquid(adt, x, y, &water, &geometry, env.liquidTypes);
            ExtractRoadCoverage(adt, x, y, &roads, roadTextures);
            ExtractCityCoverage(adt, x, y, &cities, env.cities);
            ExtractFactionCoverage(adt, x, y, &factions, env.factions);
        }
    }

    ExtractWmoGeometry(adt, *env.reader, &geometry, env.liquidTypes, &wmos);
    ExtractDoodadGeometry(adt, *env.reader, &geometry, &doodads);
    geometry.Clean();

    water.BuildSpatialIndex();
    roads.BuildSpatialIndex();
    cities.BuildSpatialIndex();
    factions.BuildSpatialIndex();
}

void FuzzWmo(const uint8_t* data, size_t size)
{
    const uint8_t* root = nullptr;
    const uint8_t* group = nullptr;
    size_t rootSize = 0;
    size_t groupSize = 0;

    if (!TakeChunk(data, size, root, rootSize) || !TakeChunk(data, size, group, groupSize))
    {
        return;
    }

    auto& env = Env();
    env.reader->Insert("f.wmo", root, rootSize);
    env.reader->Insert("f_000.wmo", group, groupSize);
    env.reader->Insert("d.m2", data, size);

    MODF::Entry entry{};
    entry.x = WORLDSIZE;
    entry.z = WORLDSIZE;
    entry.doodadSet = 1;

    Structure geometry;
    ExtractWmoPlacement(entry, "f.wmo", *env.reader, &geometry, env.liquidTypes);
    geometry.Clean();
    env.reader->Clear();
}

void FuzzM2(const uint8_t* data, size_t size)
{
    std::vector<uint8_t> copy(data, data + size);
    View<M2> view(copy.data(), copy.size());

    if (M2* m2 = view.Get())
    {
        Structure geometry;
        Matrix4x4 transform;
        AddM2Collision(m2, transform, &geometry, DOODAD);
        geometry.Clean();
    }
}

void FuzzWdt(const uint8_t* data, size_t size)
{
    std::vector<uint8_t> copy(data, data + size);
    View<Wdt> view(copy.data(), copy.size());
    Wdt* wdt = view.Get();

    if (!wdt || !wdt->IsValid())
    {
        return;
    }

    volatile unsigned int present = 0;

    for (int y = 0; y < WDT_MAP_SIZE; ++y)
    {
        for (int x = 0; x < WDT_MAP_SIZE; ++x)
        {
            present = present + (wdt->Main()->adt[y][x].exists ? 1u : 0u);
        }
    }

    Structure geometry;
    ExtractGlobalWmo(wdt, *Env().reader, &geometry, Env().liquidTypes);
}

void FuzzDbc(const uint8_t* data, size_t size)
{
    std::vector<uint8_t> copy(data, data + size);
    View<Dbc> view(copy.data(), copy.size());
    Dbc* dbc = view.Get();

    if (!dbc || !dbc->IsValid())
    {
        return;
    }

    volatile size_t sum = 0;

    for (unsigned int r = 0; r < dbc->GetRecordCount() && r < 4096; ++r)
    {
        for (unsigned int f = 0; f < dbc->GetFieldCount() && f < 64; ++f)
        {
            sum = sum + dbc->Read<unsigned int>(r, f) + std::strlen(dbc->ReadString(r, f));
        }
    }
}
} // namespace

extern "C" int LLVMFuzzerInitialize(int*, char***)
{
    Logger::SetQuiet(true);
    Env();
    return 0;
}

void FuzzTcData(int format, const uint8_t* data, size_t size)
{
    const std::vector<unsigned char> bytes(data, data + size);
    Structure structure;

    switch (format)
    {
        case 5:
            if (const auto tile = TcData::ParseMapTile(bytes, 32, 32))
            {
                WaterMap water;
                CityMap cities;
                FactionMap factions;
                TcData::AddMapTile(*tile, &structure, &water, &cities, &factions, Env().cities, Env().factions);
            }
            break;
        case 6:
            if (const auto model = TcData::ParseWorldModel(bytes))
            {
                TcData::ModelSpawn spawn;
                spawn.rot[0] = 10.0f;
                spawn.rot[1] = 100.0f;
                spawn.rot[2] = -5.0f;
                spawn.scale = 1.5f;
                const auto rotation = TcData::SpawnRotation(spawn.rot);
                TcData::AddWorldModel(
                    *model, [&](const Vector3& v) { return TcData::SpawnToWow(spawn, rotation, v); }, &structure,
                    DOODAD, Env().liquidTypes);
            }
            break;
        case 7:
            (void)TcData::ParseTileSpawns(bytes);
            break;
        case 8:
            (void)TcData::ParseGameObjectModels(bytes);
            break;
        default:
            (void)Db2::ParseHeader(data, size);
            break;
    }

    structure.Clean();
}

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size)
{
    if (size < 1)
    {
        return 0;
    }

    switch (data[0] % 10)
    {
        case 0:
            FuzzAdt(data + 1, size - 1);
            break;
        case 1:
            FuzzWmo(data + 1, size - 1);
            break;
        case 2:
            FuzzM2(data + 1, size - 1);
            break;
        case 3:
            FuzzWdt(data + 1, size - 1);
            break;
        case 4:
            FuzzDbc(data + 1, size - 1);
            break;
        default:
            FuzzTcData(data[0] % 10, data + 1, size - 1);
            break;
    }

    return 0;
}
