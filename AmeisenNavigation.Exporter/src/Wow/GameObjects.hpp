#pragma once

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../Dbc/Dbc.hpp"
#include "../Mpq/CachedFileReader.hpp"
#include "../Utils/Matrix4x4.hpp"
#include "../Utils/Structure.hpp"
#include "AdtChunkExtractor.hpp"

// ─────────────────────────────────────────────
// Game objects spawned by the server (fences, gates, crates, stalls, braziers...) are not in the ADTs: without them
// the navmesh leads straight through them. Their spawns come from a file (the world database of a server emulator,
// or what a bot saw in the game), their models from GameObjectDisplayInfo.dbc, and their collision is added to the
// map like a doodad.
// ─────────────────────────────────────────────

/// One spawned game object (WoW coordinates).
struct GameObjectSpawn
{
    int mapId = -1;
    unsigned int displayId = 0;
    std::string model;          // model path, empty = from GameObjectDisplayInfo.dbc by displayId
    Vector3 position;
    float orientation = 0.0f;   // radians around z, used when the rotation is zero
    float rotation[4]{};        // quaternion x, y, z, w (0, 0, 0, 0 = use the orientation)
    float scale = 1.0f;
    int type = -1;              // GameObject type, -1 = unknown
};

/// Game object types left out by default: they move or change (open doors, transports, buildings that get
/// destroyed). Doors are assumed open: a bot opens them or walks through when they are.
inline const std::unordered_set<int>& DefaultExcludedGameObjectTypes()
{
    static const std::unordered_set<int> types{
        0,  // DOOR
        11, // TRANSPORT (elevators)
        15, // MO_TRANSPORT (ships, zeppelins)
        33, // DESTRUCTIBLE_BUILDING
        35, // TRAPDOOR
    };

    return types;
}

namespace GameObjects {
inline std::string_view Trim(std::string_view s) noexcept
{
    constexpr std::string_view whitespace = " \t\r\n\"'";
    const auto begin = s.find_first_not_of(whitespace);
    return begin == std::string_view::npos ? std::string_view{}
                                           : s.substr(begin, s.find_last_not_of(whitespace) - begin + 1);
}

inline std::vector<std::string_view> Split(std::string_view line)
{
    std::vector<std::string_view> fields;
    const char delimiter = line.find('\t') != std::string_view::npos   ? '\t'
                           : line.find(';') != std::string_view::npos ? ';'
                                                                      : ',';

    while (true)
    {
        const auto pos = line.find(delimiter);
        fields.push_back(Trim(line.substr(0, pos)));

        if (pos == std::string_view::npos)
        {
            break;
        }

        line = line.substr(pos + 1);
    }

    return fields;
}

template <typename T>
bool Parse(std::string_view s, T& out) noexcept
{
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
    return ec == std::errc() && ptr == s.data() + s.size();
}

inline std::string Lower(std::string_view s)
{
    std::string out(s);
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

/// Read spawns from a delimited text file (comma, semicolon or tab). With a header line the columns are found by
/// name (map, displayId or model, x/position_x, y/position_y, z/position_z, orientation, rotation0..3, scale/size,
/// type), without one they are: map, displayId, x, y, z, orientation, rotation0, rotation1, rotation2, rotation3,
/// scale, type (trailing columns optional). Lines starting with '#' are comments. Invalid lines are skipped and
/// counted in *invalid.
inline std::vector<GameObjectSpawn> ReadSpawns(const std::filesystem::path& file, size_t* invalid = nullptr)
{
    std::vector<GameObjectSpawn> spawns;
    std::ifstream in(file);
    enum Column { MAP, DISPLAY, MODEL, X, Y, Z, O, R0, R1, R2, R3, SCALE, TYPE, COLUMN_COUNT };
    int columns[COLUMN_COUNT]{0, 1, -1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    bool first = true;
    size_t bad = 0;

    for (std::string line; std::getline(in, line);)
    {
        const std::string_view trimmed = Trim(line);

        if (trimmed.empty() || trimmed[0] == '#')
        {
            continue;
        }

        const auto fields = Split(trimmed);
        int probe = 0;

        if (first && !Parse(fields[0], probe))
        {
            // Header line.
            std::fill(std::begin(columns), std::end(columns), -1);
            static const std::unordered_map<std::string, Column> names{
                {"map", MAP},          {"mapid", MAP},          {"displayid", DISPLAY}, {"display", DISPLAY},
                {"model", MODEL},      {"x", X},                {"position_x", X},      {"y", Y},
                {"position_y", Y},     {"z", Z},                {"position_z", Z},      {"orientation", O},
                {"o", O},              {"rotation0", R0},       {"rotation1", R1},      {"rotation2", R2},
                {"rotation3", R3},     {"qx", R0},              {"qy", R1},             {"qz", R2},
                {"qw", R3},            {"scale", SCALE},        {"size", SCALE},        {"type", TYPE},
            };

            for (size_t i = 0; i < fields.size(); ++i)
            {
                if (const auto it = names.find(Lower(fields[i])); it != names.end())
                {
                    columns[it->second] = static_cast<int>(i);
                }
            }

            first = false;
            continue;
        }

        first = false;
        const auto field = [&](Column c) -> std::string_view {
            return columns[c] >= 0 && static_cast<size_t>(columns[c]) < fields.size() ? fields[static_cast<size_t>(columns[c])]
                                                                                     : std::string_view{};
        };

        GameObjectSpawn spawn;
        const auto optionalFloat = [&](Column c, float& out) { return field(c).empty() || Parse(field(c), out); };

        bool ok = Parse(field(MAP), spawn.mapId) && Parse(field(X), spawn.position.x)
                  && Parse(field(Y), spawn.position.y) && Parse(field(Z), spawn.position.z)
                  && optionalFloat(O, spawn.orientation) && optionalFloat(R0, spawn.rotation[0])
                  && optionalFloat(R1, spawn.rotation[1]) && optionalFloat(R2, spawn.rotation[2])
                  && optionalFloat(R3, spawn.rotation[3]) && optionalFloat(SCALE, spawn.scale)
                  && (field(TYPE).empty() || Parse(field(TYPE), spawn.type));

        spawn.model = std::string(field(MODEL));
        ok = ok && (!spawn.model.empty() || Parse(field(DISPLAY), spawn.displayId));
        ok = ok && spawn.position.IsFinite() && std::fabs(spawn.position.x) < 20000.0f
             && std::fabs(spawn.position.y) < 20000.0f && std::fabs(spawn.position.z) < 20000.0f
             && std::isfinite(spawn.orientation) && std::isfinite(spawn.scale) && spawn.scale > 0.0f
             && spawn.scale <= 100.0f && spawn.mapId >= 0
             && std::all_of(std::begin(spawn.rotation), std::end(spawn.rotation),
                            [](float v) { return std::isfinite(v); });

        if (ok)
        {
            spawns.push_back(std::move(spawn));
        }
        else
        {
            bad++;
        }
    }

    if (invalid)
    {
        *invalid = bad;
    }

    return spawns;
}

/// GameObjectDisplayInfo.dbc (WotLK and older): field 0 = ID, field 1 = ModelName.
inline std::unordered_map<unsigned int, std::string> LoadDisplayModels(const Dbc* dbc)
{
    std::unordered_map<unsigned int, std::string> models;

    if (!dbc || !dbc->IsValid())
    {
        return models;
    }

    for (unsigned int i = 0; i < dbc->GetRecordCount(); ++i)
    {
        if (const char* name = dbc->ReadString(i, 1u); name && *name)
        {
            models[dbc->Read<unsigned int>(i, 0u)] = name;
        }
    }

    return models;
}

/// Transform from the model's space to WoW coordinates: scale, rotation (quaternion, or the orientation around z),
/// translation.
inline Matrix4x4 SpawnTransform(const GameObjectSpawn& spawn) noexcept
{
    Matrix4x4 transform;
    transform.SetScale({spawn.scale, spawn.scale, spawn.scale});
    const float* q = spawn.rotation;

    if (q[0] != 0.0f || q[1] != 0.0f || q[2] != 0.0f || q[3] != 0.0f)
    {
        transform.SetRotation(q[0], q[1], q[2], q[3]);
    }
    else
    {
        transform.SetRotation(0.0f, 0.0f, std::sin(spawn.orientation * 0.5f), std::cos(spawn.orientation * 0.5f));
    }

    transform.SetTranslation(spawn.position);
    return transform;
}

/// Add a spawn's collision (M2 or WMO model) to the structure. Returns false if the model isn't there.
inline bool Add(const GameObjectSpawn& spawn, const std::string& modelPath, CachedFileReader& reader,
                Structure* structure, const std::unordered_map<unsigned int, LiquidType>& liquidTypes)
{
    const std::string lower = Lower(modelPath);
    const Matrix4x4 transform = SpawnTransform(spawn);
    const size_t first = structure->tris.size();
    const uint32_t source = AnpWorld::MakeSource(AnpWorld::SourceKind::GAME_OBJECT, spawn.displayId);

    if (lower.ends_with(".wmo"))
    {
        const bool added = ExtractWmo(modelPath.c_str(), transform, 0, reader, structure, liquidTypes, DOODAD);
        structure->TagSources(first, source);
        return added;
    }

    // .mdx/.mdl (DBC names) and .m2 all mean the M2 file.
    const auto dot = modelPath.find_last_of('.');
    const std::string m2Name = (dot == std::string::npos ? modelPath : modelPath.substr(0, dot)) + ".m2";
    const M2* m2 = reader.GetFileContent<M2>(m2Name.c_str());

    if (!m2)
    {
        return false;
    }

    AddM2Collision(m2, transform, structure, DOODAD);
    structure->TagSources(first, source);
    return true;
}
} // namespace GameObjects
