#include <algorithm>
#include <charconv>
#include <cstdio>
#include <string_view>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "../../AmeisenNavigation/src/Utils/Logger.hpp"
#include "Exporter.hpp"

namespace {
struct CommandLine
{
    ExportOptions options;
    int threads = 0;
    bool listMaps = false;
};

void PrintUsage()
{
    std::printf(
        "Usage: AmeisenNavigation.Exporter --wow <path> --output <path> [options]\n"
        "       AmeisenNavigation.Exporter --tc-data <path> --output <path> [options]\n\n"
        "  -w, --wow <path>        WoW client folder (or its Data folder), MPQ clients (up to WotLK)\n"
        "      --tc-data <path>    Folder with TrinityCore's extracted maps/ and vmaps/ (recent clients, see README)\n"
        "      --area-table <csv>  With --tc-data: AreaTable as CSV for faction and city areas\n"
        "      --liquid-types <csv> With --tc-data: LiquidType as CSV for WMO liquid kinds\n"
        "  -o, --output <path>     Output folder for the .anp files\n"
        "  -m, --map <ids>         Only export these map ids (comma separated, e.g. 0,1,530,571)\n"
        "  -t, --tile <x,y>        Only export a single ADT (debugging)\n"
        "  -j, --threads <n>       Number of worker threads (default: all cores)\n"
        "  -s, --skip-existing     Don't rebuild maps whose .anp file already exists\n"
        "  -d, --debug             Debug logging and area debug images (<output>/debug)\n"
        "      --no-world          Don't write the static world (.anw, collision geometry for line of sight etc.)\n"
        "  -l, --list-maps         List the maps (Map.dbc, or the files in --tc-data) and exit\n"
        "  -g, --gameobjects <file> Add the collision of these game object spawns (see README)\n"
        "      --gameobject-exclude <types|none> Game object types left out [0,11,15,33,35: doors,\n"
        "                          transports, destructible buildings, trapdoors]\n"
        "  -h, --help              Show this help\n\n"
        "Agent (yards, degrees), defaults in brackets:\n"
        "      --climb <h>         Highest step walked up without jumping [1.0], obstacles above it block\n"
        "      --radius <r>        Agent radius, distance kept from walls [0.6]\n"
        "      --height <h>        Agent height, lower ceilings block [2.0]\n"
        "      --slope <deg>       Steepest walkable slope [55]\n"
        "      --jump-height <h>   Highest obstacle jumped over, 0 = no jump links [1.5]\n"
        "      --drop-height <h>   Highest ledge dropped down, 0 = no drop links [10]\n\n"
        "Example: AmeisenNavigation.Exporter -w \"C:\\WoW\" -o \"C:\\meshes\" -m 0,1 -j 8\n");
}

bool ParseInt(std::string_view s, int& out) noexcept
{
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
    return ec == std::errc() && ptr == s.data() + s.size();
}

bool ParseFloat(std::string_view s, float& out, float min, float max) noexcept
{
    float value = 0.0f;
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);

    if (ec != std::errc() || ptr != s.data() + s.size() || !(value >= min && value <= max))
    {
        return false;
    }

    out = value;
    return true;
}

/// Returns false (after printing the reason) if the arguments are invalid.
bool ParseArguments(int argc, char** argv, CommandLine& cmd, bool& exitEarly)
{
    exitEarly = false;
    auto& options = cmd.options;

    for (int i = 1; i < argc; ++i)
    {
        const std::string_view arg = argv[i];
        const bool hasValue = i + 1 < argc;

        if (arg == "--help" || arg == "-h")
        {
            PrintUsage();
            exitEarly = true;
            return true;
        }

        if (arg == "--debug" || arg == "-d")
        {
            options.debug = true;
        }
        else if (arg == "--list-maps" || arg == "-l")
        {
            cmd.listMaps = true;
        }
        else if (arg == "--no-world")
        {
            options.writeWorld = false;
        }
        else if (arg == "--skip-existing" || arg == "-s")
        {
            options.skipExisting = true;
        }
        else if ((arg == "--wow" || arg == "-w") && hasValue)
        {
            options.wowDir = argv[++i];
        }
        else if (arg == "--tc-data" && hasValue)
        {
            options.tcDataDir = argv[++i];
        }
        else if (arg == "--area-table" && hasValue)
        {
            options.areaTableCsv = argv[++i];
        }
        else if (arg == "--liquid-types" && hasValue)
        {
            options.liquidTypeCsv = argv[++i];
        }
        else if ((arg == "--output" || arg == "-o") && hasValue)
        {
            options.outputDir = argv[++i];
        }
        else if ((arg == "--map" || arg == "-m") && hasValue)
        {
            std::string_view list = argv[++i];

            while (!list.empty())
            {
                const auto comma = list.find(',');
                const auto token = list.substr(0, comma);
                int id = 0;

                if (!ParseInt(token, id) || id < 0)
                {
                    LogE("Invalid --map value: ", token);
                    return false;
                }

                options.mapIds.push_back(id);
                list = comma == std::string_view::npos ? std::string_view() : list.substr(comma + 1);
            }
        }
        else if ((arg == "--tile" || arg == "-t") && hasValue)
        {
            const std::string_view tile = argv[++i];
            const auto comma = tile.find(',');

            if (comma == std::string_view::npos || !ParseInt(tile.substr(0, comma), options.tileX)
                || !ParseInt(tile.substr(comma + 1), options.tileY) || options.tileX < 0 || options.tileX >= 64
                || options.tileY < 0 || options.tileY >= 64)
            {
                LogE("Invalid --tile value: ", tile, " (expected x,y with 0 <= x,y < 64)");
                return false;
            }
        }
        else if ((arg == "--gameobjects" || arg == "-g") && hasValue)
        {
            options.gameObjects = argv[++i];
        }
        else if (arg == "--gameobject-exclude" && hasValue)
        {
            std::string_view list = argv[++i];
            options.gameObjectExcludedTypes.clear();

            while (!list.empty() && list != "none")
            {
                const auto comma = list.find(',');
                int type = 0;

                if (!ParseInt(list.substr(0, comma), type))
                {
                    LogE("Invalid --gameobject-exclude value: ", argv[i]);
                    return false;
                }

                options.gameObjectExcludedTypes.push_back(type);
                list = comma == std::string_view::npos ? std::string_view() : list.substr(comma + 1);
            }
        }
        else if ((arg == "--jump-height" || arg == "--drop-height") && hasValue)
        {
            auto& links = options.buildConfig.links;
            float value = 0.0f;

            if (!ParseFloat(argv[++i], value, 0.0f, 50.0f))
            {
                LogE("Invalid ", arg, " value: ", argv[i]);
                return false;
            }

            if (arg == "--jump-height")
            {
                links.jumps = value > 0.0f;
                links.maxJumpObstacle = value;
                links.maxJumpUp = std::min(links.maxJumpUp, value);
            }
            else
            {
                links.drops = value > 0.0f;
                links.maxDropHeight = value;
            }
        }
        else if ((arg == "--climb" || arg == "--radius" || arg == "--height" || arg == "--slope") && hasValue)
        {
            auto& build = options.buildConfig;
            const std::string_view value = argv[++i];
            const bool ok = arg == "--climb"    ? ParseFloat(value, build.agentClimb, 0.1f, 5.0f)
                            : arg == "--radius" ? ParseFloat(value, build.agentRadius, 0.1f, 5.0f)
                            : arg == "--height" ? ParseFloat(value, build.agentHeight, 0.5f, 10.0f)
                                                : ParseFloat(value, build.walkableSlopeAngle, 10.0f, 85.0f);

            if (!ok)
            {
                LogE("Invalid ", arg, " value: ", value);
                return false;
            }
        }
        else if ((arg == "--threads" || arg == "-j") && hasValue)
        {
            if (!ParseInt(argv[++i], cmd.threads) || cmd.threads < 1)
            {
                LogE("Invalid --threads value: ", argv[i]);
                return false;
            }
        }
        else
        {
            LogE("Unknown or incomplete argument: ", arg);
            PrintUsage();
            return false;
        }
    }

    if ((options.wowDir.empty() && options.tcDataDir.empty()) || (options.outputDir.empty() && !cmd.listMaps))
    {
        LogE("Missing required arguments.");
        PrintUsage();
        return false;
    }

    return true;
}

void PrintBanner()
{
    const bool color = Logger::IsColorEnabled();
    std::fputs(std::format("{}"
                           "      ___                   _                 _   __\n"
                           "     /   |  ____ ___  ___  (_)_______  ____  / | / /___ __   __\n"
                           "    / /| | / __ `__ \\/ _ \\/ / ___/ _ \\/ __ \\/  |/ / __ `/ | / /\n"
                           "   / ___ |/ / / / / /  __/ (__  )  __/ / / / /|  / /_/ /| |/ / \n"
                           "  /_/  |_/_/ /_/ /_/\\___/_/____/\\___/_/ /_/_/ |_/\\__,_/ |___/\n"
                           "                                          Exporter {}{}\n\n",
                           color ? "\033[96m" : "", AMEISENNAV_VERSION, color ? "\033[0m" : "")
                   .c_str(),
               stdout);
}
} // namespace

int main(int argc, char** argv)
{
    Logger::Initialize();

    CommandLine cmd;
    bool exitEarly = false;

    if (!ParseArguments(argc, argv, cmd, exitEarly))
    {
        return 1;
    }

    if (exitEarly)
    {
        return 0;
    }

    Logger::SetDebugEnabled(cmd.options.debug || Logger::IsDebugEnabled());
    PrintBanner();

#ifdef _OPENMP
    if (cmd.threads > 0)
    {
        omp_set_num_threads(cmd.threads);
    }

    // Tiles use nested parallelism for single-ADT exports (MSVC only implements OpenMP 2.0).
#if _OPENMP >= 200805
    omp_set_max_active_levels(2);
#else
    omp_set_nested(1);
#endif
#endif

    if (cmd.listMaps)
    {
        const auto maps =
            cmd.options.tcDataDir.empty() ? ListMaps(cmd.options.wowDir) : ListTcMaps(cmd.options.tcDataDir);

        for (const auto& [id, name] : maps)
        {
            std::printf("%5u  %s\n", id, name.c_str());
        }

        return maps.empty() ? 1 : 0;
    }

    const ExportReport report = RunExport(cmd.options);

    if (!report.setupOk)
    {
        return 1;
    }

    return report.failedMaps > 0 ? 2 : 0;
}
