// anav_navmesh2ply: dump a map's navmesh (ANP or any MMAP format the server reads) as a colored PLY mesh, to look at
// it in MeshLab, Blender or CloudCompare (slope or area colors, off-mesh connections as ribbons).

#include <charconv>
#include <cstdio>
#include <string>
#include <string_view>

#include "AmeisenNavigation.hpp"
#include "NavMeshPly.hpp"

namespace {
void PrintUsage()
{
    std::printf("Usage: anav_navmesh2ply <meshes folder> <mapId> <output.ply> [options]\n\n"
                "  --anp                 ANP navmeshes (default: MMAPs, format detected)\n"
                "  --format <n>          MMAP format like iMmapFormat: 0 auto, 1 TC 3.3.5a, 2 SkyFire, 3 TC master\n"
                "  --box <x0,y0,x1,y1>   Only this WoW x/y range\n"
                "  --area                Color by area instead of slope\n\n"
                "Colors: slope 0 blue, 15 cyan, 30 green, 45 yellow, 60+ red. Off-mesh connections: jump yellow,\n"
                "drop magenta, other links white.\n");
}

bool ParseFloats(std::string_view s, float* out, int count)
{
    for (int i = 0; i < count; ++i)
    {
        const auto comma = s.find(',');
        const std::string_view token = s.substr(0, comma);
        const auto [ptr, ec] = std::from_chars(token.data(), token.data() + token.size(), out[i]);

        if (ec != std::errc() || ptr != token.data() + token.size() || (i + 1 < count && comma == std::string_view::npos))
        {
            return false;
        }

        s = comma == std::string_view::npos ? std::string_view{} : s.substr(comma + 1);
    }

    return s.empty();
}
} // namespace

int main(int argc, char** argv)
{
    if (argc < 4)
    {
        PrintUsage();
        return 2;
    }

    Logger::Initialize();
    const std::filesystem::path meshes = argv[1];
    int mapId = -1;
    const std::string_view mapArg = argv[2];

    if (std::from_chars(mapArg.data(), mapArg.data() + mapArg.size(), mapId).ec != std::errc() || mapId < 0)
    {
        PrintUsage();
        return 2;
    }

    const std::string output = argv[3];
    bool anp = false;
    int format = 0;
    NavMeshPly::Options options;

    for (int i = 4; i < argc; ++i)
    {
        const std::string_view arg = argv[i];

        if (arg == "--anp")
        {
            anp = true;
        }
        else if (arg == "--area")
        {
            options.color = NavMeshPly::ColorMode::AREA;
        }
        else if (arg == "--format" && i + 1 < argc)
        {
            const std::string_view value = argv[++i];

            if (std::from_chars(value.data(), value.data() + value.size(), format).ec != std::errc() || format < 0
                || format > 3)
            {
                PrintUsage();
                return 2;
            }
        }
        else if (arg == "--box" && i + 1 < argc)
        {
            float box[4]{};

            if (!ParseFloats(argv[++i], box, 4))
            {
                PrintUsage();
                return 2;
            }

            options.box = true;
            options.boxMin[0] = std::min(box[0], box[2]);
            options.boxMin[1] = std::min(box[1], box[3]);
            options.boxMax[0] = std::max(box[0], box[2]);
            options.boxMax[1] = std::max(box[1], box[3]);
        }
        else
        {
            PrintUsage();
            return 2;
        }
    }

    const MmapFormat formats[]{MmapFormat::UNKNOWN, MmapFormat::TC335A, MmapFormat::SF548, MmapFormat::TC_MASTER};
    std::unique_ptr<INavSource> source;

    if (anp)
    {
        source = std::make_unique<AnpNavSource>(meshes);
    }
    else
    {
        source = std::make_unique<MmapNavSource>(meshes, formats[format]);
    }

    const dtNavMesh* mesh = source->Get(mapId);

    if (!mesh)
    {
        LogE("No navmesh for map ", mapId, " in ", meshes.string());
        return 1;
    }

    NavMeshPly::Stats stats;

    if (!NavMeshPly::Write(*mesh, output, options, &stats))
    {
        LogE("Failed to write ", output);
        return 1;
    }

    LogS("Wrote ", output, ": ", stats.triangles, " triangles, ", stats.offMeshConnections, " off-mesh connections");
    return 0;
}
