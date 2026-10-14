// anav_db2info: header of DB2 files (WDC3-5) and, with TrinityCore's ExtractorDB2LoadInfo.h, whether the extractors
// can read them. Run it on the tables the extractors need (Map, LiquidType, LiquidMaterial, LiquidObject,
// CinematicCamera, GameObjectDisplayInfo...) exported from the client with a CASC tool, after a client patch broke the
// extraction.

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "../AmeisenNavigation.Exporter/src/Dbc/Db2Header.hpp"

namespace {
std::vector<unsigned char> ReadHead(const std::filesystem::path& path)
{
    std::ifstream in(path, std::ios::binary);
    std::vector<unsigned char> bytes(4 + 4 + 128 + 68);
    in.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    bytes.resize(static_cast<size_t>(in.gcount()));
    return bytes;
}
} // namespace

int main(int argc, char** argv)
{
    std::vector<std::filesystem::path> files;
    std::filesystem::path loadInfoPath;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];

        if (arg == "--loadinfo" && i + 1 < argc)
        {
            loadInfoPath = argv[++i];
        }
        else if (arg == "-h" || arg == "--help")
        {
            files.clear();
            break;
        }
        else
        {
            files.emplace_back(arg);
        }
    }

    if (files.empty())
    {
        std::printf("Usage: anav_db2info <file.db2>... [--loadinfo <ExtractorDB2LoadInfo.h>]\n\n"
                    "Prints the layout hash and field count of DB2 files (WDC3-5). With TrinityCore's\n"
                    "src/tools/extractor_common/ExtractorDB2LoadInfo.h, checks each file against the layout the\n"
                    "extractors expect. A mismatch means the client build changed the table: update its LoadInfo\n"
                    "(field list from WoWDBDefs, github.com/wowdev/WoWDBDefs) before extracting.\n");
        return 1;
    }

    std::unordered_map<std::string, Db2::ExpectedLayout> expected;

    if (!loadInfoPath.empty())
    {
        std::ifstream in(loadInfoPath);
        std::stringstream text;
        text << in.rdbuf();
        expected = Db2::ParseLoadInfo(text.str());

        if (expected.empty())
        {
            std::fprintf(stderr, "No layouts found in %s\n", loadInfoPath.string().c_str());
            return 1;
        }
    }

    int problems = 0;

    for (const auto& file : files)
    {
        const auto head = ReadHead(file);
        const auto header = Db2::ParseHeader(head.data(), head.size());
        const std::string name = file.filename().string();

        if (!header)
        {
            std::printf("%-28s not a WDC3/WDC4/WDC5 file\n", name.c_str());
            problems++;
            continue;
        }

        std::printf("%-28s %s, %u records, %u fields (%u in total), layout hash 0x%08X, table hash 0x%08X\n",
                    name.c_str(), header->format.c_str(), header->recordCount, header->fieldCount,
                    header->totalFieldCount, header->layoutHash, header->tableHash);

        if (expected.empty())
        {
            continue;
        }

        const auto it = expected.find(Db2::TableKey(name));

        if (it == expected.end())
        {
            std::printf("%-28s   not used by the extractors\n", "");
            continue;
        }

        const Db2::ExpectedLayout& layout = it->second;
        const bool hashOk = layout.layoutHash == header->layoutHash;
        const bool fieldsOk =
            layout.fileFieldCount < 0 || static_cast<uint32_t>(layout.fileFieldCount) == header->fieldCount;

        if (hashOk && fieldsOk)
        {
            std::printf("%-28s   OK (%s)\n", "", layout.structName.c_str());
            continue;
        }

        problems++;
        std::printf("%-28s   MISMATCH: %s expects layout hash 0x%08X and %d fields in the file\n", "",
                    layout.structName.c_str(), layout.layoutHash, layout.fileFieldCount);
        std::printf("%-28s   -> set .LayoutHash = 0x%08X and the fields of this build (%u columns)\n", "",
                    header->layoutHash, header->fieldCount);
    }

    return problems > 0 ? 2 : 0;
}
