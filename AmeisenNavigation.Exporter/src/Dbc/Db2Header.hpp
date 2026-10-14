#pragma once

#include <cctype>
#include <cstdint>
#include <cstring>
#include <optional>
#include <regex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

/// Header of a DB2 file of a recent client (WDC3, WDC4, WDC5) and the layouts TrinityCore's extractors expect
/// (src/tools/extractor_common/ExtractorDB2LoadInfo.h). A client build that adds a field to Map.db2 or LiquidType.db2
/// changes the layout hash and the field count: the extractors then refuse the file and the layout has to be updated
/// (field list from WoWDBDefs). This tells which tables changed.
namespace Db2 {
struct Header
{
    std::string format; // "WDC3", "WDC4", "WDC5"
    uint32_t version = 0;
    uint32_t recordCount = 0;
    uint32_t fieldCount = 0; // columns in the file (arrays count once)
    uint32_t recordSize = 0;
    uint32_t tableHash = 0;
    uint32_t layoutHash = 0;
    uint32_t minId = 0;
    uint32_t maxId = 0;
    uint32_t locale = 0;
    uint16_t flags = 0;
    uint16_t idIndex = 0;
    uint32_t totalFieldCount = 0;
    uint32_t sectionCount = 0;
};

/// Parse the header, nullopt for anything else (WDBC, WDB2, truncated files).
inline std::optional<Header> ParseHeader(const unsigned char* data, size_t size)
{
    if (size < 4)
    {
        return std::nullopt;
    }

    Header header;
    header.format.assign(reinterpret_cast<const char*>(data), 4);
    size_t pos = 4;

    if (header.format == "WDC5")
    {
        // Version and a 128 byte schema string in front of the WDC4 layout.
        if (size < pos + 4 + 128)
        {
            return std::nullopt;
        }

        std::memcpy(&header.version, data + pos, 4);
        pos += 4 + 128;
    }
    else if (header.format != "WDC3" && header.format != "WDC4")
    {
        return std::nullopt;
    }

    if (size < pos + 68)
    {
        return std::nullopt;
    }

    const auto u32 = [&](size_t offset) {
        uint32_t v = 0;
        std::memcpy(&v, data + pos + offset, 4);
        return v;
    };
    const auto u16 = [&](size_t offset) {
        uint16_t v = 0;
        std::memcpy(&v, data + pos + offset, 2);
        return v;
    };

    header.recordCount = u32(0);
    header.fieldCount = u32(4);
    header.recordSize = u32(8);
    header.tableHash = u32(16);
    header.layoutHash = u32(20);
    header.minId = u32(24);
    header.maxId = u32(28);
    header.locale = u32(32);
    header.flags = u16(36);
    header.idIndex = u16(38);
    header.totalFieldCount = u32(40);
    header.sectionCount = u32(64);
    return header;
}

/// What the extractors expect of a table.
struct ExpectedLayout
{
    std::string structName;
    uint32_t layoutHash = 0;
    int fieldCount = -1;     // DB2Meta FieldCount
    int fileFieldCount = -1; // DB2Meta FileFieldCount, columns in the file
};

/// "Map.db2", "MapLoadInfo" -> "map".
inline std::string TableKey(std::string_view name)
{
    if (const auto slash = name.find_last_of("/\\"); slash != std::string_view::npos)
    {
        name = name.substr(slash + 1);
    }

    for (const std::string_view suffix : {".db2", ".DB2", "LoadInfo"})
    {
        if (name.ends_with(suffix))
        {
            name = name.substr(0, name.size() - suffix.size());
        }
    }

    std::string key;

    for (const char c : name)
    {
        if (std::isalnum(static_cast<unsigned char>(c)))
        {
            key += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
    }

    return key;
}

/// Layouts in ExtractorDB2LoadInfo.h by table key. Reads the designated initializers of TrinityCore master
/// (`.LayoutHash = 0x...`, `.FieldCount = n`, `.FileFieldCount = n`) and falls back to the first hex constant of a
/// struct for older positional initializers.
inline std::unordered_map<std::string, ExpectedLayout> ParseLoadInfo(const std::string& text)
{
    std::unordered_map<std::string, ExpectedLayout> layouts;
    static const std::regex structRegex(R"(struct\s+(\w+LoadInfo)\b)");
    static const std::regex hashRegex(R"(\.LayoutHash\s*=\s*(0x[0-9A-Fa-f]+))");
    static const std::regex fieldRegex(R"(\.FieldCount\s*=\s*(\d+))");
    static const std::regex fileFieldRegex(R"(\.FileFieldCount\s*=\s*(\d+))");
    static const std::regex anyHexRegex(R"(\b(0x[0-9A-Fa-f]{8})\b)");

    std::vector<std::pair<size_t, std::string>> structs;

    for (auto it = std::sregex_iterator(text.begin(), text.end(), structRegex); it != std::sregex_iterator(); ++it)
    {
        structs.emplace_back(static_cast<size_t>(it->position()), (*it)[1].str());
    }

    for (size_t i = 0; i < structs.size(); ++i)
    {
        const size_t begin = structs[i].first;
        const size_t end = i + 1 < structs.size() ? structs[i + 1].first : text.size();
        const std::string body = text.substr(begin, end - begin);
        ExpectedLayout layout;
        layout.structName = structs[i].second;
        std::smatch match;

        if (std::regex_search(body, match, hashRegex) || std::regex_search(body, match, anyHexRegex))
        {
            layout.layoutHash = static_cast<uint32_t>(std::stoul(match[1].str(), nullptr, 16));
        }
        else
        {
            continue;
        }

        if (std::regex_search(body, match, fieldRegex))
        {
            layout.fieldCount = std::stoi(match[1].str());
        }

        if (std::regex_search(body, match, fileFieldRegex))
        {
            layout.fileFieldCount = std::stoi(match[1].str());
        }

        layouts[TableKey(layout.structName)] = std::move(layout);
    }

    return layouts;
}
} // namespace Db2
