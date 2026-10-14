#pragma once

enum class MmapFormat
{
    UNKNOWN,   // auto detect
    TC335A,    // TrinityCore 3.3.5a: "{:03}.mmap", "{:03}{:02}{:02}.mmtile"
    SF548,     // SkyFire 5.4.8 / older TrinityCore master: "{:04}.mmap", "{:04}_{:02}_{:02}.mmtile"
    CUSTOM,    // user supplied std::format patterns
    TC_MASTER, // TrinityCore master (MMAP v16+, retail and classic clients): SF548 names, .mmap with a header
};
