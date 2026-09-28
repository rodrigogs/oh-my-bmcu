#pragma once
// Checks on the AHUB (0x33) frames that ahub_bus.cpp handles. Hardware-free, so they are tested on
// the host (test/test_ahub_frame). An A1 only speaks BambuBus (0x3D), so none of this runs there.
#include <stdint.h>

// The active channel of one AMS, as an all_filament_stu set request sends it: 0-3, or 0xFF for none.
// Everything that reads now_filament_num tells a channel by < 4 and none by 0xFF, so any other byte
// is stored as none.
static inline uint8_t ahub_now_filament_num(uint8_t raw)
{
    return (raw < 4u) ? raw : 0xFFu;
}
