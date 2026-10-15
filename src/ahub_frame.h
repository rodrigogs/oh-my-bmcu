#pragma once
// Checks on the AHUB (0x33) frames that ahub_bus.cpp handles. Hardware-free, so they are tested on
// the host (test/test_ahub_frame). An A1 only speaks BambuBus (0x3D), so none of this runs there.
#include <stdbool.h>
#include <stdint.h>

// Header of the frames ahub_bus.cpp decodes and builds: 33, flag with bit 7 set, length, CRC8 of
// bytes 0-2, command. The frame is length * 4 + 12 bytes, all but its last word covered by the CRC32
// in that word. The RX parser (_bus_hardware.h) also hands over 0x33 frames with flag bit 7 clear,
// sized from byte 4 with the CRC8 at byte 6, as a BambuBus long header. This firmware knows no AHUB
// layout of that kind (byte 4 is where the short header has the command), so they are dropped.
#define AHUB_FLAG_SHORT_HEADER 0x80u

static inline bool ahub_frame_short_header(const uint8_t *buf)
{
    return (buf[1] & AHUB_FLAG_SHORT_HEADER) != 0u;
}

// Words the CRC32 covers in a short-header frame: length + 2, from byte 2 as the RX parser sizes it.
static inline uint32_t ahub_frame_crc_words(const uint8_t *buf)
{
    return (uint32_t)buf[2] + 2u;
}
