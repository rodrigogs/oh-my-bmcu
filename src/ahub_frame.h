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

// A set request, as the RX parser hands it over (ahubus_package_set_head): 33, flag, length, CRC8,
// command 03, set_type, set_adr, data_struct_count, then the data from byte 8. The frame is
// length * 4 + 12 bytes and its last 4 are the CRC32, so the data ends 4 bytes before the frame does.
#define AHUB_SET_DATA_OFF 8
#define AHUB_CRC32_LEN    4

// Data bytes each set type reads. filament_info: the 44-byte info, then the channel. dryer_stu:
// dryer_power to dryer_time_left (4 bytes), then the channel. all_filament_stu: data_struct_count
// entries of the AMS address, now_filament_num and the four channels' motion.
#define AHUB_SET_FILAMENT_INFO_LEN 45
#define AHUB_SET_DRYER_STU_LEN     5
#define AHUB_SET_FILAMENT_STU_LEN  6

// len: the whole frame, CRC32 included. True if data_len bytes of set data lie before the CRC32.
// Every frame passed its CRC32, so a shorter one is a format this firmware does not know, not a cut
// one; the handlers read fixed offsets, and what lay past the data came from the CRC32 and from an
// older frame still in the RX buffer.
static inline bool ahub_set_data_fits(int len, int data_len)
{
    return AHUB_SET_DATA_OFF + data_len + AHUB_CRC32_LEN <= len;
}

// The active channel of one AMS, as an all_filament_stu set request sends it: 0-3, or 0xFF for none.
// Everything that reads now_filament_num tells a channel by < 4 and none by 0xFF, so any other byte
// is stored as none.
static inline uint8_t ahub_now_filament_num(uint8_t raw)
{
    return (raw < 4u) ? raw : 0xFFu;
}
