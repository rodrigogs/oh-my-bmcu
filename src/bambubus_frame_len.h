#pragma once
// Shortest printer short frame (3D C5 ...) each command's handler can take its fields from.
// Hardware-free, so it is tested on the host (test/test_bambubus_frame_len).
//
// Every frame that reaches get_packge_type() passed CRC16 over its own length, so it is complete: a
// frame shorter than the fields its handler reads is a format this firmware does not know, not a cut
// one. The handlers read fixed offsets whatever the length, so such a frame's fields came from its
// own CRC16 and from an older frame still in the 1280-byte RX buffer; a 0x03 or 0x04 of that kind
// runs set_motion(), which can end a load or clear the loaded channel. Each minimum is the last byte
// the handler uses plus the CRC16, never more: the 0x03 and 0x04 frames captured from the printer
// (quoted in bambu_bus_ams.cpp) are one byte longer, a byte the handlers ignore. No 0x05 capture is
// known, so its minima are the handler's reads alone. A shorter frame is dropped as a CRC failure
// is. 0x08 checks its own length (bambubus_set_filament.h); the other commands have no handler.
#include <stdbool.h>
#include <stdint.h>

#define BAMBUBUS_CRC16_LEN 2

// 0x03 motion, 3D C5 0C C8 03 00 07 00 7F 02 36 54 (12 bytes): unit, state, channel and motion flag
// at buf[5..8]; buf[9] is not used.
#define BAMBUBUS_MOTION_MIN_LEN (8 + 1 + BAMBUBUS_CRC16_LEN)

// 0x04 motion with status, 3D C5 0D F1 04 00 01 00 03 FF 00 B2 C4 (13 bytes): unit, state and motion
// flag at buf[5..7], channel at buf[9]; buf[8] and buf[10] are not used.
#define BAMBUBUS_STU_MOTION_MIN_LEN (9 + 1 + BAMBUBUS_CRC16_LEN)

// 0x05 online detect: the subtype at buf[5], all a discovery probe (0x00) reads. The confirm (0x01)
// also reads the unit at buf[6] and compares the prefix and the 16 ID bytes of our last reply at
// buf[7..23].
#define BAMBUBUS_ONLINE_DETECT_MIN_LEN  (5 + 1 + BAMBUBUS_CRC16_LEN)
#define BAMBUBUS_ONLINE_ID_OFF          7
#define BAMBUBUS_ONLINE_ID_LEN          17
#define BAMBUBUS_ONLINE_CONFIRM_MIN_LEN (BAMBUBUS_ONLINE_ID_OFF + BAMBUBUS_ONLINE_ID_LEN + BAMBUBUS_CRC16_LEN)

// buf: a short frame that passed CRC16, len: its length, CRC16 included (at least 6). False: too
// short for its command's fields, drop it.
static inline bool bambubus_short_frame_len_ok(const uint8_t *buf, int len)
{
    switch (buf[4])
    {
    case 0x03:
        return len >= BAMBUBUS_MOTION_MIN_LEN;
    case 0x04:
        return len >= BAMBUBUS_STU_MOTION_MIN_LEN;
    case 0x05:
        if (len < BAMBUBUS_ONLINE_DETECT_MIN_LEN) return false;
        return (buf[5] != 0x01) || (len >= BAMBUBUS_ONLINE_CONFIRM_MIN_LEN);
    default:
        return true;
    }
}
