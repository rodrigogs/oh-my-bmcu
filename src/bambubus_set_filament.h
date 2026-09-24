#pragma once
// Decoding of the printer's set-filament requests, 0x08 (short frame) and 0x218 (long frame).
// Hardware-free, so it is tested on the host (test/test_bambubus_set_filament).
//
// Every frame that reaches a handler passed CRC16 over its own length, so it is complete: a frame
// shorter than the layout below is a format this firmware does not know, not a cut one. The
// handlers used to read the fixed offsets whatever the length, so fields past the payload came from
// the CRC16 and from an older frame still in the 1280-byte RX buffer, and were saved to flash. Now a
// frame too short for the fixed fields (id, color, temperatures) is dropped, and the name is taken
// only from bytes inside the payload, zero-filled after them. No capture of these frames has been
// checked against the layout, so a shorter name field is not grounds to drop the whole request.
// A frame that holds the full layout decodes exactly as before.
//
// The name is the filament type ("PLA", "PETG-HF"). RAM keeps up to 19 chars from 0x08 and 16 from
// 0x218, but the NVM record (Flash_FilamentInfo::name) has 16 bytes, so a longer name reads back cut
// to 16 after a power cycle. The journal layout stays as it is, so saved records remain loadable.
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct
{
    uint8_t ams_num;
    uint8_t channel;
    char    id[8];
    uint8_t color_R;
    uint8_t color_G;
    uint8_t color_B;
    uint8_t color_A;
    int16_t temperature_min;
    int16_t temperature_max;
    char    name[20]; // always NUL-terminated
} bambubus_set_filament_t;

// Both layouts carry the same fields, from the filament id on: id[8], R, G, B, A, temperature_min
// and temperature_max (int16, little endian), then the name.
#define BAMBUBUS_FIL_FIXED_LEN 16

// 0x08: 3D, flags, length, CRC8, 08, ams << 4 | channel, (unused), fields from buf[7] with the
// 20-byte name at buf[23..42], CRC16: 45 bytes. Shortest accepted: the fixed fields, 7 + 16 + CRC16.
#define BAMBUBUS_SET_FIL_FIELDS_OFF 7
#define BAMBUBUS_SET_FIL_NAME_LEN   20
#define BAMBUBUS_SET_FIL_MIN_LEN    (BAMBUBUS_SET_FIL_FIELDS_OFF + BAMBUBUS_FIL_FIXED_LEN + 2)

// 0x218 payload (long-frame header and CRC16 excluded): ams, channel, fields from datas[2] with the
// 16-byte name at datas[18..33]: 34 bytes. Shortest accepted: the fixed fields, 2 + 16.
#define BAMBUBUS_SET_FIL2_FIELDS_OFF 2
#define BAMBUBUS_SET_FIL2_NAME_LEN   16
#define BAMBUBUS_SET_FIL2_MIN_LEN    (BAMBUBUS_SET_FIL2_FIELDS_OFF + BAMBUBUS_FIL_FIXED_LEN)

// p: first field byte, n: payload bytes from p on (at least BAMBUBUS_FIL_FIXED_LEN), name_len: size
// of the name field in this layout.
static inline void bambubus_set_filament_fields(bambubus_set_filament_t *o, const uint8_t *p, int n, int name_len)
{
    memcpy(o->id, p, sizeof(o->id));
    o->color_R = p[8];
    o->color_G = p[9];
    o->color_B = p[10];
    o->color_A = p[11];
    memcpy(&o->temperature_min, p + 12, 2);
    memcpy(&o->temperature_max, p + 14, 2);

    int k = n - BAMBUBUS_FIL_FIXED_LEN;
    if (k > name_len) k = name_len;
    memset(o->name, 0, sizeof(o->name));
    memcpy(o->name, p + BAMBUBUS_FIL_FIXED_LEN, (size_t)k);
    o->name[sizeof(o->name) - 1u] = 0;
}

// 0x08 set_filament_info. len: whole frame including CRC16. False: too short, drop it.
static inline bool bambubus_decode_set_filament(const uint8_t *buf, int len, bambubus_set_filament_t *o)
{
    if (len < BAMBUBUS_SET_FIL_MIN_LEN) return false;

    o->ams_num = (uint8_t)((buf[5] >> 4) & 0x0Fu);
    o->channel = (uint8_t)(buf[5] & 0x0Fu);
    bambubus_set_filament_fields(o, buf + BAMBUBUS_SET_FIL_FIELDS_OFF,
                                 len - 2 - BAMBUBUS_SET_FIL_FIELDS_OFF, BAMBUBUS_SET_FIL_NAME_LEN);
    return true;
}

// 0x218 set_filament_info_type2. datas, data_length: the long-frame payload. False: too short.
static inline bool bambubus_decode_set_filament_type2(const uint8_t *datas, int data_length, bambubus_set_filament_t *o)
{
    if (data_length < BAMBUBUS_SET_FIL2_MIN_LEN) return false;

    o->ams_num = datas[0];
    o->channel = datas[1];
    bambubus_set_filament_fields(o, datas + BAMBUBUS_SET_FIL2_FIELDS_OFF,
                                 data_length - BAMBUBUS_SET_FIL2_FIELDS_OFF, BAMBUBUS_SET_FIL2_NAME_LEN);
    return true;
}
