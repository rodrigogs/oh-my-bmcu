// Host tests for src/bambubus_set_filament.h: a set-filament request (0x08 short frame, 0x218 long
// frame payload) long enough that the old handlers read only payload bytes must decode exactly as
// they did, one too short for the fixed fields must be dropped, and no decoded byte may come from
// past the payload (the CRC16 or an older frame still in the RX buffer).

#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "bambubus_set_filament.h"

// Stands for bytes past the payload: the CRC16 and whatever an earlier, longer frame left in the
// 1280-byte RX buffer. Payload bytes written by the tests never take this value.
#define STALE 0xAAu

void setUp(void) {}
void tearDown(void) {}

// ---- adapted from bambu_bus_ams.cpp: the 0x08 and 0x218 set-filament handlers before bambubus_set_filament.h ----
// The reference for "decodes as before"; not firmware code any more, so not checked.
// The 0x08 handler before this change: fixed offsets, length ignored.
static void old_decode_0x08(const uint8_t *buf, bambubus_set_filament_t *o)
{
    o->ams_num = (uint8_t)((buf[5] >> 4) & 0x0Fu);
    o->channel = (uint8_t)(buf[5] & 0x0Fu);
    memcpy(o->id, buf + 7, 8);
    o->color_R = buf[15];
    o->color_G = buf[16];
    o->color_B = buf[17];
    o->color_A = buf[18];
    memcpy(&o->temperature_min, buf + 19, 2);
    memcpy(&o->temperature_max, buf + 21, 2);
    memcpy(o->name, buf + 23, 20);
    o->name[19] = 0;
}

// The 0x218 handler before this change.
static void old_decode_0x218(const uint8_t *d, bambubus_set_filament_t *o)
{
    o->ams_num = d[0];
    o->channel = d[1];
    memcpy(o->id, d + 2, 8);
    o->color_R = d[10];
    o->color_G = d[11];
    o->color_B = d[12];
    o->color_A = d[13];
    memcpy(&o->temperature_min, d + 14, 2);
    memcpy(&o->temperature_max, d + 16, 2);
    memset(o->name, 0, 20);
    memcpy(o->name, d + 18, 16);
    o->name[19] = 0;
}

static void assert_same(const bambubus_set_filament_t *a, const bambubus_set_filament_t *b)
{
    TEST_ASSERT_EQUAL_HEX8(a->ams_num, b->ams_num);
    TEST_ASSERT_EQUAL_HEX8(a->channel, b->channel);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(a->id, b->id, 8);
    TEST_ASSERT_EQUAL_HEX8(a->color_R, b->color_R);
    TEST_ASSERT_EQUAL_HEX8(a->color_G, b->color_G);
    TEST_ASSERT_EQUAL_HEX8(a->color_B, b->color_B);
    TEST_ASSERT_EQUAL_HEX8(a->color_A, b->color_A);
    TEST_ASSERT_EQUAL_INT16(a->temperature_min, b->temperature_min);
    TEST_ASSERT_EQUAL_INT16(a->temperature_max, b->temperature_max);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(a->name, b->name, 20);
}

static uint32_t rng = 1u;

// Pseudo-random payload byte, never STALE.
static uint8_t payload_byte(void)
{
    rng = rng * 1664525u + 1013904223u;
    uint8_t v = (uint8_t)(rng >> 24);
    return (v == STALE) ? 0x00u : v;
}

// buf gets len - 2 payload bytes, then STALE up to the end: the CRC16 and older bytes.
static void fill_frame(uint8_t *buf, int buf_size, int len)
{
    for (int i = 0; i < buf_size; i++)
        buf[i] = (i < len - 2) ? payload_byte() : STALE;
}

// A frame built like the printer's: header, command, ams/channel, fields, name, CRC16 position.
static int make_0x08(uint8_t *buf, uint8_t ams_ch, const char *name, int name_field)
{
    memset(buf, STALE, 128);
    buf[0] = 0x3D;
    buf[1] = 0xC5;
    buf[3] = 0x00; // CRC8, not checked here
    buf[4] = 0x08;
    buf[5] = ams_ch;
    buf[6] = 0x00;
    static const uint8_t fields[16] = {'G', 'F', 'G', '0', '2', 0, 0, 0,   // id
                                       0x12, 0x34, 0x56, 0xFF,             // RGBA
                                       0xDC, 0x00, 0x04, 0x01};            // 220, 260
    memcpy(buf + 7, fields, sizeof(fields));
    memset(buf + 23, 0, (size_t)name_field);
    memcpy(buf + 23, name, strlen(name) < (size_t)name_field ? strlen(name) : (size_t)name_field);
    const int len = 23 + name_field + 2;
    buf[2] = (uint8_t)len;
    return len;
}

// From 44 bytes on the old handler read only payload bytes: it copied the 20-byte name field but
// cleared its last byte, buf[42].
static void test_0x08_frames_of_44_bytes_or_more_decode_as_before(void)
{
    uint8_t buf[128];
    for (int len = 44; len <= 64; len++)
    {
        for (int round = 0; round < 50; round++)
        {
            fill_frame(buf, (int)sizeof(buf), len);
            bambubus_set_filament_t got, want;
            memset(&got, 0x5A, sizeof(got));
            memset(&want, 0x5A, sizeof(want));
            TEST_ASSERT_TRUE(bambubus_decode_set_filament(buf, len, &got));
            old_decode_0x08(buf, &want);
            assert_same(&want, &got);
        }
    }
}

static void test_0x08_fields_at_their_offsets(void)
{
    uint8_t buf[128];
    const int len = make_0x08(buf, 0x02, "PETG-HF", 20);
    TEST_ASSERT_EQUAL_INT(45, len);

    bambubus_set_filament_t o;
    TEST_ASSERT_TRUE(bambubus_decode_set_filament(buf, len, &o));
    TEST_ASSERT_EQUAL_UINT8(0, o.ams_num);
    TEST_ASSERT_EQUAL_UINT8(2, o.channel);
    TEST_ASSERT_EQUAL_STRING("GFG02", o.id);
    TEST_ASSERT_EQUAL_HEX8(0x12, o.color_R);
    TEST_ASSERT_EQUAL_HEX8(0x34, o.color_G);
    TEST_ASSERT_EQUAL_HEX8(0x56, o.color_B);
    TEST_ASSERT_EQUAL_HEX8(0xFF, o.color_A);
    TEST_ASSERT_EQUAL_INT16(220, o.temperature_min);
    TEST_ASSERT_EQUAL_INT16(260, o.temperature_max);
    TEST_ASSERT_EQUAL_STRING("PETG-HF", o.name);

    TEST_ASSERT_TRUE(bambubus_decode_set_filament(buf, make_0x08(buf, 0x13, "PLA", 20), &o));
    TEST_ASSERT_EQUAL_UINT8(1, o.ams_num);
    TEST_ASSERT_EQUAL_UINT8(3, o.channel);
}

static void test_0x08_name_keeps_19_chars(void)
{
    uint8_t buf[128];
    const int len = make_0x08(buf, 0x00, "ABCDEFGHIJKLMNOPQRST", 20);
    bambubus_set_filament_t o;
    TEST_ASSERT_TRUE(bambubus_decode_set_filament(buf, len, &o));
    TEST_ASSERT_EQUAL_STRING("ABCDEFGHIJKLMNOPQRS", o.name);
}

static void test_0x08_too_short_for_the_fixed_fields_is_dropped(void)
{
    uint8_t buf[128];
    bambubus_set_filament_t o;
    for (int len = 0; len < BAMBUBUS_SET_FIL_MIN_LEN; len++)
    {
        fill_frame(buf, (int)sizeof(buf), len);
        TEST_ASSERT_FALSE(bambubus_decode_set_filament(buf, len, &o));
    }

    // Fixed fields present, no name byte in the payload.
    TEST_ASSERT_EQUAL_INT(25, BAMBUBUS_SET_FIL_MIN_LEN);
    const int len = make_0x08(buf, 0x01, "", 0);
    TEST_ASSERT_EQUAL_INT(25, len);
    TEST_ASSERT_TRUE(bambubus_decode_set_filament(buf, len, &o));
    TEST_ASSERT_EQUAL_UINT8(1, o.channel);
    TEST_ASSERT_EQUAL_INT16(260, o.temperature_max);
    TEST_ASSERT_EQUAL_STRING("", o.name);
}

static void test_0x08_name_stops_at_the_payload_end(void)
{
    // A 16-byte name field: the old handler took bytes 16..18 of the name from the CRC16 and from
    // past the frame.
    uint8_t buf[128];
    const int len = make_0x08(buf, 0x00, "ABCDEFGHIJKLMNOP", 16);
    TEST_ASSERT_EQUAL_INT(41, len);

    bambubus_set_filament_t o;
    TEST_ASSERT_TRUE(bambubus_decode_set_filament(buf, len, &o));
    TEST_ASSERT_EQUAL_STRING("ABCDEFGHIJKLMNOP", o.name);
    for (int i = 16; i < 20; i++)
        TEST_ASSERT_EQUAL_HEX8(0, (uint8_t)o.name[i]);
}

// Every length: either dropped, or every decoded byte is a payload byte.
static void test_0x08_never_reads_past_the_payload(void)
{
    uint8_t buf[128];
    for (int len = 0; len <= 64; len++)
    {
        memset(buf, STALE, sizeof(buf));
        for (int i = 0; i < len - 2; i++)
            buf[i] = (uint8_t)('a' + (i % 26)); // non-NUL, so a leaked byte shows in the name

        bambubus_set_filament_t o;
        if (!bambubus_decode_set_filament(buf, len, &o))
        {
            TEST_ASSERT_LESS_THAN_INT(BAMBUBUS_SET_FIL_MIN_LEN, len);
            continue;
        }

        const int name_in_payload = (len - 2 - 23) < 19 ? (len - 2 - 23) : 19;
        TEST_ASSERT_EQUAL_INT(name_in_payload, (int)strlen(o.name));
        for (int i = 0; i < 20; i++)
            TEST_ASSERT_NOT_EQUAL((int)STALE, (int)(uint8_t)o.name[i]);
        TEST_ASSERT_EQUAL_INT16((int16_t)(buf[19] | (buf[20] << 8)), o.temperature_min);
        TEST_ASSERT_EQUAL_INT16((int16_t)(buf[21] | (buf[22] << 8)), o.temperature_max);
    }
}

static void test_0x218_payloads_of_34_bytes_or_more_decode_as_before(void)
{
    uint8_t d[96];
    for (int n = 34; n <= 60; n++)
    {
        for (int round = 0; round < 50; round++)
        {
            fill_frame(d, (int)sizeof(d), n + 2);
            bambubus_set_filament_t got, want;
            memset(&got, 0x5A, sizeof(got));
            memset(&want, 0x5A, sizeof(want));
            TEST_ASSERT_TRUE(bambubus_decode_set_filament_type2(d, n, &got));
            old_decode_0x218(d, &want);
            assert_same(&want, &got);
        }
    }
}

static void test_0x218_fields_and_16_char_name(void)
{
    uint8_t d[96];
    memset(d, STALE, sizeof(d));
    const uint8_t head[18] = {0x00, 0x03, 'G', 'F', 'L', '9', '9', 0, 0, 0,
                              0xAB, 0xCD, 0xEF, 0x80, 0xBE, 0x00, 0xE6, 0x00};
    memcpy(d, head, sizeof(head));
    memcpy(d + 18, "Bambu Support For PLA", 16); // 21 chars, field holds 16

    bambubus_set_filament_t o;
    TEST_ASSERT_TRUE(bambubus_decode_set_filament_type2(d, 34, &o));
    TEST_ASSERT_EQUAL_UINT8(0, o.ams_num);
    TEST_ASSERT_EQUAL_UINT8(3, o.channel);
    TEST_ASSERT_EQUAL_STRING("GFL99", o.id);
    TEST_ASSERT_EQUAL_HEX8(0xAB, o.color_R);
    TEST_ASSERT_EQUAL_HEX8(0x80, o.color_A);
    TEST_ASSERT_EQUAL_INT16(190, o.temperature_min);
    TEST_ASSERT_EQUAL_INT16(230, o.temperature_max);
    TEST_ASSERT_EQUAL_STRING("Bambu Support Fo", o.name);
}

static void test_0x218_short_payloads(void)
{
    uint8_t d[96];
    bambubus_set_filament_t o;

    TEST_ASSERT_EQUAL_INT(18, BAMBUBUS_SET_FIL2_MIN_LEN);
    for (int n = 0; n < BAMBUBUS_SET_FIL2_MIN_LEN; n++)
    {
        fill_frame(d, (int)sizeof(d), n + 2);
        TEST_ASSERT_FALSE(bambubus_decode_set_filament_type2(d, n, &o));
    }

    // The old handler read the name from past the payload for every n below 34.
    for (int n = BAMBUBUS_SET_FIL2_MIN_LEN; n <= 40; n++)
    {
        memset(d, STALE, sizeof(d));
        memset(d, 0x00, 2);
        for (int i = 2; i < n; i++)
            d[i] = (uint8_t)('A' + (i % 26));

        TEST_ASSERT_TRUE(bambubus_decode_set_filament_type2(d, n, &o));
        const int name_in_payload = (n - 18) < 16 ? (n - 18) : 16;
        TEST_ASSERT_EQUAL_INT(name_in_payload, (int)strlen(o.name));
        for (int i = 0; i < 20; i++)
            TEST_ASSERT_NOT_EQUAL((int)STALE, (int)(uint8_t)o.name[i]);
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_0x08_frames_of_44_bytes_or_more_decode_as_before);
    RUN_TEST(test_0x08_fields_at_their_offsets);
    RUN_TEST(test_0x08_name_keeps_19_chars);
    RUN_TEST(test_0x08_too_short_for_the_fixed_fields_is_dropped);
    RUN_TEST(test_0x08_name_stops_at_the_payload_end);
    RUN_TEST(test_0x08_never_reads_past_the_payload);
    RUN_TEST(test_0x218_payloads_of_34_bytes_or_more_decode_as_before);
    RUN_TEST(test_0x218_fields_and_16_char_name);
    RUN_TEST(test_0x218_short_payloads);
    return UNITY_END();
}
