// Host tests for src/bambubus_frame_len.h: every short frame the printer is known to send, and every
// frame one byte short of that but still holding the fields its handler uses, must pass; a frame
// whose handler would read a field from its CRC16 or past its end must be dropped; the check itself
// reads no byte past the frame; and commands without a minimum are left as they were.

#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "bambubus_frame_len.h"

// Stands for bytes past the frame: whatever an earlier, longer frame left in the 1280-byte RX buffer.
#define STALE 0xAAu

void setUp(void) {}
void tearDown(void) {}

// ---- bambu_bus_ams.cpp at this commit: the printer's 0x03 request, verbatim ----
// 3D C5 0C C8 03 00 07 00 7F 02 36 54
// ---- end of the bambu_bus_ams.cpp copy ----
static const uint8_t kPrinterMotion[] = {0x3D, 0xC5, 0x0C, 0xC8, 0x03, 0x00, 0x07, 0x00, 0x7F, 0x02, 0x36, 0x54};

// ---- bambu_bus_ams.cpp at this commit: the printer's 0x04 request, verbatim ----
// 3D C5 0D F1 04 00 01 00 03 FF 00 B2 C4
// ---- end of the bambu_bus_ams.cpp copy ----
static const uint8_t kPrinterPoll[] = {0x3D, 0xC5, 0x0D, 0xF1, 0x04, 0x00, 0x01, 0x00, 0x03, 0xFF, 0x00, 0xB2, 0xC4};

// A short frame of len bytes for command cmd, subtype sub (buf[5], if inside), the rest of the payload
// zero, STALE after it. Only buf[4] and buf[5] matter to the check.
static void frame(uint8_t *buf, int size, uint8_t cmd, uint8_t sub, int len)
{
    memset(buf, STALE, (size_t)size);
    memset(buf, 0x00, (size_t)len);
    buf[0] = 0x3D;
    buf[1] = 0xC5;
    buf[2] = (uint8_t)len;
    buf[4] = cmd;
    if (len > 5) buf[5] = sub;
}

// The captured frames pass, and so do they without the byte their handlers ignore (buf[9] of 0x03,
// buf[10] of 0x04): the minima are one byte below what the printer sends.
static void test_the_captured_frames_pass(void)
{
    TEST_ASSERT_TRUE(bambubus_short_frame_len_ok(kPrinterMotion, (int)sizeof(kPrinterMotion)));
    TEST_ASSERT_TRUE(bambubus_short_frame_len_ok(kPrinterPoll, (int)sizeof(kPrinterPoll)));

    TEST_ASSERT_EQUAL_INT((int)sizeof(kPrinterMotion) - 1, BAMBUBUS_MOTION_MIN_LEN);
    TEST_ASSERT_EQUAL_INT((int)sizeof(kPrinterPoll) - 1, BAMBUBUS_STU_MOTION_MIN_LEN);
    TEST_ASSERT_TRUE(bambubus_short_frame_len_ok(kPrinterMotion, BAMBUBUS_MOTION_MIN_LEN));
    TEST_ASSERT_TRUE(bambubus_short_frame_len_ok(kPrinterPoll, BAMBUBUS_STU_MOTION_MIN_LEN));
}

// 0x03: the motion flag at buf[8] must be inside the payload, so 11 bytes; 6..10 are dropped.
static void test_motion_minimum(void)
{
    uint8_t buf[64];
    TEST_ASSERT_EQUAL_INT(11, BAMBUBUS_MOTION_MIN_LEN);
    for (int len = 6; len < 64; len++)
    {
        frame(buf, (int)sizeof(buf), 0x03, 0x00, len);
        TEST_ASSERT_EQUAL_MESSAGE(len >= 11, bambubus_short_frame_len_ok(buf, len), "0x03");
    }
}

// 0x04: the channel at buf[9] must be inside the payload, so 12 bytes; 6..11 are dropped.
static void test_stu_motion_minimum(void)
{
    uint8_t buf[64];
    TEST_ASSERT_EQUAL_INT(12, BAMBUBUS_STU_MOTION_MIN_LEN);
    for (int len = 6; len < 64; len++)
    {
        frame(buf, (int)sizeof(buf), 0x04, 0x01, len);
        TEST_ASSERT_EQUAL_MESSAGE(len >= 12, bambubus_short_frame_len_ok(buf, len), "0x04");
    }
}

// 0x05: a probe needs its subtype (8 bytes), a confirm the unit and the prefix and 16 ID bytes it
// echoes (26 bytes; the one the host tests send is 29). Other subtypes read only buf[5].
static void test_online_detect_minimum(void)
{
    uint8_t buf[64];
    TEST_ASSERT_EQUAL_INT(8, BAMBUBUS_ONLINE_DETECT_MIN_LEN);
    TEST_ASSERT_EQUAL_INT(26, BAMBUBUS_ONLINE_CONFIRM_MIN_LEN);
    for (int len = 6; len < 64; len++)
    {
        frame(buf, (int)sizeof(buf), 0x05, 0x00, len);
        TEST_ASSERT_EQUAL_MESSAGE(len >= 8, bambubus_short_frame_len_ok(buf, len), "0x05 probe");
        frame(buf, (int)sizeof(buf), 0x05, 0x01, len);
        TEST_ASSERT_EQUAL_MESSAGE(len >= 26, bambubus_short_frame_len_ok(buf, len), "0x05 confirm");
        frame(buf, (int)sizeof(buf), 0x05, 0x02, len);
        TEST_ASSERT_EQUAL_MESSAGE(len >= 8, bambubus_short_frame_len_ok(buf, len), "0x05 other");
    }
}

// A 7-byte 0x05 has no subtype: buf[5] is its CRC16, and the check must not read it as one, whatever
// it holds (at 0x00 it used to be answered as a probe).
static void test_a_0x05_without_its_subtype_is_dropped(void)
{
    uint8_t buf[16];
    for (int b = 0; b < 256; b++)
    {
        frame(buf, (int)sizeof(buf), 0x05, 0x00, 7);
        buf[5] = (uint8_t)b;
        TEST_ASSERT_FALSE(bambubus_short_frame_len_ok(buf, 7));
    }
}

// The check looks at nothing past len: the same answer with the buffer after the frame all 0x00 or
// all STALE.
static void test_bytes_past_the_frame_do_not_matter(void)
{
    const uint8_t cmds[] = {0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x20, 0x41};
    const uint8_t subs[] = {0x00, 0x01, 0x02};
    for (unsigned c = 0; c < sizeof(cmds); c++)
        for (unsigned s = 0; s < sizeof(subs); s++)
            for (int len = 6; len < 40; len++)
            {
                uint8_t a[48], b[48];
                frame(a, (int)sizeof(a), cmds[c], subs[s], len);
                memcpy(b, a, sizeof(b));
                memset(b + len, 0x00, sizeof(b) - (size_t)len);
                TEST_ASSERT_EQUAL(bambubus_short_frame_len_ok(a, len), bambubus_short_frame_len_ok(b, len));
            }
}

// Commands without a minimum here pass at any length, as before: 0x06, 0x07 and 0x20 have no handler
// (the RX parser takes the heartbeat itself) and 0x08 checks its own (bambubus_set_filament.h).
static void test_other_commands_are_left_as_they_were(void)
{
    const uint8_t cmds[] = {0x00, 0x01, 0x02, 0x06, 0x07, 0x08, 0x20, 0x41, 0xFF};
    uint8_t buf[16];
    for (unsigned c = 0; c < sizeof(cmds); c++)
        for (int len = 6; len <= 12; len++)
        {
            frame(buf, (int)sizeof(buf), cmds[c], 0x00, len);
            TEST_ASSERT_TRUE(bambubus_short_frame_len_ok(buf, len));
        }
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_the_captured_frames_pass);
    RUN_TEST(test_motion_minimum);
    RUN_TEST(test_stu_motion_minimum);
    RUN_TEST(test_online_detect_minimum);
    RUN_TEST(test_a_0x05_without_its_subtype_is_dropped);
    RUN_TEST(test_bytes_past_the_frame_do_not_matter);
    RUN_TEST(test_other_commands_are_left_as_they_were);
    return UNITY_END();
}
