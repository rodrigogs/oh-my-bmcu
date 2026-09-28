// Host tests for src/ahub_frame.h, the checks ahub_bus.cpp runs on AHUB (0x33) frames: the active
// channel a set request stores is a channel (0-3) or none (0xFF), a set request is applied only if
// every byte its handler reads lies before the frame's CRC32, and only the short header is decoded,
// its CRC32 covering the frame the RX parser (src/_bus_hardware.h, run here) hands over.

#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "_bus_hardware.h"
#include "ahub_frame.h"
#include "bus_rx_frames.h"
#include "crc_bus.h"

static const uint32_t FRAME_GAP_TICKS = 400u * 18u; // 400 us of silence between frames

static _bus_port_deal port;
static uint32_t tick;

void bambubus_heartbeat_seen_fast(void) {}

static void no_send(uint8_t *, uint16_t) {}

void setUp(void)
{
    port.init(no_send);
    tick = 0x10000000u;
}

void tearDown(void) {}

static void test_a_channel_or_none_is_stored_as_sent(void)
{
    for (uint8_t ch = 0; ch < 4u; ch++)
        TEST_ASSERT_EQUAL_HEX8(ch, ahub_now_filament_num(ch));
    TEST_ASSERT_EQUAL_HEX8(0xFFu, ahub_now_filament_num(0xFFu));
}

// Every other byte would be neither a channel (< 4) nor none (0xFF) for the code that reads it.
static void test_any_other_value_is_stored_as_none(void)
{
    for (unsigned v = 4u; v < 0xFFu; v++)
        TEST_ASSERT_EQUAL_HEX8(0xFFu, ahub_now_filament_num((uint8_t)v));
}

// Every frame the RX parser hands over is length * 4 + 12 bytes (length: header byte 2, 0-255), the
// last 4 of them the CRC32.
static int frame_len(int length) { return length * 4 + 12; }

// ---- adapted from ahub_bus.cpp: the last byte each set handler reads (data_ptr = buf + 4) ----
// ---- anchor: ahubus_slave_get_package_set from /data_ptr = buf \+ 4/ to /default:/ ----
// filament_info: the 44-byte info at data_ptr[4..47], the channel at data_ptr[48].
static int filament_info_last_read(void) { return 4 + 48; }
// dryer_stu: dryer_power..dryer_time_left at data_ptr[4..7], the channel at data_ptr[8].
static int dryer_stu_last_read(void) { return 4 + 8; }
// all_filament_stu: count 6-byte entries from data_ptr + 4; count 0 reads nothing past buf[7].
static int all_filament_stu_last_read(int count) { return count == 0 ? 7 : 4 + 4 + 6 * count - 1; }

static void test_filament_info_needs_a_frame_of_60_bytes(void)
{
    for (int length = 0; length <= 255; length++)
    {
        const int len = frame_len(length);
        const bool fits = ahub_set_data_fits(len, AHUB_SET_FILAMENT_INFO_LEN);
        TEST_ASSERT_EQUAL(length >= 12, fits);
        TEST_ASSERT_EQUAL(filament_info_last_read() < len - 4, fits);
    }
    TEST_ASSERT_TRUE(ahub_set_data_fits(60, AHUB_SET_FILAMENT_INFO_LEN));
    TEST_ASSERT_FALSE(ahub_set_data_fits(56, AHUB_SET_FILAMENT_INFO_LEN)); // channel in the CRC32
}

static void test_dryer_stu_needs_a_frame_of_20_bytes(void)
{
    for (int length = 0; length <= 255; length++)
    {
        const int len = frame_len(length);
        const bool fits = ahub_set_data_fits(len, AHUB_SET_DRYER_STU_LEN);
        TEST_ASSERT_EQUAL(length >= 2, fits);
        TEST_ASSERT_EQUAL(dryer_stu_last_read() < len - 4, fits);
    }
    TEST_ASSERT_TRUE(ahub_set_data_fits(20, AHUB_SET_DRYER_STU_LEN));
    TEST_ASSERT_FALSE(ahub_set_data_fits(16, AHUB_SET_DRYER_STU_LEN)); // channel in the CRC32
}

// Accepted exactly when all count entries lie before the CRC32: 6 * count <= 4 * length.
static void test_all_filament_stu_needs_every_entry_in_the_frame(void)
{
    for (int length = 0; length <= 255; length++)
    {
        const int len = frame_len(length);
        for (int count = 0; count <= 255; count++)
        {
            const bool fits = ahub_set_data_fits(len, count * AHUB_SET_FILAMENT_STU_LEN);
            TEST_ASSERT_EQUAL(6 * count <= 4 * length, fits);
            TEST_ASSERT_EQUAL(all_filament_stu_last_read(count) < len - 4, fits);
        }
    }
    // The four AMS of a hub in the shortest frame that holds them: 24 data bytes, length 6.
    TEST_ASSERT_TRUE(ahub_set_data_fits(frame_len(6), 4 * AHUB_SET_FILAMENT_STU_LEN));
    TEST_ASSERT_FALSE(ahub_set_data_fits(frame_len(5), 4 * AHUB_SET_FILAMENT_STU_LEN));
    TEST_ASSERT_TRUE(ahub_set_data_fits(frame_len(0), 0));
}

// A count of 255 in a 12-byte frame made the handler read up to buf[1537], past the 1280-byte RX
// buffer, and store what it found there as the channels' motion.
static void test_no_count_reads_past_the_rx_buffer(void)
{
    for (int count = 0; count <= 255; count++)
    {
        const bool fits = ahub_set_data_fits(frame_len(255), count * AHUB_SET_FILAMENT_STU_LEN);
        if (fits)
            TEST_ASSERT_LESS_THAN_INT(1280, all_filament_stu_last_read(count));
        TEST_ASSERT_EQUAL(count <= 170, fits);
    }
}

// An AHUB frame of header length `length`: length * 4 + 12 bytes. Short header: 33, flag, length,
// CRC8. Long header (flag bit 7 clear), as the RX parser reads it: length at byte 4, CRC8 of bytes
// 0-5 at byte 6, byte 2 free (b2). The CRC32 word is not checked by the parser and left as filler.
static int make_ahub(uint8_t *out, uint8_t flag, int length, uint8_t b2)
{
    const int len = length * 4 + 12;
    for (int i = 0; i < len; i++)
        out[i] = (uint8_t)(0x40u + ((i * 7) & 0x3Fu));
    out[0] = 0x33;
    out[1] = flag;
    if (flag & 0x80u)
    {
        out[2] = (uint8_t)length;
        out[3] = bus_crc8(out, 3);
    }
    else
    {
        out[2] = b2;
        out[4] = (uint8_t)length;
        out[6] = bus_crc8(out, 6);
    }
    return len;
}

// Fed after a quiet line; returns the length the parser handed over (0: none).
static int parse(const uint8_t *f, int len)
{
    tick += FRAME_GAP_TICKS;
    for (int i = 0; i < len; i++)
    {
        tick += BYTE_TICKS;
        port.rx_byte(f[i], tick, false);
    }
    const int got = port.recv_data_len;
    if (got != 0)
        TEST_ASSERT_EQUAL_UINT8((uint8_t)_bus_data_type::ahub_bus, (uint8_t)port.bus_package_type);
    port.recv_data_len = 0;
    port.bus_package_type = _bus_data_type::none;
    return got;
}

static void test_flag_bit_7_tells_the_short_header(void)
{
    uint8_t f[8] = {0x33};
    for (unsigned flag = 0; flag <= 0xFFu; flag++)
    {
        f[1] = (uint8_t)flag;
        TEST_ASSERT_EQUAL((flag & 0x80u) != 0u, ahub_frame_short_header(f));
    }
}

// The CRC32 covers every word but the last of each short-header frame the parser hands over.
static void test_the_crc32_covers_each_short_frame_the_parser_hands_over(void)
{
    static const uint8_t flags[] = {0x80, 0xC0, 0xC5, 0xFF};
    uint8_t f[1280];
    for (unsigned k = 0; k < sizeof(flags); k++)
        for (int length = 0; length <= 255; length++)
        {
            const int len = make_ahub(f, flags[k], length, 0);
            TEST_ASSERT_EQUAL_INT(len, parse(f, len));
            TEST_ASSERT_TRUE(ahub_frame_short_header(port.bus_recv_data_ptr));
            TEST_ASSERT_EQUAL_UINT32((uint32_t)len / 4u - 1u, ahub_frame_crc_words(port.bus_recv_data_ptr));
        }
}

// The parser hands long-header frames over, sized from byte 4. Counted from byte 2, the CRC32 missed
// them unless byte 2 equalled byte 4; then byte 4, their length, was taken as the command (1-3:
// heartbeat, query, set) and the handlers read short-header offsets. Now every one is dropped.
static void test_long_header_frames_are_dropped(void)
{
    static const uint8_t flags[] = {0x00, 0x05, 0x40, 0x7F};
    uint8_t f[1280];
    for (unsigned k = 0; k < sizeof(flags); k++)
        for (int length = 0; length <= 255; length++)
        {
            const uint8_t b2s[] = {(uint8_t)length, (uint8_t)(length ^ 0x01u)};
            for (unsigned j = 0; j < sizeof(b2s); j++)
            {
                const int len = make_ahub(f, flags[k], length, b2s[j]);
                TEST_ASSERT_EQUAL_INT(len, parse(f, len));
                TEST_ASSERT_FALSE(ahub_frame_short_header(port.bus_recv_data_ptr));
                // The CRC32 counted from byte 2 spans this frame only when byte 2 is its length.
                TEST_ASSERT_EQUAL(j == 0, ahub_frame_crc_words(port.bus_recv_data_ptr) == (uint32_t)len / 4u - 1u);
            }
        }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_a_channel_or_none_is_stored_as_sent);
    RUN_TEST(test_any_other_value_is_stored_as_none);
    RUN_TEST(test_filament_info_needs_a_frame_of_60_bytes);
    RUN_TEST(test_dryer_stu_needs_a_frame_of_20_bytes);
    RUN_TEST(test_all_filament_stu_needs_every_entry_in_the_frame);
    RUN_TEST(test_no_count_reads_past_the_rx_buffer);
    RUN_TEST(test_flag_bit_7_tells_the_short_header);
    RUN_TEST(test_the_crc32_covers_each_short_frame_the_parser_hands_over);
    RUN_TEST(test_long_header_frames_are_dropped);
    return UNITY_END();
}
