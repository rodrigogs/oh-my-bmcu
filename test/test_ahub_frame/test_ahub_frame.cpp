// Host tests for the checks ahub_bus.cpp runs on AHUB (0x33) frames: a set request is applied only if
// every byte its handler reads lies before the frame's CRC32 (V11's length literals), an
// all_filament_stu request is applied only if every entry's active channel is a channel (0-3) or none
// (0xFF), and only the short header is decoded (src/ahub_frame.h), its CRC32 covering the frame the RX
// parser (src/_bus_hardware.h, run here) hands over.

#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "_bus_hardware.h"
#include "ahub_frame.h"
#include "bus_rx_frames.h"
#include "crc_bus.h"

static _bus_port_deal port;

void bambubus_heartbeat_seen_fast(void) {}

static bool no_send(uint8_t *, uint16_t) { return true; }

void setUp(void) { port.init(no_send); }

void tearDown(void) {}

// Every frame the RX parser hands over is length * 4 + 12 bytes (length: header byte 2, 0-255), the
// last 4 of them the CRC32.
static int frame_len(int length) { return length * 4 + 12; }

// ---- adapted from ahub_bus.cpp: the set handlers' length and channel checks and the last byte each reads (data_ptr = buf + 4) ----
// ---- anchor: ahubus_slave_get_package_set from /data_ptr = buf \+ 4/ to /default:/ ----
// filament_info: the 44-byte info at data_ptr[4..47], the channel at data_ptr[48].
static bool filament_info_accepted(int length) { return !(length < 57); }
static int filament_info_last_read(void) { return 4 + 48; }
// dryer_stu: dryer_power..dryer_time_left at data_ptr[4..7], the channel at data_ptr[8].
static bool dryer_stu_accepted(int length) { return !(length < 17); }
static int dryer_stu_last_read(void) { return 4 + 8; }
// all_filament_stu: count 6-byte entries from data_ptr + 4 (AMS address, active channel, four motions);
// count 0 reads nothing past buf[7]. Every entry's channel is checked before any entry is stored.
static bool all_filament_stu_count_accepted(int length, int count) { return !(count > (length - 12) / 6); }
static int all_filament_stu_last_read(int count) { return count == 0 ? 7 : 4 + 4 + 6 * count - 1; }
static bool all_filament_stu_channels_accepted(const uint8_t *buf, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
    {
        const uint8_t channel = buf[9u + (uint32_t)i * 6u];
        if (channel >= 4u && channel != 0xFFu) return false;
    }
    return true;
}
// Where the store loop reads entry i's active channel (data_struct_ptr[1]).
static int all_filament_stu_channel_at(int i) { return 4 + 4 + 6 * i + 1; }

// The handler returns first for a frame under 12 bytes; from there on, every length a frame can have
// (the parser hands over multiples of 4, but the check does not rely on it).
static const int MIN_SET_LEN = 12;

static void test_filament_info_is_accepted_exactly_when_its_channel_is_in_the_frame(void)
{
    for (int len = MIN_SET_LEN; len <= 1280; len++)
        TEST_ASSERT_EQUAL(filament_info_last_read() < len - 4, filament_info_accepted(len));
    TEST_ASSERT_TRUE(filament_info_accepted(frame_len(12)));  // 60 bytes
    TEST_ASSERT_FALSE(filament_info_accepted(frame_len(11))); // 56: the channel in the CRC32
}

static void test_dryer_stu_is_accepted_exactly_when_its_channel_is_in_the_frame(void)
{
    for (int len = MIN_SET_LEN; len <= 1280; len++)
        TEST_ASSERT_EQUAL(dryer_stu_last_read() < len - 4, dryer_stu_accepted(len));
    TEST_ASSERT_TRUE(dryer_stu_accepted(frame_len(2)));  // 20 bytes
    TEST_ASSERT_FALSE(dryer_stu_accepted(frame_len(1))); // 16: the channel in the CRC32
}

// Accepted exactly when all count entries lie before the CRC32.
static void test_all_filament_stu_is_accepted_exactly_when_every_entry_is_in_the_frame(void)
{
    for (int len = MIN_SET_LEN; len <= 1280; len++)
        for (int count = 0; count <= 255; count++)
            TEST_ASSERT_EQUAL(all_filament_stu_last_read(count) < len - 4, all_filament_stu_count_accepted(len, count));
    // The four AMS of a hub in the shortest frame that holds them: 24 data bytes, length 6.
    TEST_ASSERT_TRUE(all_filament_stu_count_accepted(frame_len(6), 4));
    TEST_ASSERT_FALSE(all_filament_stu_count_accepted(frame_len(5), 4));
    TEST_ASSERT_TRUE(all_filament_stu_count_accepted(frame_len(0), 0));
}

// A count of 255 in a 12-byte frame made the handler read up to buf[1537], past the 1280-byte RX
// buffer, and store what it found there as the channels' motion.
static void test_no_count_reads_past_the_rx_buffer(void)
{
    for (int count = 0; count <= 255; count++)
    {
        const bool accepted = all_filament_stu_count_accepted(frame_len(255), count);
        if (accepted)
            TEST_ASSERT_LESS_THAN_INT(1280, all_filament_stu_last_read(count));
        TEST_ASSERT_EQUAL(count <= 170, accepted);
    }
}

// The active channel stored is a channel (0-3) or none (0xFF): everything that reads now_filament_num
// tells a channel by < 4 and none by 0xFF. Any other byte in any entry rejects the whole request, so
// no entry is stored; the check reads the byte the store loop stores.
static void test_all_filament_stu_takes_only_a_channel_or_none(void)
{
    uint8_t buf[12 + 6 * 4] = {0x33, 0x80, 6, 0, 0x03, 0x02, 0x00, 4};
    for (int i = 0; i < 4; i++)
        TEST_ASSERT_EQUAL_INT(9 + 6 * i, all_filament_stu_channel_at(i));

    for (unsigned v = 0; v <= 0xFFu; v++)
    {
        for (int i = 0; i < 4; i++)
            buf[all_filament_stu_channel_at(i)] = (uint8_t)i;
        buf[all_filament_stu_channel_at(2)] = (uint8_t)v;
        TEST_ASSERT_EQUAL(v < 4u || v == 0xFFu, all_filament_stu_channels_accepted(buf, 4));
        // An entry past the count is not checked.
        TEST_ASSERT_TRUE(all_filament_stu_channels_accepted(buf, 2));
    }
}

// Fed straight into the parser; returns the length it handed over (0: none) and leaves the frame at
// port.bus_recv_data_ptr until the next call.
static int parse(const uint8_t *f, int len)
{
    port.release_packet();
    for (int i = 0; i < len; i++)
        port.irq(f[i]);
    const int got = port.recv_data_len;
    if (got != 0)
        TEST_ASSERT_EQUAL_UINT8((uint8_t)_bus_data_type::ahub_bus, (uint8_t)port.bus_package_type);
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
// heartbeat, query, set) and the handlers read short-header offsets. V11's (byte 2 + 3) * 4 == length
// check lets exactly that case through, so ahub_bus.cpp drops every long-header frame.
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
    RUN_TEST(test_filament_info_is_accepted_exactly_when_its_channel_is_in_the_frame);
    RUN_TEST(test_dryer_stu_is_accepted_exactly_when_its_channel_is_in_the_frame);
    RUN_TEST(test_all_filament_stu_is_accepted_exactly_when_every_entry_is_in_the_frame);
    RUN_TEST(test_no_count_reads_past_the_rx_buffer);
    RUN_TEST(test_all_filament_stu_takes_only_a_channel_or_none);
    RUN_TEST(test_flag_bit_7_tells_the_short_header);
    RUN_TEST(test_the_crc32_covers_each_short_frame_the_parser_hands_over);
    RUN_TEST(test_long_header_frames_are_dropped);
    return UNITY_END();
}
