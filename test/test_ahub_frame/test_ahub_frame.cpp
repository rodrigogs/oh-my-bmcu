// Host tests for src/ahub_frame.h, the checks ahub_bus.cpp runs on AHUB (0x33) frames: the active
// channel a set request stores is a channel (0-3) or none (0xFF), and a set request is applied only
// if every byte its handler reads lies before the frame's CRC32.

#include <stdint.h>
#include <unity.h>

#include "ahub_frame.h"

void setUp(void) {}
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

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_a_channel_or_none_is_stored_as_sent);
    RUN_TEST(test_any_other_value_is_stored_as_none);
    RUN_TEST(test_filament_info_needs_a_frame_of_60_bytes);
    RUN_TEST(test_dryer_stu_needs_a_frame_of_20_bytes);
    RUN_TEST(test_all_filament_stu_needs_every_entry_in_the_frame);
    RUN_TEST(test_no_count_reads_past_the_rx_buffer);
    return UNITY_END();
}
