// Host tests for src/ahub_frame.h, the checks ahub_bus.cpp runs on AHUB (0x33) frames: the active
// channel a set request stores is a channel (0-3) or none (0xFF).

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

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_a_channel_or_none_is_stored_as_sent);
    RUN_TEST(test_any_other_value_is_stored_as_none);
    return UNITY_END();
}
