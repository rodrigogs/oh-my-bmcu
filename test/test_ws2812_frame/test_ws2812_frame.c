// Host tests for src/ws2812_frame.h: a strip must be redrawn only when what the LEDs show would
// change, so the per-pass baseline-then-state writes of Motion_control cost no redraw in a steady
// state, while every real colour change and blink phase is still sent.

#include <stdint.h>
#include <unity.h>

#include "ws2812_frame.h"

// Same packing as WS2812_class::set_RGB.
static uint32_t grb(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
}

// Colours used by Motion_control.cpp.
#define C_OFF      grb(0x00, 0x00, 0x00)
#define C_WHITE    grb(0x38, 0x35, 0x32)   // loaded baseline (stu_apply_baseline)
#define C_RED      grb(0xFF, 0x00, 0x00)   // latch / error
#define C_ON_USE   grb(0x00, 0xB0, 0xFF)
#define C_SEND_OUT grb(0x00, 0xD5, 0x2A)
#define C_PULLBACK grb(0xA0, 0x2D, 0xFF)

// RGB_update without the 10 ms throttle: send every dirty strip, count the redraws.
static unsigned rgb_update(ws2812_frame_t *s)
{
    if (!ws2812_frame_dirty(s)) return 0u;
    ws2812_frame_mark_shown(s);
    return 1u;
}

// One main-loop pass for the active channel strip (LED0 = state, LED1 = online LED):
// stu_apply_baseline writes the baseline, motor_motion_switch overrides it with the state colour,
// then the online LED is written.
static void pass_active_channel(ws2812_frame_t *s, uint32_t baseline, uint32_t state, uint32_t online)
{
    ws2812_frame_set(s, 0, baseline);
    ws2812_frame_set(s, 0, state);
    ws2812_frame_set(s, 1, online);
}

static ws2812_frame_t strip;

void setUp(void)
{
    ws2812_frame_init(&strip, 2);
}

void tearDown(void) {}

static void test_new_strip_is_sent_once_even_if_black(void)
{
    // LED contents are unknown after init, so the boot frame (all black) must still go out.
    ws2812_frame_set(&strip, 0, C_OFF);
    TEST_ASSERT_TRUE(ws2812_frame_dirty(&strip));
    TEST_ASSERT_EQUAL_UINT(1u, rgb_update(&strip));
    TEST_ASSERT_FALSE(ws2812_frame_dirty(&strip));
    TEST_ASSERT_EQUAL_UINT(0u, rgb_update(&strip));
}

static void test_steady_on_use_with_baseline_overwrite_causes_no_redraw(void)
{
    // The defect: every pass writes white then blue into LED0. Comparing against the last
    // requested value made the strip dirty on every pass (1000 redraws here); only the first
    // pass may redraw.
    unsigned draws = 0u;
    for (int i = 0; i < 1000; i++)
    {
        pass_active_channel(&strip, C_WHITE, C_ON_USE, C_OFF);
        draws += rgb_update(&strip);
    }
    TEST_ASSERT_EQUAL_UINT(1u, draws);
    TEST_ASSERT_EQUAL_HEX32(C_ON_USE, strip.shown[0]);
    TEST_ASSERT_EQUAL_HEX32(C_OFF, strip.shown[1]);
}

static void test_state_change_is_sent_once(void)
{
    pass_active_channel(&strip, C_WHITE, C_ON_USE, C_OFF);
    rgb_update(&strip);

    unsigned draws = 0u;
    for (int i = 0; i < 100; i++)
    {
        pass_active_channel(&strip, C_WHITE, C_SEND_OUT, C_OFF);
        draws += rgb_update(&strip);
    }
    TEST_ASSERT_EQUAL_UINT(1u, draws);
    TEST_ASSERT_EQUAL_HEX32(C_SEND_OUT, strip.shown[0]);
}

static void test_online_led_change_is_sent_while_led0_is_overwritten(void)
{
    pass_active_channel(&strip, C_WHITE, C_ON_USE, C_OFF);
    rgb_update(&strip);

    pass_active_channel(&strip, C_WHITE, C_ON_USE, grb(0x10, 0x00, 0x00));
    TEST_ASSERT_TRUE(ws2812_frame_dirty(&strip));
    TEST_ASSERT_EQUAL_UINT(1u, rgb_update(&strip));
    TEST_ASSERT_EQUAL_HEX32(grb(0x10, 0x00, 0x00), strip.shown[1]);
}

static void test_change_reverted_before_update_needs_no_redraw(void)
{
    pass_active_channel(&strip, C_WHITE, C_ON_USE, C_OFF);
    rgb_update(&strip);

    ws2812_frame_set(&strip, 0, C_RED);
    TEST_ASSERT_TRUE(ws2812_frame_dirty(&strip));
    ws2812_frame_set(&strip, 0, C_ON_USE);
    TEST_ASSERT_FALSE(ws2812_frame_dirty(&strip));
}

static void test_throttled_change_stays_pending_until_sent(void)
{
    pass_active_channel(&strip, C_WHITE, C_ON_USE, C_OFF);
    rgb_update(&strip);

    // RGB_update skips the redraw (10 ms throttle) for several passes; the change must not be lost.
    for (int i = 0; i < 5; i++)
    {
        pass_active_channel(&strip, C_WHITE, C_SEND_OUT, C_OFF);
        TEST_ASSERT_TRUE(ws2812_frame_dirty(&strip));
    }
    TEST_ASSERT_EQUAL_UINT(1u, rgb_update(&strip));
    TEST_ASSERT_EQUAL_HEX32(C_SEND_OUT, strip.shown[0]);
}

static void test_latch_blink_is_sent_on_every_phase_change_only(void)
{
    // g_on_use_low_latch set during pull_back: stu_apply_baseline writes red, then
    // MC_STU_RGB_set_latch(..., blink=1) alternates red / purple every 1000 ms. 1 ms passes.
    unsigned draws = 0u;
    for (uint32_t now_ms = 0; now_ms < 5000u; now_ms++)
    {
        const uint32_t state = ((now_ms / 1000u) & 1u) ? C_RED : C_PULLBACK;
        pass_active_channel(&strip, C_RED, state, C_OFF);
        draws += rgb_update(&strip);
        TEST_ASSERT_EQUAL_HEX32(state, strip.shown[0]);
    }
    // First frame plus the phase changes at 1000, 2000, 3000 and 4000 ms.
    TEST_ASSERT_EQUAL_UINT(5u, draws);
}

static void test_calibration_blink_sends_every_on_and_off(void)
{
    // MC_PULL_calibration blink_all(): online LED on, RGB_update, off, RGB_update.
    rgb_update(&strip);

    unsigned draws = 0u;
    for (int k = 0; k < 3; k++)
    {
        ws2812_frame_set(&strip, 1, grb(0x10, 0x10, 0x00));
        draws += rgb_update(&strip);
        TEST_ASSERT_EQUAL_HEX32(grb(0x10, 0x10, 0x00), strip.shown[1]);
        ws2812_frame_set(&strip, 1, C_OFF);
        draws += rgb_update(&strip);
        TEST_ASSERT_EQUAL_HEX32(C_OFF, strip.shown[1]);
    }
    TEST_ASSERT_EQUAL_UINT(6u, draws);
}

static void test_out_of_range_index_is_ignored(void)
{
    ws2812_frame_t sys;
    ws2812_frame_init(&sys, 1);
    rgb_update(&sys);

    ws2812_frame_set(&sys, 1, C_RED);
    ws2812_frame_set(&sys, WS2812_FRAME_MAX_LEDS, C_RED);
    TEST_ASSERT_FALSE(ws2812_frame_dirty(&sys));
    TEST_ASSERT_EQUAL_HEX32(0u, sys.want[1]);
}

static void test_clear_forces_a_redraw(void)
{
    rgb_update(&strip);
    ws2812_frame_clear(&strip);
    TEST_ASSERT_TRUE(ws2812_frame_dirty(&strip));
    TEST_ASSERT_EQUAL_UINT(1u, rgb_update(&strip));
    TEST_ASSERT_FALSE(ws2812_frame_dirty(&strip));
}

static void test_init_clamps_led_count(void)
{
    ws2812_frame_t s;
    ws2812_frame_init(&s, 9);
    TEST_ASSERT_EQUAL_UINT8(WS2812_FRAME_MAX_LEDS, s.num);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_new_strip_is_sent_once_even_if_black);
    RUN_TEST(test_steady_on_use_with_baseline_overwrite_causes_no_redraw);
    RUN_TEST(test_state_change_is_sent_once);
    RUN_TEST(test_online_led_change_is_sent_while_led0_is_overwritten);
    RUN_TEST(test_change_reverted_before_update_needs_no_redraw);
    RUN_TEST(test_throttled_change_stays_pending_until_sent);
    RUN_TEST(test_latch_blink_is_sent_on_every_phase_change_only);
    RUN_TEST(test_calibration_blink_sends_every_on_and_off);
    RUN_TEST(test_out_of_range_index_is_ignored);
    RUN_TEST(test_clear_forces_a_redraw);
    RUN_TEST(test_init_clamps_led_count);
    return UNITY_END();
}
