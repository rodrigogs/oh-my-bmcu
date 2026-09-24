// Host tests for src/ws2812_frame.h: a strip must be redrawn only when what the LEDs show would
// change, so the per-pass baseline-then-state writes of Motion_control cost no redraw in a steady
// state, while every real colour change and blink phase is still sent. The redraw throttle must
// leave every strip's data pin low for the WS2812 reset time between frames, as updata() no longer
// waits for it.

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

// ---- redraw throttle (ws2812_throttle_t), as used by RGB_update() ----

#define TICKS_PER_US 18u                           // SysTick = HCLK/8 at 144 MHz
#define MIN_GAP      (10u * 1000u * TICKS_PER_US)  // RGB_update(): time_hw_tpms * 10
#define LATCH_TICKS  (280u * TICKS_PER_US)         // WS2812B-2020 reset time, the longest one quoted
#define TBIT_TICKS   22u                           // WS2812_TBIT_TICKS in ws2812.cpp

static void test_throttle_first_redraw_is_immediate_then_spaced(void)
{
    ws2812_throttle_t t = {0u, 0u};
    const uint32_t t0 = 0x12345678u;

    TEST_ASSERT_TRUE(ws2812_throttle_due(&t, t0, MIN_GAP));
    TEST_ASSERT_FALSE(ws2812_throttle_due(&t, t0, MIN_GAP));
    TEST_ASSERT_FALSE(ws2812_throttle_due(&t, t0 + MIN_GAP - 1u, MIN_GAP));
    TEST_ASSERT_TRUE(ws2812_throttle_due(&t, t0 + MIN_GAP, MIN_GAP));
    TEST_ASSERT_FALSE(ws2812_throttle_due(&t, t0 + 2u * MIN_GAP - 1u, MIN_GAP));
}

// The old marker was last == 0: a redraw that started at tick 0 let the next one start at once.
static void test_throttle_redraw_at_tick_zero_still_throttles(void)
{
    ws2812_throttle_t t = {0u, 0u};

    TEST_ASSERT_TRUE(ws2812_throttle_due(&t, 0u, MIN_GAP));
    TEST_ASSERT_FALSE(ws2812_throttle_due(&t, 1u, MIN_GAP));
    TEST_ASSERT_FALSE(ws2812_throttle_due(&t, MIN_GAP - 1u, MIN_GAP));
    TEST_ASSERT_TRUE(ws2812_throttle_due(&t, MIN_GAP, MIN_GAP));
}

static void test_throttle_across_the_systick_wrap(void)
{
    ws2812_throttle_t t = {0u, 0u};
    uint32_t t0 = 0xFFFFFFFFu - 100u; // deadline past the wrap

    TEST_ASSERT_TRUE(ws2812_throttle_due(&t, t0, MIN_GAP));
    TEST_ASSERT_FALSE(ws2812_throttle_due(&t, t0 + 200u, MIN_GAP));
    TEST_ASSERT_FALSE(ws2812_throttle_due(&t, t0 + MIN_GAP - 1u, MIN_GAP));
    TEST_ASSERT_TRUE(ws2812_throttle_due(&t, t0 + MIN_GAP, MIN_GAP));

    // Deadline before the wrap, first call after it.
    t0 = 0xFFFFFFFFu - MIN_GAP - 100u;
    TEST_ASSERT_TRUE(ws2812_throttle_due(&t, t0, MIN_GAP));
    TEST_ASSERT_FALSE(ws2812_throttle_due(&t, t0 + MIN_GAP - 1u, MIN_GAP));
    TEST_ASSERT_TRUE(ws2812_throttle_due(&t, t0 + MIN_GAP + 200u, MIN_GAP));
}

// main.cpp: SYS_RGB (1 LED) and RGBOUT[0..3] (2 LEDs each), one data pin each.
#define N_STRIPS 5

static ws2812_frame_t sim_strip[N_STRIPS];
static uint32_t sim_frame_end[N_STRIPS];
static uint8_t sim_drawn[N_STRIPS];
static uint32_t sim_min_low;
static unsigned sim_passes;

static void sim_init(void)
{
    for (int k = 0; k < N_STRIPS; k++)
    {
        ws2812_frame_init(&sim_strip[k], k == 0 ? 1u : 2u);
        sim_drawn[k] = 0u;
    }
    sim_min_low = 0xFFFFFFFFu;
    sim_passes = 0u;
}

// RGB_update() on a simulated clock: the throttle, then each dirty strip in turn, IRQs off for its
// bits (start aligned to the next tick) and 2 us of ISRs served before the next strip. Records how
// long each pin was low between its previous frame and this one.
static void sim_rgb_update(ws2812_throttle_t *t, uint32_t *now)
{
    bool dirty = false;
    for (int k = 0; k < N_STRIPS; k++)
        dirty = dirty || ws2812_frame_dirty(&sim_strip[k]);
    if (!dirty || !ws2812_throttle_due(t, *now, MIN_GAP))
        return;

    sim_passes++;
    for (int k = 0; k < N_STRIPS; k++)
    {
        if (!ws2812_frame_dirty(&sim_strip[k])) continue;
        *now += 1u;
        if (sim_drawn[k] && (uint32_t)(*now - sim_frame_end[k]) < sim_min_low)
            sim_min_low = (uint32_t)(*now - sim_frame_end[k]);
        *now += (uint32_t)sim_strip[k].num * 24u * TBIT_TICKS;
        sim_frame_end[k] = *now;
        sim_drawn[k] = 1u;
        ws2812_frame_mark_shown(&sim_strip[k]);
        *now += 2u * TICKS_PER_US;
    }
}

// Worst case for the latch: RGB_update() runs back to back (the main loop, or a calibration/blink
// loop spinning on it) for 2 s and the LEDs change on every call. After a pass that redrew all five
// strips only the last one changes, so its next frame starts right at the throttle deadline while
// its previous one ended a whole pass after the previous deadline.
static uint32_t sim_tight_loop(uint32_t start)
{
    ws2812_throttle_t t = {0u, 0u};
    uint32_t now = start;
    sim_init();
    for (uint32_t i = 0; (uint32_t)(now - start) < 2000u * 1000u * TICKS_PER_US; i++)
    {
        const int first = (sim_passes & 1u) ? N_STRIPS - 1 : 0;
        for (int k = first; k < N_STRIPS; k++)
            for (uint8_t led = 0; led < sim_strip[k].num; led++)
                ws2812_frame_set(&sim_strip[k], led, (i * 5u + (uint32_t)k) & 0xFFFFFFu);
        sim_rgb_update(&t, &now);
        now += 1u * TICKS_PER_US;
    }
    return sim_min_low;
}

static void test_every_strip_gets_its_reset_time_in_a_tight_rgb_update_loop(void)
{
    // First redraw at tick 0, and a run across the 2^32 SysTick wrap.
    const uint32_t starts[2] = {0u, 0xFFFFFFFFu - 1000u * 1000u * TICKS_PER_US};
    for (int r = 0; r < 2; r++)
    {
        const uint32_t min_low = sim_tight_loop(starts[r]);
        // About 200 redraw passes in 2 s: the throttle rate, no pass skipped.
        TEST_ASSERT_UINT_WITHIN(2u, 200u, sim_passes);
        TEST_ASSERT_TRUE(min_low >= LATCH_TICKS);
        // A pass (bit time plus ISRs) takes well under 0.5 ms of the 10 ms gap.
        TEST_ASSERT_TRUE(min_low > MIN_GAP - 500u * TICKS_PER_US);
        TEST_ASSERT_TRUE(min_low < MIN_GAP - 200u * TICKS_PER_US); // the worst case was hit
    }
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
    RUN_TEST(test_throttle_first_redraw_is_immediate_then_spaced);
    RUN_TEST(test_throttle_redraw_at_tick_zero_still_throttles);
    RUN_TEST(test_throttle_across_the_systick_wrap);
    RUN_TEST(test_every_strip_gets_its_reset_time_in_a_tight_rgb_update_loop);
    return UNITY_END();
}
