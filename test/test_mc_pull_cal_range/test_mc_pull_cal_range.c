// Host tests for src/mc_pull_cal_range.h: a buffer calibration step that timed out, or a first
// step too shallow to keep, must not be stored as a range so narrow that Hall noise or
// rest-position scatter trips the jam latch or the NVM-wiping recalibration gesture; the load stops
// must stay inside any real buffer's travel; a successful calibration must be stored exactly as
// before; a channel with a fallback range must be flagged in the calibration record (in bits every
// older release wrote as 0) and flashed at the end of the run and, briefly, at every later boot;
// and a run with a fallback must not end with the green success blink.
//
// MC_PULL_calibration.cpp itself is not built on the host: the capture loops, the LEDs and the
// flash calls stay there, and everything they decide goes through the functions tested here.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "mc_pull_cal_range.h"

#define CENTER 1.65f            // normalised idle reading (MC_PULL_V_OFFSET)
#define CAL_CENTER_EPS_V 0.02f  // MC_PULL_calibration.cpp: "back at rest" band = rest scatter

// Motion_control.cpp thresholds (percent of the calibrated range).
#define JAM_LATCH_PCT 40.0f          // on_use: pct < 40 -> jam latch
#define A1_HARD_STOP_PCT 95.0f       // MC_LOAD_S1_HARD_STOP_PCT (A1)
#define P1S_HARD_STOP_PCT 97.0f      // MC_LOAD_S1_HARD_STOP_PCT (P1S), the highest load stop
#define CAL_RESET_PCT_THRESH 15      // recalibration gesture, all slots empty, held 5 s
#define CAL_RESET_V_DELTA 0.10f
#define CAL_RESET_NEAR_MIN 0.03f

// Copy of pull_v_to_percent_f() in src/Motion_control.cpp (keep in sync), range passed in.
static float pull_v_to_percent(float vmin, float vmax, float v)
{
    const float c = 1.65f;

    if (vmin > 1.60f) vmin = 1.60f;
    if (vmax < 1.70f) vmax = 1.70f;
    if (vmax <= (vmin + 0.10f)) { vmin = 1.55f; vmax = 1.75f; }

    float pos01;
    if (v <= c)
    {
        float den = c - vmin;
        if (den < 0.05f) den = 0.05f;
        pos01 = 0.5f * (v - vmin) / den;
    }
    else
    {
        float den = vmax - c;
        if (den < 0.05f) den = 0.05f;
        pos01 = 0.5f + 0.5f * (v - c) / den;
    }

    if (pos01 < 0.0f) pos01 = 0.0f;
    if (pos01 > 1.0f) pos01 = 1.0f;
    return pos01 * 100.0f;
}

// Motion_control.cpp: the per-channel "hard_blue" condition of the recalibration gesture.
static bool reset_gesture_pressed(float vmin, float vmax, float v)
{
    int pct = (int)(pull_v_to_percent(vmin, vmax, v) + 0.5f);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (pct <= CAL_RESET_PCT_THRESH) ||
           (v <= (1.65f - CAL_RESET_V_DELTA)) ||
           (v <= (vmin + CAL_RESET_NEAR_MIN));
}

static uint32_t float_bits(float f)
{
    uint32_t u;
    memcpy(&u, &f, sizeof(u));
    return u;
}

// capture_minmax_one_ch_event before this fix, for the "unchanged when successful" check.
static mc_pull_cal_range_t old_range(float center_v, float vmin, float vmax, int8_t pol)
{
    mc_pull_cal_range_t r;
    if (vmin > (center_v - 0.050f)) vmin = (center_v - 0.050f);
    if (vmax < (center_v + 0.050f)) vmax = (center_v + 0.050f);
    if (vmax <= vmin + 0.10f) { vmin = center_v - 0.10f; vmax = center_v + 0.10f; }
    r.vmin = vmin;
    r.vmax = vmax;
    r.pol = pol;
    r.fallback = false;
    return r;
}

// Stored range for two steps given as mV from centre; 0 = the step timed out, which the capture
// functions report as the centre (and pol = 1 for the first step).
static mc_pull_cal_range_t outcome(int lo_mv, int hi_mv, int8_t pol)
{
    const bool ok_min = (lo_mv > 0);
    const bool ok_max = (hi_mv > 0);
    return mc_pull_cal_range(CENTER, ok_min, ok_min ? CENTER - 0.001f * (float)lo_mv : CENTER,
                             ok_min ? pol : (int8_t)1, ok_max, ok_max ? CENTER + 0.001f * (float)hi_mv : CENTER);
}

static mc_pull_cal_range_t first_step_timed_out(float cap_max)
{
    return mc_pull_cal_range(CENTER, false, CENTER, 1, true, cap_max);
}

static mc_pull_cal_range_t second_step_timed_out(float cap_min, int8_t pol)
{
    return mc_pull_cal_range(CENTER, true, cap_min, pol, false, CENTER);
}

static mc_pull_cal_range_t both_steps_timed_out(void)
{
    return mc_pull_cal_range(CENTER, false, CENTER, 1, false, CENTER);
}

// Every first-step result a run can give: a timeout, or any end 100..800 mV below centre (a step
// needs 100 mV to count); and a spread of second-step results.
#define LO_CASES 702
static int lo_case(int i) { return (i == 0) ? 0 : 99 + i; }
static const int HI_CASES[] = {0, 100, 101, 120, 149, 150, 200, 350, 800};
#define N_HI (int)(sizeof(HI_CASES) / sizeof(HI_CASES[0]))

void setUp(void) {}
void tearDown(void) {}

static void test_successful_steps_are_stored_exactly_as_before(void)
{
    // A kept low side spans at least 150 mV; sweep 151..800 mV below and 100..800 mV above
    // centre, both polarities: bit-identical to the old code, and not a fallback.
    for (int lo = 151; lo <= 800; lo += 7)
    {
        for (int hi = 100; hi <= 800; hi += 11)
        {
            for (int k = 0; k < 2; k++)
            {
                const int8_t pol = k ? (int8_t)-1 : (int8_t)1;
                const float cap_min = CENTER - 0.001f * (float)lo;
                const float cap_max = CENTER + 0.001f * (float)hi;

                const mc_pull_cal_range_t n = mc_pull_cal_range(CENTER, true, cap_min, pol, true, cap_max);
                const mc_pull_cal_range_t o = old_range(CENTER, cap_min, cap_max, pol);

                TEST_ASSERT_EQUAL_HEX32(float_bits(o.vmin), float_bits(n.vmin));
                TEST_ASSERT_EQUAL_HEX32(float_bits(o.vmax), float_bits(n.vmax));
                TEST_ASSERT_EQUAL_INT8(o.pol, n.pol);
                TEST_ASSERT_FALSE(n.fallback);
            }
        }
    }

    // The normalised end of a pol = -1 first step comes from 3.30 - raw (cal_apply_polarity).
    const float raw_max = CENTER + 0.200f;
    const mc_pull_cal_range_t n = mc_pull_cal_range(CENTER, true, 3.30f - raw_max, -1, true, CENTER + 0.3f);
    TEST_ASSERT_EQUAL_FLOAT(3.30f - raw_max, n.vmin);
    TEST_ASSERT_EQUAL_INT8(-1, n.pol);
    TEST_ASSERT_FALSE(n.fallback);
}

static void test_shallow_first_step_falls_back_like_a_timed_out_one(void)
{
    // 100..149 mV counted as a step but is too shallow to keep: the low side is the same fallback
    // as for a timeout, the channel is flagged, the measured high side and polarity are kept.
    for (int lo = 100; lo <= 149; lo++)
    {
        for (int k = 0; k < 2; k++)
        {
            const int8_t pol = k ? (int8_t)-1 : (int8_t)1;
            const float cap_min = CENTER - 0.001f * (float)lo;
            const mc_pull_cal_range_t r = mc_pull_cal_range(CENTER, true, cap_min, pol, true, 2.00f);

            TEST_ASSERT_FALSE(mc_pull_cal_low_ok(CENTER, true, cap_min));
            TEST_ASSERT_EQUAL_FLOAT(MC_PULL_CAL_FALLBACK_V_MIN, r.vmin);
            TEST_ASSERT_EQUAL_FLOAT(first_step_timed_out(2.00f).vmin, r.vmin);
            TEST_ASSERT_EQUAL_FLOAT(2.00f, r.vmax);
            TEST_ASSERT_EQUAL_INT8(pol, r.pol);  // a >= 100 mV move is still a measured direction
            TEST_ASSERT_TRUE(r.fallback);
        }
    }

    // pol = -1: the shallow end also comes from 3.30 - raw.
    const mc_pull_cal_range_t n = mc_pull_cal_range(CENTER, true, 3.30f - (CENTER + 0.120f), -1, true, 2.0f);
    TEST_ASSERT_EQUAL_FLOAT(MC_PULL_CAL_FALLBACK_V_MIN, n.vmin);
    TEST_ASSERT_EQUAL_INT8(-1, n.pol);
    TEST_ASSERT_TRUE(n.fallback);

    // The floor is the audit's 150 mV: 151 mV is kept.
    TEST_ASSERT_TRUE(mc_pull_cal_low_ok(CENTER, true, CENTER - 0.151f));
    TEST_ASSERT_FALSE(mc_pull_cal_low_ok(CENTER, false, CENTER - 0.300f));
}

static void test_shallow_second_step_is_kept(void)
{
    // The high side's safe fallback (1.75 V) is the least a step captures: a shallow push is kept,
    // since falling back could only narrow it.
    for (int hi = 100; hi <= 149; hi++)
    {
        const float cap_max = CENTER + 0.001f * (float)hi;
        const mc_pull_cal_range_t r = mc_pull_cal_range(CENTER, true, 1.20f, 1, true, cap_max);
        TEST_ASSERT_EQUAL_FLOAT(cap_max, r.vmax);
        TEST_ASSERT_TRUE(r.vmax >= MC_PULL_CAL_FALLBACK_V_MAX - 1e-6f);
        TEST_ASSERT_FALSE(r.fallback);
    }
}

static void test_timed_out_first_step_stores_the_nominal_low_side(void)
{
    const mc_pull_cal_range_t r = first_step_timed_out(2.10f);
    TEST_ASSERT_EQUAL_FLOAT(MC_PULL_CAL_FALLBACK_V_MIN, r.vmin);
    TEST_ASSERT_EQUAL_FLOAT(1.00f, r.vmin);  // MC_PULL_V_MIN default in Motion_control.cpp
    TEST_ASSERT_EQUAL_FLOAT(2.10f, r.vmax);  // the measured side is kept
    TEST_ASSERT_EQUAL_INT8(1, r.pol);
    TEST_ASSERT_TRUE(r.fallback);
}

static void test_timed_out_second_step_stores_the_minimum_real_high_side(void)
{
    const mc_pull_cal_range_t r = second_step_timed_out(1.20f, -1);
    TEST_ASSERT_EQUAL_FLOAT(1.20f, r.vmin);
    TEST_ASSERT_EQUAL_FLOAT(MC_PULL_CAL_FALLBACK_V_MAX, r.vmax);
    TEST_ASSERT_EQUAL_FLOAT(CENTER + MC_PULL_CAL_PRESS_DELTA_V, r.vmax);
    TEST_ASSERT_EQUAL_INT8(-1, r.pol);  // learned by the first step
    TEST_ASSERT_TRUE(r.fallback);
}

static void test_both_steps_timed_out_keep_the_unmeasured_polarity_at_plus_one(void)
{
    const mc_pull_cal_range_t r = both_steps_timed_out();
    TEST_ASSERT_EQUAL_FLOAT(MC_PULL_CAL_FALLBACK_V_MIN, r.vmin);
    TEST_ASSERT_EQUAL_FLOAT(MC_PULL_CAL_FALLBACK_V_MAX, r.vmax);
    TEST_ASSERT_EQUAL_INT8(1, r.pol);  // never measured: the pre-V10.3 stock orientation
    TEST_ASSERT_TRUE(r.fallback);
}

static void test_unmeasured_first_step_never_stores_an_inverted_polarity(void)
{
    TEST_ASSERT_EQUAL_INT8(1, mc_pull_cal_range(CENTER, false, CENTER, -1, true, 2.0f).pol);
    TEST_ASSERT_EQUAL_INT8(1, mc_pull_cal_range(CENTER, false, CENTER, -1, false, CENTER).pol);
}

static void test_rest_scatter_never_trips_the_jam_latch_on_any_stored_range(void)
{
    // Whatever the steps gave, the jam latch stays clear of 1.5x the rest scatter (30 mV): true for
    // a kept low side from 150 mV and for the fallback; a kept 100 mV side put it at 20 mV.
    const float v = CENTER - (1.5f * CAL_CENTER_EPS_V - 0.0001f);

    for (int i = 0; i < LO_CASES; i++)
    {
        for (int j = 0; j < N_HI; j++)
        {
            const mc_pull_cal_range_t r = outcome(lo_case(i), HI_CASES[j], 1);
            TEST_ASSERT_TRUE(pull_v_to_percent(r.vmin, r.vmax, v) >= JAM_LATCH_PCT);
        }
    }
}

static void test_jam_latch_needs_a_deliberate_pull_and_still_trips_on_a_real_one(void)
{
    const mc_pull_cal_range_t a = first_step_timed_out(2.0f);
    const mc_pull_cal_range_t b = outcome(120, 300, 1);  // shallow: the same fallback

    // Nothing up to CAL_PRESS_DELTA_V (100 mV), the move the firmware treats as deliberate.
    for (int mv = 0; mv <= 100; mv++)
    {
        TEST_ASSERT_TRUE(pull_v_to_percent(a.vmin, a.vmax, CENTER - 0.001f * (float)mv) >= JAM_LATCH_PCT);
        TEST_ASSERT_TRUE(pull_v_to_percent(b.vmin, b.vmax, CENTER - 0.001f * (float)mv) >= JAM_LATCH_PCT);
    }

    // Trips at 0.2 * 650 mV = 130 mV, so a real jam pulling the buffer to its end is still seen.
    TEST_ASSERT_TRUE(pull_v_to_percent(a.vmin, a.vmax, CENTER - 0.132f) < JAM_LATCH_PCT);
    TEST_ASSERT_TRUE(pull_v_to_percent(a.vmin, a.vmax, CENTER - 0.300f) < JAM_LATCH_PCT);
    TEST_ASSERT_TRUE(pull_v_to_percent(b.vmin, b.vmax, CENTER - 0.132f) < JAM_LATCH_PCT);

    // A kept 150 mV side trips at 0.2 * 150 = 30 mV.
    const mc_pull_cal_range_t c = outcome(151, 300, 1);
    TEST_ASSERT_TRUE(pull_v_to_percent(c.vmin, c.vmax, CENTER - 0.032f) < JAM_LATCH_PCT);
}

static void test_recalibration_gesture_keeps_its_absolute_100mV_rule(void)
{
    // With every slot empty, held 5 s this wipes the NVM: whatever the steps gave, never less than
    // 100 mV below centre (a kept 100 mV side let the 15 % rule fire at 69 mV).
    for (int i = 0; i < LO_CASES; i++)
    {
        for (int j = 0; j < N_HI; j++)
        {
            const mc_pull_cal_range_t r = outcome(lo_case(i), HI_CASES[j], 1);
            for (int mv = 0; mv < 100; mv++)
                TEST_ASSERT_FALSE(reset_gesture_pressed(r.vmin, r.vmax, CENTER - 0.001f * (float)mv));
        }
    }

    // ...and the user can still do it on a channel that fell back.
    const mc_pull_cal_range_t a = first_step_timed_out(2.0f);
    const mc_pull_cal_range_t b = both_steps_timed_out();
    const mc_pull_cal_range_t c = outcome(120, 300, 1);
    TEST_ASSERT_TRUE(reset_gesture_pressed(a.vmin, a.vmax, CENTER - 0.101f));
    TEST_ASSERT_TRUE(reset_gesture_pressed(b.vmin, b.vmax, CENTER - 0.101f));
    TEST_ASSERT_TRUE(reset_gesture_pressed(c.vmin, c.vmax, CENTER - 0.101f));
}

static void test_load_stops_stay_within_the_minimum_real_travel(void)
{
    const mc_pull_cal_range_t a = second_step_timed_out(1.20f, 1);
    const mc_pull_cal_range_t b = both_steps_timed_out();
    const float v = CENTER + MC_PULL_CAL_PRESS_DELTA_V;

    // Any buffer the calibration can measure moves at least CAL_PRESS_DELTA_V: every hard stop
    // (A1 95 %, P1S 97 %) trips within that travel.
    TEST_ASSERT_TRUE(pull_v_to_percent(a.vmin, a.vmax, v) >= P1S_HARD_STOP_PCT);
    TEST_ASSERT_TRUE(pull_v_to_percent(b.vmin, b.vmax, v) >= P1S_HARD_STOP_PCT);
    TEST_ASSERT_TRUE(pull_v_to_percent(a.vmin, a.vmax, CENTER + 0.090f) >= A1_HARD_STOP_PCT - 0.01f);
}

static void test_no_stored_side_is_narrower_than_its_floor(void)
{
    // Whatever the steps gave: the low side spans at least 150 mV, the high side at least
    // CAL_PRESS_DELTA_V, the smallest move a step accepts (the old code stored 50 mV for a failed
    // side and kept a 100 mV low side).
    for (int i = 0; i < LO_CASES; i++)
    {
        for (int j = 0; j < N_HI; j++)
        {
            const mc_pull_cal_range_t r = outcome(lo_case(i), HI_CASES[j], 1);
            TEST_ASSERT_TRUE(CENTER - r.vmin >= MC_PULL_CAL_MIN_LOW_SPAN_V - 1e-6f);
            TEST_ASSERT_TRUE(r.vmax - CENTER >= MC_PULL_CAL_PRESS_DELTA_V - 1e-6f);
        }
    }
}

static void test_fallback_flag_is_set_exactly_when_a_side_is_a_fallback(void)
{
    for (int i = 0; i < LO_CASES; i++)
    {
        for (int j = 0; j < N_HI; j++)
        {
            const int lo = lo_case(i);
            const int hi = HI_CASES[j];
            const mc_pull_cal_range_t r = outcome(lo, hi, 1);
            const float cap_min = CENTER - 0.001f * (float)lo;
            const bool low_kept = mc_pull_cal_low_ok(CENTER, lo > 0, cap_min);
            TEST_ASSERT_EQUAL(!(low_kept && hi > 0), r.fallback);
            TEST_ASSERT_EQUAL_HEX32(float_bits(low_kept ? cap_min : MC_PULL_CAL_FALLBACK_V_MIN), float_bits(r.vmin));
        }
    }
}

static void test_run_records_each_channel_and_its_fallback(void)
{
    mc_pull_cal_run_t run = {0u, 0u};

    // ch 0 calibrated, ch 1 shallow first step, ch 2 has no buffer, ch 3 second step timed out.
    const mc_pull_cal_range_t r0 = mc_pull_cal_run_channel(&run, 0, CENTER, true, 1.20f, -1, true, 2.0f);
    const mc_pull_cal_range_t r1 = mc_pull_cal_run_channel(&run, 1, CENTER, true, CENTER - 0.12f, 1, true, 2.0f);
    const mc_pull_cal_range_t r3 = mc_pull_cal_run_channel(&run, 3, CENTER, true, 1.30f, 1, false, CENTER);

    TEST_ASSERT_EQUAL_HEX8(0x0Bu, run.inserted_mask);
    TEST_ASSERT_EQUAL_HEX8(0x0Au, run.fallback_mask);

    // The returned range is the one mc_pull_cal_range() gives for the same steps.
    const mc_pull_cal_range_t e0 = mc_pull_cal_range(CENTER, true, 1.20f, -1, true, 2.0f);
    TEST_ASSERT_EQUAL_HEX32(float_bits(e0.vmin), float_bits(r0.vmin));
    TEST_ASSERT_EQUAL_HEX32(float_bits(e0.vmax), float_bits(r0.vmax));
    TEST_ASSERT_EQUAL_INT8(-1, r0.pol);
    TEST_ASSERT_FALSE(r0.fallback);
    TEST_ASSERT_EQUAL_FLOAT(MC_PULL_CAL_FALLBACK_V_MIN, r1.vmin);
    TEST_ASSERT_TRUE(r1.fallback);
    TEST_ASSERT_EQUAL_FLOAT(1.30f, r3.vmin);
    TEST_ASSERT_EQUAL_FLOAT(MC_PULL_CAL_FALLBACK_V_MAX, r3.vmax);
    TEST_ASSERT_TRUE(r3.fallback);
}

static void test_green_end_blink_only_when_every_buffer_and_write_succeeded(void)
{
    const mc_pull_cal_run_t clean = {0x0Fu, 0x00u};
    const mc_pull_cal_run_t empty = {0x00u, 0x00u};
    const mc_pull_cal_run_t failed = {0x0Fu, 0x02u};

    TEST_ASSERT_EQUAL_INT(MC_PULL_CAL_END_GREEN, mc_pull_cal_end(true, &clean));
    TEST_ASSERT_EQUAL_INT(MC_PULL_CAL_END_GREEN, mc_pull_cal_end(true, &empty));
    TEST_ASSERT_EQUAL_INT(MC_PULL_CAL_END_NONE, mc_pull_cal_end(true, &failed));
    TEST_ASSERT_EQUAL_INT(MC_PULL_CAL_END_FLASH_ERROR, mc_pull_cal_end(false, &clean));
    TEST_ASSERT_EQUAL_INT(MC_PULL_CAL_END_FLASH_ERROR, mc_pull_cal_end(false, &failed));

    // The fallback flash follows whenever a buffer fell back, also after the flash-error blink.
    TEST_ASSERT_EQUAL_UINT32(MC_PULL_CAL_END_FLASHES, mc_pull_cal_flashes(&failed, false));
    TEST_ASSERT_EQUAL_UINT32(0u, mc_pull_cal_flashes(&clean, false));
}

static void test_end_flash_marks_the_fallback_buffers(void)
{
    const mc_pull_cal_run_t run = {0x0Bu, 0x02u};  // buffers on 0, 1, 3; channel 1 fell back

    for (int k = 0; k < 2; k++)
    {
        const bool on = (k == 0);
        TEST_ASSERT_EQUAL_INT(MC_PULL_CAL_LED_GREEN, mc_pull_cal_flash_led(&run, 0, on, false));
        TEST_ASSERT_EQUAL_INT(on ? MC_PULL_CAL_LED_RED : MC_PULL_CAL_LED_OFF, mc_pull_cal_flash_led(&run, 1, on, false));
        TEST_ASSERT_EQUAL_INT(MC_PULL_CAL_LED_OFF, mc_pull_cal_flash_led(&run, 2, on, false));
        TEST_ASSERT_EQUAL_INT(MC_PULL_CAL_LED_GREEN, mc_pull_cal_flash_led(&run, 3, on, false));
    }

    // Every buffer fell back: no channel shows green at all.
    const mc_pull_cal_run_t all = {0x0Fu, 0x0Fu};
    for (uint8_t ch = 0; ch < 4; ch++)
    {
        TEST_ASSERT_NOT_EQUAL(MC_PULL_CAL_LED_GREEN, mc_pull_cal_flash_led(&all, ch, true, false));
        TEST_ASSERT_NOT_EQUAL(MC_PULL_CAL_LED_GREEN, mc_pull_cal_flash_led(&all, ch, false, false));
    }

    // 2.4 s, as before.
    TEST_ASSERT_EQUAL_UINT32(2400u, 2u * MC_PULL_CAL_END_FLASHES * MC_PULL_CAL_FLASH_MS);
}

static void test_boot_flash_is_brief_and_only_on_connected_fallback_buffers(void)
{
    const bool inserted[4] = {true, true, false, true};

    // Record: channels 1 and 2 fell back; channel 2 has no buffer now.
    const mc_pull_cal_run_t run = mc_pull_cal_run_loaded(inserted, 0x06u);
    TEST_ASSERT_EQUAL_HEX8(0x0Bu, run.inserted_mask);
    TEST_ASSERT_EQUAL_HEX8(0x06u, run.fallback_mask);

    TEST_ASSERT_EQUAL_UINT32(MC_PULL_CAL_BOOT_FLASHES, mc_pull_cal_flashes(&run, true));
    // Delays the bus start by less than a second, and visibly: several flashes.
    TEST_ASSERT_TRUE(2u * MC_PULL_CAL_BOOT_FLASHES * MC_PULL_CAL_FLASH_MS <= 1000u);
    TEST_ASSERT_TRUE(MC_PULL_CAL_BOOT_FLASHES >= 3u);

    for (int k = 0; k < 2; k++)
    {
        const bool on = (k == 0);
        TEST_ASSERT_EQUAL_INT(MC_PULL_CAL_LED_OFF, mc_pull_cal_flash_led(&run, 0, on, true));
        TEST_ASSERT_EQUAL_INT(on ? MC_PULL_CAL_LED_RED : MC_PULL_CAL_LED_OFF, mc_pull_cal_flash_led(&run, 1, on, true));
        TEST_ASSERT_EQUAL_INT(MC_PULL_CAL_LED_OFF, mc_pull_cal_flash_led(&run, 2, on, true));
        TEST_ASSERT_EQUAL_INT(MC_PULL_CAL_LED_OFF, mc_pull_cal_flash_led(&run, 3, on, true));
    }

    // Nothing to show: no flash, so the boot is not delayed at all.
    const mc_pull_cal_run_t clean = mc_pull_cal_run_loaded(inserted, 0x00u);
    const mc_pull_cal_run_t gone = mc_pull_cal_run_loaded(inserted, 0x04u);  // only the absent buffer
    TEST_ASSERT_EQUAL_UINT32(0u, mc_pull_cal_flashes(&clean, true));
    TEST_ASSERT_EQUAL_UINT32(0u, mc_pull_cal_flashes(&gone, true));

    // Only the four channel bits of the record count.
    TEST_ASSERT_EQUAL_HEX8(0x00u, mc_pull_cal_run_loaded(inserted, 0xF0u).fallback_mask);
}

// Flash_MC_PULL_cal_write_all's rsv in V10.3..V10.5 (V6..V10.2 wrote 0), and their reader.
static uint32_t v105_rsv_pack(const int8_t pol[4])
{
    uint32_t rsv = 0u;
    for (uint8_t ch = 0u; ch < 4u; ch++)
    {
        if (pol && pol[ch] < 0) rsv |= (1u << ch);
    }
    return rsv;
}

static int8_t v105_rsv_pol(uint32_t rsv, uint8_t ch)
{
    return (rsv & (1u << ch)) ? -1 : 1;
}

static void pol_from_bits(unsigned bits, int8_t pol[4])
{
    for (uint8_t ch = 0u; ch < 4u; ch++) pol[ch] = (bits & (1u << ch)) ? (int8_t)-1 : (int8_t)1;
}

static void test_record_from_an_older_release_reads_as_no_fallback(void)
{
    TEST_ASSERT_EQUAL_HEX8(0u, mc_pull_cal_rsv_fallback(0u));  // V6..V10.2

    for (unsigned bits = 0u; bits < 16u; bits++)
    {
        int8_t pol[4];
        pol_from_bits(bits, pol);
        const uint32_t rsv = v105_rsv_pack(pol);

        TEST_ASSERT_EQUAL_HEX8(0u, mc_pull_cal_rsv_fallback(rsv));
        for (uint8_t ch = 0u; ch < 4u; ch++) TEST_ASSERT_EQUAL_INT8(pol[ch], mc_pull_cal_rsv_pol(rsv, ch));

        // With no fallback the new writer stores exactly what V10.5 stored.
        TEST_ASSERT_EQUAL_HEX32(rsv, mc_pull_cal_rsv_pack(pol, 0u));
    }
}

static void test_fallback_bits_round_trip_and_older_firmware_reads_the_same_polarity(void)
{
    for (unsigned bits = 0u; bits < 16u; bits++)
    {
        for (unsigned fb = 0u; fb < 16u; fb++)
        {
            int8_t pol[4];
            pol_from_bits(bits, pol);
            const uint32_t rsv = mc_pull_cal_rsv_pack(pol, (uint8_t)fb);

            TEST_ASSERT_EQUAL_HEX32(0u, rsv & ~0xFFu);  // rsv bits 8..31 stay 0
            TEST_ASSERT_EQUAL_HEX8(fb, mc_pull_cal_rsv_fallback(rsv));
            for (uint8_t ch = 0u; ch < 4u; ch++)
            {
                TEST_ASSERT_EQUAL_INT8(pol[ch], mc_pull_cal_rsv_pol(rsv, ch));
                TEST_ASSERT_EQUAL_INT8(pol[ch], v105_rsv_pol(rsv, ch));
            }
        }
    }

    // No polarity array (as the old writer allowed), and stray upper bits of the mask dropped.
    TEST_ASSERT_EQUAL_HEX32(0x50u, mc_pull_cal_rsv_pack(NULL, 0x05u));
    TEST_ASSERT_EQUAL_HEX32(0x00u, mc_pull_cal_rsv_pack(NULL, 0xF0u));

    // Only bits 4..7 are fallback bits, whatever a record holds above them.
    TEST_ASSERT_EQUAL_HEX8(0x0Au, mc_pull_cal_rsv_fallback(0xFFFFFFA5u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_successful_steps_are_stored_exactly_as_before);
    RUN_TEST(test_shallow_first_step_falls_back_like_a_timed_out_one);
    RUN_TEST(test_shallow_second_step_is_kept);
    RUN_TEST(test_timed_out_first_step_stores_the_nominal_low_side);
    RUN_TEST(test_timed_out_second_step_stores_the_minimum_real_high_side);
    RUN_TEST(test_both_steps_timed_out_keep_the_unmeasured_polarity_at_plus_one);
    RUN_TEST(test_unmeasured_first_step_never_stores_an_inverted_polarity);
    RUN_TEST(test_rest_scatter_never_trips_the_jam_latch_on_any_stored_range);
    RUN_TEST(test_jam_latch_needs_a_deliberate_pull_and_still_trips_on_a_real_one);
    RUN_TEST(test_recalibration_gesture_keeps_its_absolute_100mV_rule);
    RUN_TEST(test_load_stops_stay_within_the_minimum_real_travel);
    RUN_TEST(test_no_stored_side_is_narrower_than_its_floor);
    RUN_TEST(test_fallback_flag_is_set_exactly_when_a_side_is_a_fallback);
    RUN_TEST(test_run_records_each_channel_and_its_fallback);
    RUN_TEST(test_green_end_blink_only_when_every_buffer_and_write_succeeded);
    RUN_TEST(test_end_flash_marks_the_fallback_buffers);
    RUN_TEST(test_boot_flash_is_brief_and_only_on_connected_fallback_buffers);
    RUN_TEST(test_record_from_an_older_release_reads_as_no_fallback);
    RUN_TEST(test_fallback_bits_round_trip_and_older_firmware_reads_the_same_polarity);
    return UNITY_END();
}
