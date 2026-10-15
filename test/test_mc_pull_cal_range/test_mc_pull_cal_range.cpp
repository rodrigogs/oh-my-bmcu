// Host tests for src/mc_pull_cal_range.h and the ranges the boot buffer calibration keeps
// (MC_PULL_calibration.cpp, V11's fail-closed run). A run stores a channel only when both steps
// counted and the range passes its checks; anything else fails the whole run (nothing saved, every
// buffer invalid). So the ranges judged here are the captured ends themselves, for every capture a
// run can produce: a first step whose end is too shallow must fail the run, so that no stored range
// lets rest-position scatter trip the jam latch or the recalibration gesture; a kept range must still
// trip the jam latch on a real pull and put the load stops inside the travel the buffer showed; and
// a record loaded at boot must be accepted exactly when a run would have stored it.
//
// MC_PULL_calibration.cpp itself is not built on the host (capture loops, LEDs, flash). Its accept
// rules are adapted copies below, locked to their regions by scripts/check_test_copies.py. The
// ranges are judged with the firmware's own percent mapping and recalibration rule
// (src/mc_pull_pct.h), jam trip level (src/jam_latch.h), and verbatim copies of the rounding to
// MC_PULL_pct, of the load stops and of the DM key threshold.

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unity.h>

#include "jam_latch.h"  // JAM_TRIP_PCT: on_use, below it for JAM_TRIP_MS -> jam latch
#include "mc_pull_cal_range.h"
#include "mc_pull_pct.h"

#define CENTER 1.65f  // normalised idle reading (MC_PULL_V_OFFSET)

// The Stage-1 load stops (percent of the calibrated range) of the three load modes; each image has
// one of them.
namespace soft_load
{
// ---- Motion_control.cpp at this commit: the soft_load Stage-1 constants, verbatim ----
    static constexpr int   MC_LOAD_S1_FAST_PCT       = 75;
    static constexpr int   MC_LOAD_S1_HARD_STOP_PCT  = 90;  // bezpiecznik
// ---- end of the Motion_control.cpp copy ----
}
namespace p1s
{
// ---- Motion_control.cpp at this commit: the P1S Stage-1 constants, verbatim ----
    static constexpr int   MC_LOAD_S1_FAST_PCT       = 88;
    static constexpr int   MC_LOAD_S1_HARD_STOP_PCT  = 97;  // bezpiecznik
// ---- end of the Motion_control.cpp copy ----
}
namespace a1
{
// ---- Motion_control.cpp at this commit: the A1 Stage-1 constants, verbatim ----
    static constexpr int   MC_LOAD_S1_FAST_PCT       = 85;
    static constexpr int   MC_LOAD_S1_HARD_STOP_PCT  = 95;  // bezpiecznik
// ---- end of the Motion_control.cpp copy ----
}

// MC_PULL_pct for a channel whose MC_PULL_pct_f is pct_f.
static uint8_t MC_PULL_pct[4];
static int pct_rounded(float pct_f)
{
    const uint8_t i = 0u;
// ---- Motion_control.cpp at this commit: MC_PULL_ONLINE_read's rounding to MC_PULL_pct, verbatim ----
        int pct = (int)(pct_f + 0.5f);
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        MC_PULL_pct[i] = (uint8_t)pct;
// ---- end of the Motion_control.cpp copy ----
    return (int)MC_PULL_pct[i];
}

// Motion_control_run's recalibration rule for a channel with range vmin..vmax reading v.
static bool reset_gesture_pressed(float vmin, float vmax, float v)
{
    const int pct = pct_rounded(mc_pull_v_to_pct(vmin, vmax, v));
    return MC_PULL_CAL_RESET_PRESSED(pct, v, vmin);
}

// ---- MC_PULL_calibration.cpp at this commit: finite_voltage, verbatim ----
static bool finite_voltage(float v)
{
    return isfinite(v) && v >= 0.05f && v <= 3.25f;
}
// ---- end of the MC_PULL_calibration.cpp copy ----

// ---- MC_PULL_calibration.cpp at this commit: dm_key_round_up_to_centi and dm_key_none_threshold_from_idle, verbatim ----
static inline uint8_t dm_key_round_up_to_centi(float v)
{
    if (v <= 0.0f) return 0u;

    float x = v * 100.0f - 0.0001f;
    int iv = (int)x;
    if ((float)iv < x) iv++;

    if (iv < 0) iv = 0;
    if (iv > 255) iv = 255;
    return (uint8_t)iv;
}

static inline float dm_key_none_threshold_from_idle(float key_value)
{
    const uint8_t key_cv = dm_key_round_up_to_centi(key_value);

    uint8_t thr_cv = (uint8_t)(key_cv + 10u);
    if (thr_cv < 60u) thr_cv = 60u;
    if (thr_cv > 139u) thr_cv = 139u;

    return 0.01f * (float)thr_cv;
}
// ---- end of the MC_PULL_calibration.cpp copy ----

// ---- adapted from MC_PULL_calibration.cpp: capture_minmax_one_ch_event's accept rule, with the two captures as inputs ----
// ---- anchor: capture_minmax_one_ch_event ----
// ok_min/vmin: the first step counted and its normalised end (capture_first_extreme_wait_release),
// ok_max/vmax: the same for the second step. true = the run keeps vmin..vmax for the channel; false
// fails the whole run. The yellow acks and the second capture are left out.
static bool run_keeps(float center_v, bool ok_min, float vmin, bool ok_max, float vmax)
{
    if (!mc_pull_cal_low_ok(center_v, ok_min, vmin)) return false;

    if (!ok_max) return false;

    if (!isfinite(vmin) || !isfinite(vmax) ||
        center_v - vmin < 0.050f || vmax - center_v < 0.050f ||
        vmax - vmin < 0.120f) return false;

    return true;
}

// ---- adapted from MC_PULL_calibration.cpp: MC_PULL_calibration_boot's check of each valid channel of a loaded record ----
// ---- anchor: MC_PULL_calibration_boot from /if \(stored\)/ to /valid &= / ----
// true = the channel stays valid (its bit is not cleared from valid).
static bool record_channel_ok(float offs, float vmin, float vmax, float dm_none)
{
    if (!isfinite(offs) || !finite_voltage(1.65f - offs) ||
        !isfinite(vmin) || !isfinite(vmax) ||
        !mc_pull_cal_low_ok(1.65f, true, vmin) || vmax - 1.65f < 0.050f ||
        vmax - vmin < 0.120f ||
        !isfinite(dm_none) || dm_none < 0.01f * 60.0f || dm_none > 0.01f * 139.0f)
        return false;
    return true;
}

// Upstream V11's rule without the low-side floor (if (!ok_min) return false;), to show what the
// floor prevents. Not firmware code here.
static bool v11_keeps(float center_v, bool ok_min, float vmin, bool ok_max, float vmax)
{
    if (!ok_min || !ok_max) return false;
    return !(center_v - vmin < 0.050f || vmax - center_v < 0.050f || vmax - vmin < 0.120f);
}

// What the captures can report, in mV from centre. A first step counts once the reading has left the
// centre by MC_PULL_CAL_PRESS_DELTA_V either way, and its end is the larger side (the polarity puts
// it below centre), so it spans 100 mV or more; 0 = it timed out. A second step counts only beyond
// MC_PULL_CAL_PRESS_DELTA_V above centre (a strict '>'), so 101 mV or more; 0 = it timed out.
#define LO_CASES 702
static int lo_case(int i) { return (i == 0) ? 0 : 99 + i; }  // 0, 100..800
static const int HI_CASES[] = {0, 101, 110, 120, 149, 150, 200, 350, 800};
#define N_HI (int)(sizeof(HI_CASES) / sizeof(HI_CASES[0]))

static float lo_v(int lo_mv) { return (lo_mv > 0) ? CENTER - 0.001f * (float)lo_mv : CENTER; }
static float hi_v(int hi_mv) { return (hi_mv > 0) ? CENTER + 0.001f * (float)hi_mv : CENTER; }

static bool kept(int lo_mv, int hi_mv)
{
    return run_keeps(CENTER, lo_mv > 0, lo_v(lo_mv), hi_mv > 0, hi_v(hi_mv));
}

static uint32_t ranges_kept;

void setUp(void) {}
void tearDown(void) {}

static void test_floor_constants_keep_the_jam_latch_clear_of_the_rest_window(void)
{
    // V11's capture constants.
    TEST_ASSERT_EQUAL_FLOAT(0.10f, MC_PULL_CAL_PRESS_DELTA_V);
    TEST_ASSERT_EQUAL_FLOAT(0.025f, MC_PULL_CAL_CENTER_EPS_V);

    // The jam latch sits (1 - JAM_TRIP_PCT / 50) of the low side below centre: 0.2 x 190 mV = 38 mV,
    // at least 1.5 x the 25 mV back-at-rest window (37.5 mV).
    const float latch_v = (1.0f - JAM_TRIP_PCT / 50.0f) * MC_PULL_CAL_MIN_LOW_SPAN_V;
    TEST_ASSERT_TRUE(latch_v >= 1.5f * MC_PULL_CAL_CENTER_EPS_V);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.038f, latch_v);

    // Wider than the smallest step, so the floor is what decides between 100 and 189 mV.
    TEST_ASSERT_TRUE(MC_PULL_CAL_MIN_LOW_SPAN_V > MC_PULL_CAL_PRESS_DELTA_V);
}

static void test_shallow_or_timed_out_first_step_fails_the_run(void)
{
    // 100..189 mV counted as a step but is too shallow to keep: the run fails, before the yellow ack.
    for (int lo = 100; lo <= 189; lo++)
    {
        TEST_ASSERT_FALSE(mc_pull_cal_low_ok(CENTER, true, lo_v(lo)));
        for (int j = 0; j < N_HI; j++) TEST_ASSERT_FALSE(kept(lo, HI_CASES[j]));
        // V11 alone kept it whenever the second step counted.
        TEST_ASSERT_TRUE(v11_keeps(CENTER, true, lo_v(lo), true, hi_v(200)));
    }

    // 190 mV and more is kept once the second step counted too.
    for (int lo = 190; lo <= 800; lo++)
    {
        TEST_ASSERT_TRUE(mc_pull_cal_low_ok(CENTER, true, lo_v(lo)));
        TEST_ASSERT_TRUE(kept(lo, 101));
        TEST_ASSERT_TRUE(kept(lo, 800));
        TEST_ASSERT_FALSE(kept(lo, 0));  // the second step timed out
    }

    // A timed-out first step fails whatever its reported end.
    TEST_ASSERT_FALSE(mc_pull_cal_low_ok(CENTER, false, CENTER - 0.300f));
    TEST_ASSERT_FALSE(run_keeps(CENTER, false, CENTER - 0.300f, true, CENTER + 0.300f));
    TEST_ASSERT_FALSE(kept(0, 300));

    // pol = -1: the normalised end of the first step is 3.30 - raw (cal_apply_polarity), the same
    // span below centre.
    for (int lo = 100; lo <= 800; lo++)
    {
        const float raw_max = CENTER + 0.001f * (float)lo;
        const float vmin = 3.30f - raw_max;
        if (lo <= 189) TEST_ASSERT_FALSE(run_keeps(CENTER, true, vmin, true, hi_v(200)));
        if (lo >= 191) TEST_ASSERT_TRUE(run_keeps(CENTER, true, vmin, true, hi_v(200)));
    }
}

static void test_rest_scatter_never_trips_the_jam_latch_on_any_kept_range(void)
{
    // 1.5 x the back-at-rest window below centre (37.4 mV) stays at or above JAM_TRIP_PCT on every
    // range a run keeps.
    const float v = CENTER - (1.5f * MC_PULL_CAL_CENTER_EPS_V - 0.0001f);

    ranges_kept = 0u;
    for (int i = 0; i < LO_CASES; i++)
    {
        for (int j = 0; j < N_HI; j++)
        {
            const int lo = lo_case(i);
            const int hi = HI_CASES[j];
            if (!kept(lo, hi)) continue;
            ranges_kept++;
            TEST_ASSERT_TRUE(mc_pull_v_to_pct(lo_v(lo), hi_v(hi), v) >= JAM_TRIP_PCT);
        }
    }
    TEST_ASSERT_EQUAL_UINT32(611u * (N_HI - 1), ranges_kept);  // 190..800 mV x every counted second step

    // V11 alone kept a 100 mV low side, which put the latch 20 mV below centre: tripped here.
    TEST_ASSERT_TRUE(v11_keeps(CENTER, true, lo_v(100), true, hi_v(200)));
    TEST_ASSERT_TRUE(mc_pull_v_to_pct(lo_v(100), hi_v(200), v) < JAM_TRIP_PCT);
}

static void test_jam_latch_still_trips_on_a_real_pull(void)
{
    // The narrowest kept low side (190 mV) trips at 0.2 x 190 = 38 mV.
    TEST_ASSERT_TRUE(kept(190, 200));
    TEST_ASSERT_TRUE(mc_pull_v_to_pct(lo_v(190), hi_v(200), CENTER - 0.037f) >= JAM_TRIP_PCT);
    TEST_ASSERT_TRUE(mc_pull_v_to_pct(lo_v(190), hi_v(200), CENTER - 0.039f) < JAM_TRIP_PCT);

    // On every kept range a buffer pulled to its calibrated end, or 21 % of its low side below
    // centre (the latch sits at 20 %), is below the trip level.
    for (int i = 0; i < LO_CASES; i++)
    {
        for (int j = 0; j < N_HI; j++)
        {
            const int lo = lo_case(i);
            const int hi = HI_CASES[j];
            if (!kept(lo, hi)) continue;
            TEST_ASSERT_TRUE(mc_pull_v_to_pct(lo_v(lo), hi_v(hi), lo_v(lo)) < JAM_TRIP_PCT);
            TEST_ASSERT_TRUE(mc_pull_v_to_pct(lo_v(lo), hi_v(hi), CENTER - 0.21f * 0.001f * (float)lo) < JAM_TRIP_PCT);
        }
    }
}

static void test_recalibration_gesture_keeps_its_absolute_100mV_rule(void)
{
    // Held 5 s with every slot empty it clears the calibration: on no kept range does it fire less
    // than 100 mV below centre, and at 101 mV it always does.
    for (int i = 0; i < LO_CASES; i++)
    {
        for (int j = 0; j < N_HI; j++)
        {
            const int lo = lo_case(i);
            const int hi = HI_CASES[j];
            if (!kept(lo, hi)) continue;
            for (int mv = 0; mv < 100; mv++)
                TEST_ASSERT_FALSE(reset_gesture_pressed(lo_v(lo), hi_v(hi), CENTER - 0.001f * (float)mv));
            TEST_ASSERT_TRUE(reset_gesture_pressed(lo_v(lo), hi_v(hi), CENTER - 0.101f));
        }
    }

    // On the narrowest kept side the 15 % rule (pct < 15.5 %: 0.69 x span) needs 131 mV.
    TEST_ASSERT_TRUE(pct_rounded(mc_pull_v_to_pct(lo_v(190), hi_v(200), CENTER - 0.130f)) > 15);
    TEST_ASSERT_TRUE(pct_rounded(mc_pull_v_to_pct(lo_v(190), hi_v(200), CENTER - 0.132f)) <= 15);

    // V11 alone kept a 100 mV side, where the 15 % rule fired 70 mV below centre.
    TEST_ASSERT_TRUE(reset_gesture_pressed(lo_v(100), hi_v(200), CENTER - 0.070f));
}

// ---- adapted from Motion_control.cpp: Motion_control_run's recalibration gesture, the channel it finds held (the per-channel state as arguments) ----
// ---- anchor: Motion_control_run from /int pressed = -1;/ to /if \(hard_blue\) \{ pressed/ ----
static int gesture_held_channel(const bool *inserted, const bool *valid, const int *pct, const float *v,
                                const float *vmin)
{
    int pressed = -1;
    for (uint8_t ch = 0; ch < 4u; ch++)
    {
        if (!inserted[ch] || !valid[ch]) continue;
        const bool hard_blue = MC_PULL_CAL_RESET_PRESSED(pct[ch], v[ch], vmin[ch]);
        if (hard_blue) { pressed = (int)ch; break; }
    }
    return pressed;
}

static void test_recalibration_gesture_skips_a_channel_whose_calibration_failed(void)
{
    // A channel whose boot calibration failed keeps V_OFFSET 0, V_MIN 1.00 and polarity 1, so its
    // reading is the raw voltage: a buffer idling at 1.55 V or less reads as held, and the gesture
    // fired 5 s after every boot (clear, reboot, a failed calibration again). It is skipped (a fork
    // deviation from V11); a valid channel held at the same time still fires it.
    const float raw_idle = 1.50f;
    const int raw_pct = pct_rounded(mc_pull_v_to_pct(1.00f, 2.30f, raw_idle));
    TEST_ASSERT_TRUE(MC_PULL_CAL_RESET_PRESSED(raw_pct, raw_idle, 1.00f));

    bool inserted[4] = {true, true, false, true};
    bool valid[4] = {false, true, true, true};
    int pct[4] = {raw_pct, 50, 0, 50};
    float v[4] = {raw_idle, CENTER, 0.0f, CENTER};
    const float vmin[4] = {1.00f, lo_v(400), lo_v(400), lo_v(400)};
    TEST_ASSERT_EQUAL_INT(-1, gesture_held_channel(inserted, valid, pct, v, vmin));

    pct[3] = 0;
    v[3] = CENTER - 0.30f;
    TEST_ASSERT_EQUAL_INT(3, gesture_held_channel(inserted, valid, pct, v, vmin));

    valid[0] = true; // V11: the uncalibrated channel fired it first
    TEST_ASSERT_EQUAL_INT(0, gesture_held_channel(inserted, valid, pct, v, vmin));
}

static void test_load_stops_trip_within_the_travel_the_buffer_showed(void)
{
    // The second step's end is where the buffer was pushed during calibration: every hard stop
    // (A1 95 %, P1S 97 %, soft_load 90 %) trips by then, and each mode's slower approach
    // (MC_LOAD_S1_FAST_PCT) starts before its stop and outside 1.5 x the back-at-rest window.
    const int stops[] = {a1::MC_LOAD_S1_HARD_STOP_PCT, p1s::MC_LOAD_S1_HARD_STOP_PCT,
                         soft_load::MC_LOAD_S1_HARD_STOP_PCT};
    const int fast[] = {a1::MC_LOAD_S1_FAST_PCT, p1s::MC_LOAD_S1_FAST_PCT, soft_load::MC_LOAD_S1_FAST_PCT};
    const float rest_hi = CENTER + 1.5f * MC_PULL_CAL_CENTER_EPS_V;

    for (int i = 0; i < LO_CASES; i++)
    {
        for (int j = 0; j < N_HI; j++)
        {
            const int lo = lo_case(i);
            const int hi = HI_CASES[j];
            if (!kept(lo, hi)) continue;
            TEST_ASSERT_TRUE(hi_v(hi) - CENTER > MC_PULL_CAL_PRESS_DELTA_V);
            for (unsigned k = 0; k < sizeof(stops) / sizeof(stops[0]); k++)
            {
                TEST_ASSERT_TRUE(mc_pull_v_to_pct(lo_v(lo), hi_v(hi), hi_v(hi)) >= (float)stops[k]);
                TEST_ASSERT_TRUE(mc_pull_v_to_pct(lo_v(lo), hi_v(hi), rest_hi) < (float)fast[k]);
                TEST_ASSERT_TRUE(fast[k] < stops[k]);
            }
        }
    }
}

static void test_loaded_record_is_kept_exactly_when_a_run_would_store_it(void)
{
    // Every range the captures can report, with a baseline the run accepts: the loaded record keeps
    // the channel exactly when the run kept it, so a record written without the floor (V11 alone)
    // makes the channel calibrate again.
    const float offs = 0.03f;
    const float dm_none = dm_key_none_threshold_from_idle(0.30f);
    for (int i = 1; i < LO_CASES; i++)
    {
        for (int j = 1; j < N_HI; j++)
        {
            const int lo = lo_case(i);
            const int hi = HI_CASES[j];
            TEST_ASSERT_EQUAL(kept(lo, hi), record_channel_ok(offs, lo_v(lo), hi_v(hi), dm_none));
        }
    }

    TEST_ASSERT_FALSE(record_channel_ok(offs, lo_v(150), hi_v(200), dm_none));
    TEST_ASSERT_TRUE(record_channel_ok(offs, lo_v(190), hi_v(200), dm_none));
    TEST_ASSERT_FALSE(record_channel_ok(NAN, lo_v(400), hi_v(200), dm_none));
    TEST_ASSERT_FALSE(record_channel_ok(offs, lo_v(400), INFINITY, dm_none));
}

static void test_every_baseline_a_run_accepts_passes_the_record_check(void)
{
    // The baseline keeps a channel only if every key reading is below 1.40 V and every Hall reading
    // inside finite_voltage(); the record check then sees dm_none = the threshold of the mean key
    // level and offs = 1.65 - the mean Hall level.
    const float vmin = lo_v(400);
    const float vmax = hi_v(300);
    char msg[96];
    float thr_lo = 2.0f, thr_hi = 0.0f;

    for (int mv = 0; mv < 1400; mv++)
    {
        const float dm_none = dm_key_none_threshold_from_idle(0.001f * (float)mv);
        if (dm_none < thr_lo) thr_lo = dm_none;
        if (dm_none > thr_hi) thr_hi = dm_none;
        TEST_ASSERT_TRUE(record_channel_ok(0.0f, vmin, vmax, dm_none));
    }
    snprintf(msg, sizeof(msg), "key idle 0..1.399 V: dm_none %.2f..%.2f V", (double)thr_lo, (double)thr_hi);
    TEST_MESSAGE(msg);

    // Hall idle means from 60 mV to 3.249 V. (At exactly 50 mV, 1.65 - (1.65 - 0.05) rounds one step
    // below 0.05 in float and the record is refused, and at exactly 3.25 V 1.65 - (1.65 - 3.25) rounds
    // one step above 3.25 under strict IEEE single arithmetic (the x86_64 CI runner; an arm64 build
    // that fuses the multiply-add into an FMA happened to land on 3.25); a Hall sensor idles near 1.65 V.)
    for (int mv = 60; mv <= 3249; mv++)
    {
        const float offs = 1.65f - 0.001f * (float)mv;
        TEST_ASSERT_TRUE(record_channel_ok(offs, vmin, vmax, dm_key_none_threshold_from_idle(0.30f)));
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_floor_constants_keep_the_jam_latch_clear_of_the_rest_window);
    RUN_TEST(test_shallow_or_timed_out_first_step_fails_the_run);
    RUN_TEST(test_rest_scatter_never_trips_the_jam_latch_on_any_kept_range);
    RUN_TEST(test_jam_latch_still_trips_on_a_real_pull);
    RUN_TEST(test_recalibration_gesture_keeps_its_absolute_100mV_rule);
    RUN_TEST(test_recalibration_gesture_skips_a_channel_whose_calibration_failed);
    RUN_TEST(test_load_stops_trip_within_the_travel_the_buffer_showed);
    RUN_TEST(test_loaded_record_is_kept_exactly_when_a_run_would_store_it);
    RUN_TEST(test_every_baseline_a_run_accepts_passes_the_record_check);
    return UNITY_END();
}
