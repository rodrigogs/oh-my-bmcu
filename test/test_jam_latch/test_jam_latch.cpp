// Host tests for src/jam_latch.h, driven through jam_latch_pass() the way Motion_control_run calls
// it once per main-loop pass. The on_use jam latch must trip only when the buffer stays below 40%
// for 500 ms (not on the first low reading, not at boot before the BMCU has pushed), also when the
// 20 s high-PWM latch has already braked the channel; the printer's resume must release it only
// once the buffer has come back to 40% since the trip, so a printer that retries by itself keeps
// getting 0xF06F; the buffer releases it at the on_use band's low edge while the printer is in
// on_use/stop_on_use (also a buffer that only rests a few mV high: that edge is inside the rest
// scatter) and only at 85% in any other state; a tangle that is still there after a release must
// latch again. While latched, the BMCU must not push the channel's filament in any
// printer state, also when another channel (or none) is active: its on_use control, hold_load in
// before_on_use and its idle control are braked (jam_latch_brakes).
// The 20 s full-force push limit (jam_push_limit_pass) must count full-force push time at any
// buffer level, restart below full force, brake the channel at exactly 20 s and saturate there,
// and go on across the anti-stall's rests, on which run() does not call it.
// Every pass also runs the firmware's auto-unload (src/auto_unload.h) as motor_motion_run does,
// held off by Motion_control_run while latched and on the release pass, until the buffer is back in
// its neutral band: lifting the buffer of a latched channel, to release it or not, and letting go
// afterwards, also after a sag and a new lift while it is still held, must never start it, and a
// new lift after the release must.

#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "auto_unload.h"
#include "jam_latch.h"

// The A1 on_use band (standard(A1); soft_load(A1) has the same one). The copy ends with the #endif
// that closes the A1 branch, so it matches the A1 constants only; #if 1 opens it here.
#if 1
// ---- Motion_control.cpp at this commit: the A1 on_use band, verbatim ----
    static constexpr float MC_ON_USE_TARGET_PCT    = 52.0f;
    static constexpr float MC_ON_USE_BAND_LO_DELTA = 0.2f;  // band_lo = target - delta
    static constexpr float MC_ON_USE_BAND_HI_PCT   = 60.0f;
#endif
// ---- end of the Motion_control.cpp copy ----
#define BAND_LO_PCT (MC_ON_USE_TARGET_PCT - MC_ON_USE_BAND_LO_DELTA) // what Motion_control_run passes
#define TARGET_PCT MC_ON_USE_TARGET_PCT
#define BAND_HI_PCT MC_ON_USE_BAND_HI_PCT
#define REST_PCT 50.0f     // the buffer spring's rest position (calibrated neutral)

static const _filament_motion IDLE = _filament_motion::idle;
static const _filament_motion SEND_OUT = _filament_motion::send_out;
static const _filament_motion ON_USE = _filament_motion::on_use;
static const _filament_motion BEFORE_PULL_BACK = _filament_motion::before_pull_back;
static const _filament_motion PULL_BACK = _filament_motion::pull_back;
static const _filament_motion BEFORE_ON_USE = _filament_motion::before_on_use;
static const _filament_motion STOP_ON_USE = _filament_motion::stop_on_use;

static uint32_t now;       // ms, as (uint32_t)time_ms_fast_from_ticks64()
static jam_latch_t st;     // g_on_use_jam[ch]
static uint8_t brake;      // g_on_use_low_latch[ch]
static uint8_t jam;        // g_on_use_jam_latch[ch]: brake + 0xF06F
static bool bmcu_on_use;   // MOTOR_CONTROL[ch].motion is on_use control, as the last pass left it
static bool filament;      // MC_ONLINE_key_stu[ch] != 0
static bool active;        // A.now_filament_num == ch
static int trips;
static bool pushing;       // run() let the channel's control push on the last pass (motor_pushes())
static int jammed_pushes;  // passes on which it did with the jam latch set
static auto_unload_t g_auto_unload[1]; // the channel's auto-unload (motor_motion_run)
static int unload_passes;  // passes on which the auto-unload drove the channel

void setUp(void)
{
    now = 1000u;
    memset(&st, 0, sizeof(st));
    brake = 0u;
    jam = 0u;
    bmcu_on_use = true;  // printing from this channel since the last pass
    filament = true;
    active = true;
    trips = 0;
    pushing = false;
    jammed_pushes = 0;
    memset(g_auto_unload, 0, sizeof(g_auto_unload));
    unload_passes = 0;
}

void tearDown(void) {}

// ---- adapted from Motion_control.cpp: the control motor_motion_switch picks for printer command m ----
// What run() runs for the channel once motor_motion_switch has followed printer command m. The
// active channel with filament at the switch: the on_use control for on_use, hold_load for
// before_on_use, the idle control for idle unless the jam latch is set; anything else it runs there
// cannot push a latched channel (stop_on_use brakes, a latched channel is stopped in idle and
// send_out, the pull-back states only retract). A channel that is not active, or with no filament
// at the switch: the idle control.
static jam_ctrl_t bmcu_ctrl(_filament_motion m)
{
    if (!(active && filament)) return JAM_CTRL_IDLE;
    if (m == ON_USE) return JAM_CTRL_ON_USE;
    if (m == BEFORE_ON_USE) return JAM_CTRL_BEFORE_ON_USE;
    if ((m == IDLE) && !jam) return JAM_CTRL_IDLE;
    return JAM_CTRL_OTHER;
}

// ---- adapted from Motion_control.cpp: whether run()'s control may push (idle: MC_PULL_stu -1, below 30%) ----
// run() on this pass: the on_use control or hold_load runs (and may push), and so does the idle
// control with the buffer below 30% (MC_PULL_stu -1: its PID pushes towards 50%), unless braked.
static bool motor_pushes(jam_ctrl_t c, float pct)
{
    const bool may_push = (c == JAM_CTRL_ON_USE) || (c == JAM_CTRL_BEFORE_ON_USE) ||
                          ((c == JAM_CTRL_IDLE) && ((int)(pct + 0.5f) < 30));
    return may_push && !jam_latch_brakes(c, brake, jam);
}

// Motion_control_run's jam loop, after jam_latch_pass() for channel ch.
static void jam_loop_hold(const uint8_t *g_on_use_jam_latch, jam_event_t ev)
{
    const uint8_t ch = 0u;
// ---- Motion_control.cpp at this commit: Motion_control_run's auto-unload hold, verbatim ----
        if (g_on_use_jam_latch[ch] || (ev == JAM_EVENT_RELEASE))
            auto_unload_hold(&g_auto_unload[ch]);
// ---- end of the Motion_control.cpp copy ----
}

// ---- adapted from Motion_control.cpp: motor_motion_run's auto-unload call ----
// For a channel whose AS5600 reads are good, with the link up: auto_unload_pass() with the motor
// state motor_motion_switch has just set (idle_ctrl: the idle control), and the key 'both' while
// filament is at the switch. A pass it drives does not run run().
static au_drive_t au_pass(bool idle_ctrl, float pct)
{
    au_in_t au;
    au.online    = true;
    au.inserted  = true;
    au.idle_ctrl = idle_ctrl;
    au.pct       = pct;
    au.ks        = filament ? 1u : 0u;
    au.now_ms    = now;
    return auto_unload_pass(&g_auto_unload[0], &au);
}

// ---- adapted from Motion_control.cpp: Motion_control_run's latch clear, jam_latch_pass() call and motor order ----
// One main-loop pass for the channel: Motion_control_run clears both latches when no filament is
// at the switch and the jam latch is set, and runs jam_latch_pass() and the auto-unload hold, then
// motor_motion_switch puts the BMCU into its on_use control when the printer commands on_use for
// the active channel and filament is at the switch, then the auto-unload's pass and, unless that
// drives the channel, run(). So the BMCU follows the printer one pass later, as jam_latch_pass()
// sees it, and the auto-unload and run() see the latch as this pass left it.
static jam_event_t pass(_filament_motion m, float pct)
{
    if (!filament && jam)
    {
        brake = 0u;
        jam = 0u;
    }

    jam_in_t in;
    in.on_use_ctrl = bmcu_on_use;
    in.filament = filament;
    in.active = active;
    in.motion = m;
    in.pct = pct;
    in.now_ms = now;

    const jam_event_t ev = jam_latch_pass(&st, &brake, &jam, &in, BAND_LO_PCT);
    if (ev == JAM_EVENT_TRIP) trips++;
    jam_loop_hold(&jam, ev);

    bmcu_on_use = active && filament && (m == ON_USE);
    const jam_ctrl_t ctrl = bmcu_ctrl(m);
    const au_drive_t drive = au_pass(ctrl == JAM_CTRL_IDLE, pct);
    if (drive == AU_DRIVE_UNLOAD) unload_passes++;
    pushing = (drive == AU_DRIVE_NONE) && motor_pushes(ctrl, pct);
    if (jam && pushing) jammed_pushes++;
    now++;
    return ev;
}

// 1 ms passes for `ms` at a constant printer command and buffer level. Returns the ms offset
// (0 = first pass of this call) of the pass on which the latch tripped or was released, or -1.
static int32_t hold(_filament_motion m, float pct, uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
        if (pass(m, pct) != JAM_EVENT_NONE) return (int32_t)i;
    return -1;
}

// The buffer moved from `from` to `to` in 1% steps, one pass each, at printer command m: a lift, or
// the spring taking it back when let go (through the auto-unload's 45-55% band on the way to 50%).
// Returns the step (0 = first) on which the latch tripped or was released, or -1.
static int32_t ramp(_filament_motion m, float from, float to)
{
    const float step = (to > from) ? 1.0f : -1.0f;
    int32_t i = 0;
    int32_t at = -1;
    for (float p = from; (step > 0.0f) ? (p < to) : (p > to); p += step, i++)
        if ((pass(m, p) != JAM_EVENT_NONE) && (at < 0)) at = i;
    if ((pass(m, to) != JAM_EVENT_NONE) && (at < 0)) at = i;
    return at;
}

static void trip_now(void)
{
    TEST_ASSERT_EQUAL_INT32(500, hold(ON_USE, 30.0f, 1000u));
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
    TEST_ASSERT_EQUAL_UINT8(1u, brake);
}

// ---- Trip ----

static void test_first_low_reading_does_not_latch(void)
{
    // The old code latched right here.
    TEST_ASSERT_EQUAL_INT(JAM_EVENT_NONE, pass(ON_USE, 39.9f));
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
    TEST_ASSERT_EQUAL_UINT8(0u, brake);

    // Nor on the first low reading after 10 s at the target.
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, TARGET_PCT, 10000u));
    TEST_ASSERT_EQUAL_INT(JAM_EVENT_NONE, pass(ON_USE, 10.0f));
}

static void test_latches_after_500ms_continuously_low(void)
{
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 35.0f, 500u));  // passes 0..499 ms
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
    TEST_ASSERT_EQUAL_INT(JAM_EVENT_TRIP, pass(ON_USE, 35.0f));  // 500 ms after the first low pass
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
    TEST_ASSERT_EQUAL_UINT8(1u, brake);
}

static void test_trip_level_is_below_40(void)
{
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 40.0f, 10000u));
    TEST_ASSERT_EQUAL_INT32(500, hold(ON_USE, 39.99f, 1000u));
}

static void test_short_dips_do_not_latch(void)
{
    // A snag the push clears within 0.5 s, twenty times in a row.
    for (int k = 0; k < 20; k++)
    {
        TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 30.0f, 450u));
        TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 45.0f, 1u));
    }
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
}

static void test_one_reading_back_above_40_restarts_the_timer(void)
{
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 35.0f, 400u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 41.0f, 1u));
    TEST_ASSERT_EQUAL_INT32(500, hold(ON_USE, 35.0f, 1000u));
}

static void test_leaving_on_use_control_restarts_the_timer(void)
{
    // A stop_on_use pass in the middle: the BMCU leaves its on_use control one pass later and
    // comes back one pass after the printer's on_use, so the new run starts on the second of those.
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 35.0f, 400u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 35.0f, 1u));
    TEST_ASSERT_EQUAL_INT32(501, hold(ON_USE, 35.0f, 1000u));
}

static void test_no_trip_outside_on_use_control_or_without_filament(void)
{
    jam_in_t in;
    in.active = true;
    in.motion = ON_USE;
    in.pct = 10.0f;

    // The BMCU is not in its on_use control (e.g. hold_load in before_on_use).
    in.on_use_ctrl = false;
    in.filament = true;
    for (uint32_t i = 0; i < 5000u; i++)
    {
        in.now_ms = now++;
        TEST_ASSERT_EQUAL_INT(JAM_EVENT_NONE, jam_latch_pass(&st, &brake, &jam, &in, BAND_LO_PCT));
    }

    // No filament at the switch (MC_ONLINE_key_stu == 0), e.g. on the pass it runs out.
    in.on_use_ctrl = true;
    in.filament = false;
    for (uint32_t i = 0; i < 5000u; i++)
    {
        in.now_ms = now++;
        TEST_ASSERT_EQUAL_INT(JAM_EVENT_NONE, jam_latch_pass(&st, &brake, &jam, &in, BAND_LO_PCT));
    }
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
    TEST_ASSERT_EQUAL_UINT8(0u, brake);
}

// The extruder drains the buffer from the on_use target with no feed: fast (e.g. a 0.4 mm nozzle
// at high flow) or slow (0.2 mm nozzle). The report comes 500 ms after the buffer reached 40%,
// whatever the rate.
static void drain_case(float pct_per_s)
{
    int32_t first_low = -1;
    int32_t trip_at = -1;
    for (int32_t i = 0; i < 60000; i++)
    {
        float pct = TARGET_PCT - pct_per_s * (float)i / 1000.0f;
        if (pct < 0.0f) pct = 0.0f;
        if (first_low < 0 && pct < 40.0f) first_low = i;
        if (pass(ON_USE, pct) == JAM_EVENT_TRIP)
        {
            trip_at = i;
            break;
        }
    }
    TEST_ASSERT_TRUE(first_low >= 0);
    TEST_ASSERT_EQUAL_INT32(first_low + 500, trip_at);
}

static void test_tangle_is_reported_500ms_after_the_buffer_reaches_40(void)
{
    drain_case(60.0f);
    setUp();
    drain_case(5.0f);
    setUp();
    drain_case(0.5f);
}

static void test_boot_with_low_buffer_and_free_spool_does_not_latch(void)
{
    // Restored on_use at boot with the buffer left at 20%: the BMCU enters its on_use control, pushes
    // and refills it in 100 ms, then holds the target. The old code latched on the first pass.
    bmcu_on_use = false;
    for (int32_t i = 0; i < 10000; i++)
    {
        float pct = 20.0f + (TARGET_PCT - 20.0f) * (float)i / 100.0f;
        if (pct > TARGET_PCT) pct = TARGET_PCT;
        TEST_ASSERT_EQUAL_INT(JAM_EVENT_NONE, pass(ON_USE, pct));
    }
}

static void test_boot_with_low_buffer_and_tangled_spool_latches(void)
{
    bmcu_on_use = false;
    TEST_ASSERT_EQUAL_INT32(501, hold(ON_USE, 20.0f, 1000u));
}

static void test_trip_timer_survives_ms_counter_wrap(void)
{
    now = 0xFFFFFF00u;
    TEST_ASSERT_EQUAL_INT32(500, hold(ON_USE, 30.0f, 1000u));
}

static void test_channel_braked_by_the_20s_latch_is_still_reported(void)
{
    // The 20 s high-PWM latch brakes the channel silently (brake set, jam clear). While the buffer
    // stays at or above 40% it stays silent, as before; once the extruder has drained the braked
    // buffer below 40% for 500 ms the printer gets 0xF06F.
    brake = 1u;
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 45.0f, 30000u));
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
    TEST_ASSERT_EQUAL_INT32(500, hold(ON_USE, 38.0f, 1000u));
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
    TEST_ASSERT_EQUAL_UINT8(1u, brake);
}

// ---- Printer command mapping ----

static void test_host_state_mapping(void)
{
    TEST_ASSERT_EQUAL_INT(JAM_HOST_ON_USE, jam_host_from_motion(true, ON_USE));
    TEST_ASSERT_EQUAL_INT(JAM_HOST_STOP_ON_USE, jam_host_from_motion(true, STOP_ON_USE));
    TEST_ASSERT_EQUAL_INT(JAM_HOST_BEFORE_ON_USE, jam_host_from_motion(true, BEFORE_ON_USE));
    TEST_ASSERT_EQUAL_INT(JAM_HOST_OTHER, jam_host_from_motion(true, IDLE));
    TEST_ASSERT_EQUAL_INT(JAM_HOST_OTHER, jam_host_from_motion(true, SEND_OUT));
    TEST_ASSERT_EQUAL_INT(JAM_HOST_OTHER, jam_host_from_motion(true, BEFORE_PULL_BACK));
    TEST_ASSERT_EQUAL_INT(JAM_HOST_OTHER, jam_host_from_motion(true, PULL_BACK));

    // Another channel (or none) is active: whatever the channel's own motion field says.
    const _filament_motion all[] = {IDLE, SEND_OUT, ON_USE, BEFORE_PULL_BACK,
                                    PULL_BACK, BEFORE_ON_USE, STOP_ON_USE};
    for (unsigned i = 0; i < sizeof(all) / sizeof(all[0]); i++)
        TEST_ASSERT_EQUAL_INT(JAM_HOST_OTHER, jam_host_from_motion(false, all[i]));
}

// ---- Release by the printer ----

static void test_latch_holds_while_printer_stays_on_use_with_low_buffer(void)
{
    // "The printer doesn't send anything on continue": nothing but on_use, buffer still low.
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 30.0f, 60000u));
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
}

static void test_resume_without_the_buffer_rising_keeps_the_latch(void)
{
    // Paused and resumed with nothing done: no filament moved, the tangle is still there. The
    // channel stays latched (braked, 0xF06F), with no new 500 ms of push.
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 30.0f, 5000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 30.0f, 10000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(BEFORE_ON_USE, 39.9f, 5000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 39.9f, 10000u));
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
    TEST_ASSERT_EQUAL_UINT8(1u, brake);
}

static void test_printer_retrying_by_itself_keeps_getting_the_jam(void)
{
    // A printer that answers 0xF06F with stop_on_use and on_use again, over and over: the latch
    // must never drop, not even for one on_use pass, so every retry sees 0xF06F and the motor
    // never pushes into the tangle.
    trip_now();
    for (int k = 0; k < 30; k++)
    {
        for (int i = 0; i < 300; i++)
        {
            TEST_ASSERT_EQUAL_INT(JAM_EVENT_NONE, pass(STOP_ON_USE, 30.0f));
            TEST_ASSERT_EQUAL_UINT8(1u, jam);
        }
        for (int i = 0; i < 1500; i++)
        {
            TEST_ASSERT_EQUAL_INT(JAM_EVENT_NONE, pass(ON_USE, 30.0f));
            TEST_ASSERT_EQUAL_UINT8(1u, jam);
            TEST_ASSERT_EQUAL_UINT8(1u, brake);
        }
    }
    TEST_ASSERT_EQUAL_INT(1, trips);
}

static void test_resume_after_the_buffer_came_back_releases(void)
{
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 30.0f, 2000u));
    // The user presses the lever and feeds some filament: the buffer reaches 40% for one reading,
    // then settles at 35%. Paused, that alone is no release.
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 40.0f, 1u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 35.0f, 3000u));
    // The resume releases it at once.
    TEST_ASSERT_EQUAL_INT32(0, hold(ON_USE, 35.0f, 10u));
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
    TEST_ASSERT_EQUAL_UINT8(0u, brake);
    // The tangle was still there: it latches again 500 ms after the BMCU's on_use control is back.
    TEST_ASSERT_EQUAL_INT32(500, hold(ON_USE, 35.0f, 1000u));
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
}

static void test_before_on_use_releases_once_the_buffer_has_come_back(void)
{
    // A latched channel is braked in before_on_use too, so only a person (feeding filament, lifting
    // the buffer) or the printer can bring the buffer back; the first such reading releases it.
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(BEFORE_ON_USE, 30.0f, 200u));
    TEST_ASSERT_EQUAL_INT32(0, hold(BEFORE_ON_USE, 40.0f, 10u));
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
    TEST_ASSERT_EQUAL_INT(0, jammed_pushes);
}

static void test_a_new_trip_starts_with_fresh_release_conditions(void)
{
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 45.0f, 10u));  // left on_use, buffer came back
    TEST_ASSERT_EQUAL_INT32(0, hold(ON_USE, 45.0f, 1u));
    TEST_ASSERT_EQUAL_INT32(500, hold(ON_USE, 30.0f, 1000u));   // latched again
    // Neither "the printer left on_use" nor "the buffer came back" carries over to this trip:
    // plain on_use does not release it, nor does a pause and resume with nothing done.
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 30.0f, 60000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 30.0f, 1000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 30.0f, 10000u));
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
}

// ---- Release by the buffer ----

static void test_buffer_back_at_band_for_1s_releases_in_on_use_and_stop_on_use(void)
{
    trip_now();
    TEST_ASSERT_EQUAL_INT32(1000, hold(ON_USE, BAND_LO_PCT, 2000u));
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
    TEST_ASSERT_EQUAL_UINT8(0u, brake);

    setUp();
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 30.0f, 3000u));
    TEST_ASSERT_EQUAL_INT32(1000, hold(STOP_ON_USE, BAND_LO_PCT, 2000u));
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
}

static void test_buffer_at_spring_rest_does_not_release(void)
{
    // A slack filament lets the buffer return to its rest position: a buffer that reads the
    // calibrated neutral there, or anything below the band's low edge, is no release. One that
    // rests a few mV high is (next test).
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, REST_PCT, 60000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, BAND_LO_PCT - 0.1f, 60000u));
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
}

static void test_a_buffer_resting_slightly_high_releases_and_the_resume_latches_again(void)
{
    // BAND_LO_PCT is 0.036 of the high side above the calibrated centre, +3.6 mV with the 1.75 V
    // fallback high side: inside the 20 mV rest scatter (jam_latch.h, mc_pull_cal_range.h). A
    // buffer that comes to rest reading that while paused releases the latch 1 s later with
    // nothing moved. The tangle is still there: the resume pushes again, and the channel latches
    // again 500 ms after the buffer is below 40%.
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 30.0f, 2000u));
    TEST_ASSERT_EQUAL_INT32(1000, hold(STOP_ON_USE, BAND_LO_PCT, 2000u));
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, BAND_LO_PCT, 500u));
    TEST_ASSERT_EQUAL_INT32(-1, ramp(ON_USE, 51.0f, 40.0f));
    TEST_ASSERT_TRUE(pushing);
    TEST_ASSERT_EQUAL_INT32((int32_t)JAM_TRIP_MS, hold(ON_USE, 30.0f, 1000u));
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
    TEST_ASSERT_EQUAL_INT(2, trips);
}

static void test_buffer_bounce_restarts_the_release_timer(void)
{
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 55.0f, 900u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 51.0f, 1u));
    TEST_ASSERT_EQUAL_INT32(1000, hold(ON_USE, 55.0f, 2000u));
}

static void test_away_from_on_use_the_buffer_must_reach_85(void)
{
    // send_out, pull-back and idle: the top of the on_use band, and anything up to the top of the
    // idle deadband and beyond, is not enough; 85% held for 1 s is.
    const _filament_motion away[] = {IDLE, SEND_OUT, BEFORE_PULL_BACK, PULL_BACK};
    for (unsigned i = 0; i < sizeof(away) / sizeof(away[0]); i++)
    {
        setUp();
        trip_now();
        TEST_ASSERT_EQUAL_INT32(-1, hold(away[i], BAND_LO_PCT, 5000u));
        TEST_ASSERT_EQUAL_INT32(-1, hold(away[i], BAND_HI_PCT, 5000u));
        TEST_ASSERT_EQUAL_INT32(-1, hold(away[i], 70.0f, 5000u));
        TEST_ASSERT_EQUAL_INT32(-1, hold(away[i], 84.9f, 5000u));
        TEST_ASSERT_EQUAL_UINT8(1u, jam);
        TEST_ASSERT_EQUAL_INT32(1000, hold(away[i], JAM_RELEASE_AWAY_PCT, 2000u));
        TEST_ASSERT_EQUAL_UINT8(0u, jam);
    }

    // Another channel active: same, even if this channel's motion field still says on_use.
    setUp();
    trip_now();
    active = false;
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, BAND_HI_PCT, 5000u));
    TEST_ASSERT_EQUAL_INT32(1000, hold(ON_USE, 90.0f, 2000u));
}

static void test_release_level_follows_the_printer_state(void)
{
    // 0.8 s at 60% while paused, then the printer goes to send_out: the band level no longer
    // applies there, and the time at it does not count towards the 85% level either.
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, BAND_HI_PCT, 800u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(SEND_OUT, BAND_HI_PCT, 5000u));
    TEST_ASSERT_EQUAL_INT32(1000, hold(SEND_OUT, 90.0f, 2000u));
}

static void test_release_timer_survives_ms_counter_wrap(void)
{
    now = 0xFFFFFC00u;
    trip_now();
    TEST_ASSERT_EQUAL_INT32(1000, hold(ON_USE, 55.0f, 2000u));
}

// ---- Motor ----

static void test_brake_decision_per_control(void)
{
    // The on_use control: either latch brakes it (the 20 s latch alone, or with the jam).
    TEST_ASSERT_FALSE(jam_latch_brakes(JAM_CTRL_ON_USE, 0u, 0u));
    TEST_ASSERT_TRUE(jam_latch_brakes(JAM_CTRL_ON_USE, 1u, 0u));
    TEST_ASSERT_TRUE(jam_latch_brakes(JAM_CTRL_ON_USE, 1u, 1u));
    // hold_load in before_on_use and the idle control: the jam latch brakes them (the 20 s latch
    // cannot be set there: set_motion() clears it on entry).
    TEST_ASSERT_FALSE(jam_latch_brakes(JAM_CTRL_BEFORE_ON_USE, 0u, 0u));
    TEST_ASSERT_TRUE(jam_latch_brakes(JAM_CTRL_BEFORE_ON_USE, 1u, 1u));
    TEST_ASSERT_FALSE(jam_latch_brakes(JAM_CTRL_IDLE, 0u, 0u));
    TEST_ASSERT_FALSE(jam_latch_brakes(JAM_CTRL_IDLE, 1u, 0u));
    TEST_ASSERT_TRUE(jam_latch_brakes(JAM_CTRL_IDLE, 1u, 1u));
    // Anything else is not run()'s business here (stopped, braked or retracting by itself).
    TEST_ASSERT_FALSE(jam_latch_brakes(JAM_CTRL_OTHER, 0u, 0u));
    TEST_ASSERT_FALSE(jam_latch_brakes(JAM_CTRL_OTHER, 1u, 0u));
    TEST_ASSERT_FALSE(jam_latch_brakes(JAM_CTRL_OTHER, 1u, 1u));
}

static void test_resume_with_before_on_use_does_not_push_into_a_held_spool(void)
{
    // Paused on the jam, the spool still held; the printer resumes with before_on_use (hold_load
    // would push towards 90% at up to 1000 PWM), then prints. The channel stays latched and braked
    // throughout, and the printer's on_use is answered with 0xF06F.
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 30.0f, 2000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(BEFORE_ON_USE, 30.0f, 10000u));
    TEST_ASSERT_FALSE(pushing);
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 30.0f, 10000u));
    TEST_ASSERT_FALSE(pushing);
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
    TEST_ASSERT_EQUAL_UINT8(1u, brake);
    TEST_ASSERT_EQUAL_INT(0, jammed_pushes);
    TEST_ASSERT_EQUAL_INT(1, trips);

    // The printer retrying by itself, before_on_use and on_use over and over: the same.
    for (int k = 0; k < 20; k++)
    {
        TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 30.0f, 300u));
        TEST_ASSERT_EQUAL_INT32(-1, hold(BEFORE_ON_USE, 30.0f, 700u));
        TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 30.0f, 1500u));
    }
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
    TEST_ASSERT_EQUAL_INT(0, jammed_pushes);
    TEST_ASSERT_EQUAL_INT(1, trips);
}

static void test_before_on_use_pushes_again_from_the_pass_it_releases(void)
{
    // Paused; the user clears the snag and feeds filament to 45%; the resume with before_on_use
    // releases the latch at once and hold_load pushes again from that pass, as before.
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 30.0f, 2000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 45.0f, 500u));
    TEST_ASSERT_EQUAL_INT(JAM_EVENT_RELEASE, pass(BEFORE_ON_USE, 45.0f));
    TEST_ASSERT_TRUE(pushing);
    TEST_ASSERT_EQUAL_INT32(-1, hold(BEFORE_ON_USE, 45.0f, 1000u));
    TEST_ASSERT_TRUE(pushing);
    TEST_ASSERT_EQUAL_INT(0, jammed_pushes);

    // A channel that was never latched runs hold_load in before_on_use as before.
    setUp();
    TEST_ASSERT_EQUAL_INT32(-1, hold(BEFORE_ON_USE, 30.0f, 1000u));
    TEST_ASSERT_TRUE(pushing);
}

// ---- Motion_control.cpp at this commit: filament_motion_enum, verbatim ----
enum class filament_motion_enum
{
    filament_motion_send,
    filament_motion_redetect,
    filament_motion_pull,
    filament_motion_stop,
    filament_motion_before_on_use,
    filament_motion_stop_on_use,
    filament_motion_pressure_ctrl_on_use,
    filament_motion_pressure_ctrl_idle,
    filament_motion_before_pull_back,
};
// ---- end of the Motion_control.cpp copy ----

// run()'s choice of the control it asks jam_latch_brakes() about, for MOTOR_CONTROL[ch].motion.
static jam_ctrl_t run_jam_ctrl(filament_motion_enum motion)
{
// ---- Motion_control.cpp at this commit: run()'s control for jam_latch_brakes(), verbatim ----
        const jam_ctrl_t jam_ctrl =
            (motion == filament_motion_enum::filament_motion_pressure_ctrl_on_use) ? JAM_CTRL_ON_USE :
            (motion == filament_motion_enum::filament_motion_before_on_use)        ? JAM_CTRL_BEFORE_ON_USE :
            (motion == filament_motion_enum::filament_motion_pressure_ctrl_idle)   ? JAM_CTRL_IDLE :
                                                                                     JAM_CTRL_OTHER;
// ---- end of the Motion_control.cpp copy ----
    return jam_ctrl;
}

static void test_run_brakes_a_jammed_channel_in_every_control_that_pushes(void)
{
    // With the jam latch set, run() brakes the on_use control, hold_load and the idle control. The
    // rest brake or stop by themselves (stop, stop_on_use), only retract (pull, before_pull_back), or
    // run only without filament at the switch (redetect) or unlatched (send: motor_motion_switch).
    struct { filament_motion_enum m; jam_ctrl_t c; bool braked; } rows[] = {
        {filament_motion_enum::filament_motion_pressure_ctrl_on_use, JAM_CTRL_ON_USE, true},
        {filament_motion_enum::filament_motion_before_on_use, JAM_CTRL_BEFORE_ON_USE, true},
        {filament_motion_enum::filament_motion_pressure_ctrl_idle, JAM_CTRL_IDLE, true},
        {filament_motion_enum::filament_motion_send, JAM_CTRL_OTHER, false},
        {filament_motion_enum::filament_motion_redetect, JAM_CTRL_OTHER, false},
        {filament_motion_enum::filament_motion_pull, JAM_CTRL_OTHER, false},
        {filament_motion_enum::filament_motion_stop, JAM_CTRL_OTHER, false},
        {filament_motion_enum::filament_motion_stop_on_use, JAM_CTRL_OTHER, false},
        {filament_motion_enum::filament_motion_before_pull_back, JAM_CTRL_OTHER, false},
    };
    for (unsigned i = 0; i < sizeof(rows) / sizeof(rows[0]); i++)
    {
        TEST_ASSERT_EQUAL_INT(rows[i].c, run_jam_ctrl(rows[i].m));
        TEST_ASSERT_EQUAL(rows[i].braked, jam_latch_brakes(run_jam_ctrl(rows[i].m), 1u, 1u));
        TEST_ASSERT_FALSE(jam_latch_brakes(run_jam_ctrl(rows[i].m), 0u, 0u));
    }
}

static void test_a_latched_channel_that_is_not_active_is_braked_in_its_idle_control(void)
{
    // A tangle trips on the active channel with the buffer drained to 20%; the printer then makes
    // another channel active, or none, and the filament stays at the switch: the channel runs the
    // idle control, whose PID would push towards 50% below 30% at up to 800 PWM, into the spool that
    // is still held. It is braked instead, for as long as it stays latched.
    TEST_ASSERT_EQUAL_INT32(500, hold(ON_USE, 20.0f, 1000u));
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 20.0f, 2000u));
    active = false;
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 20.0f, 60000u));
    TEST_ASSERT_FALSE(pushing);
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 29.0f, 60000u)); // its motion field is not read then
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
    TEST_ASSERT_EQUAL_INT(0, jammed_pushes);

    // The release rules do not depend on it: 85% held for 1 s releases it, and the idle control
    // then pushes below 30% again.
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 84.9f, 5000u));
    TEST_ASSERT_EQUAL_INT32(1000, hold(IDLE, JAM_RELEASE_AWAY_PCT, 2000u));
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 20.0f, 10u));
    TEST_ASSERT_TRUE(pushing);
    TEST_ASSERT_EQUAL_INT(0, jammed_pushes);

    // Filament pulled out past the switch: Motion_control_run clears the latch, nothing is braked.
    setUp();
    TEST_ASSERT_EQUAL_INT32(500, hold(ON_USE, 20.0f, 1000u));
    active = false;
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 20.0f, 1000u));
    TEST_ASSERT_FALSE(pushing);
    filament = false;
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 20.0f, 10u));
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
    TEST_ASSERT_EQUAL_UINT8(0u, brake);
    TEST_ASSERT_TRUE(pushing);

    // Made active again with the buffer back at 40% (fed by hand): the printer's resume releases it.
    setUp();
    TEST_ASSERT_EQUAL_INT32(500, hold(ON_USE, 20.0f, 1000u));
    active = false;
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 20.0f, 1000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 45.0f, 500u));
    active = true;
    TEST_ASSERT_EQUAL_INT32(0, hold(BEFORE_ON_USE, 45.0f, 10u));
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
    TEST_ASSERT_EQUAL_INT(0, jammed_pushes);
}

static void test_an_unlatched_or_active_idle_channel_is_not_affected(void)
{
    // Never latched: the idle control pushes below 30% as before, active or not.
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 20.0f, 100u));
    TEST_ASSERT_TRUE(pushing);
    active = false;
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 20.0f, 100u));
    TEST_ASSERT_TRUE(pushing);
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 50.0f, 100u));
    TEST_ASSERT_FALSE(pushing); // inside 30-70%: the idle control does not drive

    // Latched while active in idle: stopped by motor_motion_switch, as before.
    setUp();
    TEST_ASSERT_EQUAL_INT32(500, hold(ON_USE, 20.0f, 1000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 20.0f, 10000u));
    TEST_ASSERT_EQUAL_INT(0, jammed_pushes);
}

// ---- 20 s full-force push limit ----

#define DIR_RETRACT_POS 1.0f // MOTOR_CONTROL[ch].dir: a retract is +, so a push is -

// ms_per_pass passes at full force (900 PWM push) until the limit brakes, at most max_passes.
// Returns the number of passes it took, or -1.
static int32_t push_until_braked(uint32_t *hi, float time_s, int32_t max_passes)
{
    for (int32_t i = 1; i <= max_passes; i++)
        if (jam_push_limit_pass(hi, -900, DIR_RETRACT_POS, time_s)) return i;
    return -1;
}

static void test_push_limit_full_force_is_a_push_above_800(void)
{
    TEST_ASSERT_TRUE(jam_push_is_full(-801, 1.0f));
    TEST_ASSERT_TRUE(jam_push_is_full(-1000, 1.0f));
    TEST_ASSERT_FALSE(jam_push_is_full(-800, 1.0f));
    TEST_ASSERT_FALSE(jam_push_is_full(900, 1.0f)); // a retract
    TEST_ASSERT_FALSE(jam_push_is_full(0, 1.0f));
    TEST_ASSERT_TRUE(jam_push_is_full(801, -1.0f));
    TEST_ASSERT_FALSE(jam_push_is_full(-900, -1.0f));
    TEST_ASSERT_FALSE(jam_push_is_full(-900, 0.0f)); // direction not known yet
    TEST_ASSERT_FALSE(jam_push_is_full(900, 0.0f));
}

static void test_push_limit_brakes_at_20s(void)
{
    uint32_t hi = 0u;
    TEST_ASSERT_EQUAL_INT32(20000, push_until_braked(&hi, 0.001f, 30000));
    TEST_ASSERT_EQUAL_UINT32(JAM_PUSH_HI_MAX_US, hi);
    TEST_ASSERT_EQUAL_UINT32(20000000u, JAM_PUSH_HI_MAX_US);

    // At motor_motion_run's 200 ms step cap and at 4.8 ms passes. Each step is rounded to whole
    // microseconds: a pass of 18010 SysTick ticks at 18 MHz (time_E as motor_motion_run computes
    // it) is 1000.56 us, counted as 1001 us, so the limit comes after 19981 passes.
    hi = 0u;
    TEST_ASSERT_EQUAL_INT32(100, push_until_braked(&hi, 0.2f, 1000));
    hi = 0u;
    TEST_ASSERT_EQUAL_INT32(4167, push_until_braked(&hi, 0.0048f, 10000));
    hi = 0u;
    TEST_ASSERT_EQUAL_INT32(19981, push_until_braked(&hi, (float)18010u / (18.0f * 1000000.0f), 30000));
}

static void test_push_limit_restarts_below_full_force(void)
{
    uint32_t hi = 0u;
    TEST_ASSERT_EQUAL_INT32(-1, push_until_braked(&hi, 0.001f, 19999));
    TEST_ASSERT_EQUAL_UINT32(19999000u, hi);
    // One pass at 800 PWM, a retract, or no push: the count starts over.
    const int below[] = {-800, 850, 0};
    for (unsigned k = 0; k < sizeof(below) / sizeof(below[0]); k++)
    {
        TEST_ASSERT_FALSE(jam_push_limit_pass(&hi, below[k], DIR_RETRACT_POS, 0.001f));
        TEST_ASSERT_EQUAL_UINT32(0u, hi);
        TEST_ASSERT_EQUAL_INT32(-1, push_until_braked(&hi, 0.001f, 19999));
    }
    TEST_ASSERT_EQUAL_INT32(1, push_until_braked(&hi, 0.001f, 10));
}

static void test_push_limit_saturates_at_20s(void)
{
    uint32_t hi = 19900000u;
    TEST_ASSERT_TRUE(jam_push_limit_pass(&hi, -900, DIR_RETRACT_POS, 0.2f)); // 20.1 s
    TEST_ASSERT_EQUAL_UINT32(JAM_PUSH_HI_MAX_US, hi);
    for (int i = 0; i < 100000; i++)
        TEST_ASSERT_TRUE(jam_push_limit_pass(&hi, -900, DIR_RETRACT_POS, 0.2f));
    TEST_ASSERT_EQUAL_UINT32(JAM_PUSH_HI_MAX_US, hi);
    // No time step (the first pass after boot): nothing added.
    hi = 5000u;
    TEST_ASSERT_FALSE(jam_push_limit_pass(&hi, -900, DIR_RETRACT_POS, 0.0f));
    TEST_ASSERT_EQUAL_UINT32(5000u, hi);
}

static void test_push_limit_accumulates_across_the_anti_stall_rests(void)
{
    // A gear that does not turn: the on_use control pushes at full force (900 PWM, the anti-stall's
    // 850 PWM kick after 0.15 s), and once 0.8 s have added up the anti-stall rests for 0.5 s. run()
    // returns before jam_push_limit_pass() on the pass that starts the rest and on the rest passes,
    // so here they are no calls at all (1 + 499 of every 1300 ms). The count goes on across them:
    // 20 s of push, 25 bursts, braked on the last pass of the 25th, after about 32 s.
    uint32_t hi = 0u;
    int32_t braked_at = -1;
    for (int32_t t = 0; (t < 120000) && (braked_at < 0); t++)
    {
        const int32_t phase = t % 1300;
        if (phase >= 800) continue; // the rest: run() returns first
        const int pwm = (phase < 150) ? -900 : -850;
        if (jam_push_limit_pass(&hi, pwm, DIR_RETRACT_POS, 0.001f)) braked_at = t + 1;
    }
    TEST_ASSERT_EQUAL_INT32(24 * 1300 + 800, braked_at); // 32.000 s

    // Were the rest passes passes below full force (a call with 0 PWM), every rest would restart
    // the count and the stalled motor would go on pushing in bursts: nothing brakes in 10 minutes.
    hi = 0u;
    for (int32_t t = 0; t < 600000; t++)
    {
        const int32_t phase = t % 1300;
        const int pwm = (phase >= 800) ? 0 : ((phase < 150) ? -900 : -850);
        TEST_ASSERT_FALSE(jam_push_limit_pass(&hi, pwm, DIR_RETRACT_POS, 0.001f));
    }
    TEST_ASSERT_TRUE(hi < 800000u);
}

static void test_push_limit_counts_at_any_buffer_level(void)
{
    // A snag keeps the buffer around 40%: 450 ms below it, one reading above it, over and over, while
    // the on_use control pushes at full force (900 PWM below 50%). The timed jam trip never fires
    // (every dip is shorter than JAM_TRIP_MS); the 20 s limit brakes the channel silently at 20 s,
    // the time below 40% included. Once braked and still draining, the jam trip reports it.
    uint32_t hi = 0u;
    int32_t braked_at = -1;
    int32_t last_high = -1; // last pass with the buffer at or above 40%
    int32_t t = 0;
    for (; t < 30000 && braked_at < 0; t++)
    {
        const float pct = ((t % 451) == 450) ? 41.0f : 30.0f;
        if (pct >= JAM_TRIP_PCT) last_high = t;
        TEST_ASSERT_EQUAL_INT(JAM_EVENT_NONE, pass(ON_USE, pct));
// ---- adapted from Motion_control.cpp: run()'s on_use jam_push_limit_pass() call ----
        if (!brake && jam_push_limit_pass(&hi, -900, DIR_RETRACT_POS, 0.001f))
        {
            brake = 1u; // Motion_control.cpp: g_on_use_low_latch set, g_on_use_jam_latch clear
            braked_at = t + 1;
        }
    }
    TEST_ASSERT_EQUAL_INT32(20000, braked_at);
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
    // The trip timer started on the pass after the last reading at 41%.
    TEST_ASSERT_EQUAL_INT32(last_high + 1 + (int32_t)JAM_TRIP_MS - t, hold(ON_USE, 30.0f, 1000u));
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
}

// ---- Whole scenarios ----

static void test_printer_unload_and_reload_do_not_release_an_uncleared_tangle(void)
{
    // The printer unloads after the jam: the extruder pushes filament back into the buffer during
    // the pull-back, friction leaves the slider at 55-70%, then the printer's own reload starts.
    // The latch must hold throughout, so that send_out does not drive the motor into the tangle.
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 30.0f, 1000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(BEFORE_PULL_BACK, 45.0f, 500u));
    for (int i = 0; i < 3000; i++)
        TEST_ASSERT_EQUAL_INT(JAM_EVENT_NONE, pass(PULL_BACK, 45.0f + 25.0f * (float)i / 3000.0f));
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 70.0f, 30000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 55.0f, 30000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(SEND_OUT, 55.0f, 60000u));
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
}

static void test_fixed_tangle_resumes_normally_without_a_printer_command(void)
{
    trip_now();
    // Paused in on_use; the user clears the snag, presses the lever and lifts the buffer.
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 30.0f, 10000u));
    TEST_ASSERT_EQUAL_INT32(1000, hold(ON_USE, 58.0f, 2000u));
    // Printing again: the BMCU holds its target, with the odd short dip.
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, TARGET_PCT, 30000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, 38.0f, 300u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, TARGET_PCT, 30000u));
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
}

static void test_fixed_tangle_resumes_normally_after_a_pause(void)
{
    // Paused with stop_on_use; the user clears the snag and feeds filament until the buffer is
    // back at neutral; the resume releases it and printing goes on.
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 30.0f, 10000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 48.0f, 500u));
    TEST_ASSERT_EQUAL_INT32(0, hold(ON_USE, 48.0f, 10u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, TARGET_PCT, 60000u));
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
}

// ---- Lifting the buffer of a latched channel: release, not auto-unload ----

static void test_releasing_the_latch_by_lifting_the_buffer_does_not_unload_the_channel(void)
{
    // A tangle trips while printing; the printer then leaves the channel active in idle, where
    // motor_motion_switch stops a latched channel. A person lifts the buffer to 90% and holds it:
    // 1 s at 85% or above releases the latch, and from that pass on the channel runs its idle
    // control again with the buffer still held up (1.5 s more here). Then they let go and the spring
    // takes the buffer back through 55-45%. That used to start the 850 PWM auto-unload.
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 30.0f, 2000u));
    TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 30.0f, 90.0f));
    TEST_ASSERT_TRUE(hold(IDLE, 90.0f, 2000u) >= 0);
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 90.0f, 1500u));
    TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 90.0f, 50.0f));
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 50.0f, 5000u));
    TEST_ASSERT_EQUAL_INT(0, unload_passes);
    TEST_ASSERT_EQUAL_INT(0, jammed_pushes);

    // A new lift after that is the gesture again: it unloads the channel.
    TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 50.0f, 90.0f));
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 90.0f, 300u));
    TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 90.0f, 50.0f));
    TEST_ASSERT_TRUE(unload_passes > 0);
}

static void test_a_sag_after_the_release_by_lifting_does_not_unload_the_channel(void)
{
    // As above, but while the buffer is still held up after the release (the idle control retracts
    // against the hand above 70%), the hand sags to 75% for one pass, lifts it to 90% again and then
    // lets go. The sag used to end the hold-off (below 80%), so the new lift armed the auto-unload
    // and letting go started it. The hold-off now lasts until the buffer is back in the neutral band.
    // The same for a latched channel that is not the active one (its idle control braked until the
    // release).
    for (int k = 0; k < 2; k++)
    {
        setUp();
        trip_now();
        active = (k == 0);
        TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 30.0f, 2000u));
        TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 30.0f, 90.0f));
        TEST_ASSERT_TRUE(hold(IDLE, 90.0f, 2000u) >= 0);
        TEST_ASSERT_EQUAL_UINT8(0u, jam);
        TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 90.0f, 500u));
        TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 75.0f, 1u));
        TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 90.0f, 300u));
        TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 90.0f, 50.0f));
        TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 50.0f, 5000u));
        TEST_ASSERT_EQUAL_INT(0, unload_passes);
        TEST_ASSERT_EQUAL_INT(0, jammed_pushes);

        // Back at rest: the gesture unloads it.
        TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 50.0f, 90.0f));
        TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 90.0f, 300u));
        TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 90.0f, 50.0f));
        TEST_ASSERT_TRUE(unload_passes > 0);
    }
}

static void test_no_lift_of_a_latched_channel_that_is_not_active_unloads_it(void)
{
    // Latched, then another channel (or none) is active: the channel runs its idle control, braked,
    // where any lift of its buffer used to arm the auto-unload.
    TEST_ASSERT_EQUAL_INT32(500, hold(ON_USE, 20.0f, 1000u));
    active = false;
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 20.0f, 1000u));

    // The gesture itself (under 1 s at 90%): still latched, nothing unloads.
    TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 20.0f, 90.0f));
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 90.0f, 300u));
    TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 90.0f, 50.0f));
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 50.0f, 2000u));
    // Held at 82%, below the release level, for 3 s, then let go: the same.
    TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 50.0f, 82.0f));
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 82.0f, 3000u));
    TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 82.0f, 50.0f));
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 50.0f, 2000u));
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
    TEST_ASSERT_EQUAL_INT(0, unload_passes);

    // Held at 90% until released, then let go: released, nothing unloads.
    TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 50.0f, 90.0f));
    TEST_ASSERT_TRUE(hold(IDLE, 90.0f, 2000u) >= 0);
    TEST_ASSERT_EQUAL_UINT8(0u, jam);
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 90.0f, 500u));
    TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 90.0f, 50.0f));
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 50.0f, 3000u));
    TEST_ASSERT_EQUAL_INT(0, unload_passes);
    TEST_ASSERT_EQUAL_INT(0, jammed_pushes);

    // Then the gesture unloads it.
    TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 50.0f, 90.0f));
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 90.0f, 300u));
    TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 90.0f, 50.0f));
    TEST_ASSERT_TRUE(unload_passes > 0);
}

static void test_the_pass_that_releases_the_latch_is_held_off_too(void)
{
    // A slow lift while paused: the buffer is at the on_use band's low edge or above for 1 s,
    // which releases the latch, and reaches 80% only on that very pass. The printer then puts the
    // channel into idle with the buffer still held up, and it is let go.
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 30.0f, 1000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(STOP_ON_USE, 60.0f, JAM_RELEASE_MS));
    TEST_ASSERT_EQUAL_INT(JAM_EVENT_RELEASE, pass(STOP_ON_USE, 90.0f));
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 90.0f, 500u));
    TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 90.0f, 50.0f));
    TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 50.0f, 3000u));
    TEST_ASSERT_EQUAL_INT(0, unload_passes);
}

static void test_the_auto_unload_still_unloads_a_channel_that_was_never_latched(void)
{
    // In idle, active or not: the gesture unloads as before, and the hold-off never starts.
    for (int k = 0; k < 2; k++)
    {
        setUp();
        active = (k == 0);
        TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 50.0f, 1000u));
        TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 50.0f, 90.0f));
        TEST_ASSERT_EQUAL_INT32(-1, hold(IDLE, 90.0f, 300u));
        TEST_ASSERT_EQUAL_INT32(-1, ramp(IDLE, 90.0f, 50.0f));
        TEST_ASSERT_TRUE(unload_passes > 0);
        TEST_ASSERT_EQUAL_UINT8(0u, g_auto_unload[0].wait_low);
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_first_low_reading_does_not_latch);
    RUN_TEST(test_latches_after_500ms_continuously_low);
    RUN_TEST(test_trip_level_is_below_40);
    RUN_TEST(test_short_dips_do_not_latch);
    RUN_TEST(test_one_reading_back_above_40_restarts_the_timer);
    RUN_TEST(test_leaving_on_use_control_restarts_the_timer);
    RUN_TEST(test_no_trip_outside_on_use_control_or_without_filament);
    RUN_TEST(test_tangle_is_reported_500ms_after_the_buffer_reaches_40);
    RUN_TEST(test_boot_with_low_buffer_and_free_spool_does_not_latch);
    RUN_TEST(test_boot_with_low_buffer_and_tangled_spool_latches);
    RUN_TEST(test_trip_timer_survives_ms_counter_wrap);
    RUN_TEST(test_channel_braked_by_the_20s_latch_is_still_reported);
    RUN_TEST(test_host_state_mapping);
    RUN_TEST(test_latch_holds_while_printer_stays_on_use_with_low_buffer);
    RUN_TEST(test_resume_without_the_buffer_rising_keeps_the_latch);
    RUN_TEST(test_printer_retrying_by_itself_keeps_getting_the_jam);
    RUN_TEST(test_resume_after_the_buffer_came_back_releases);
    RUN_TEST(test_before_on_use_releases_once_the_buffer_has_come_back);
    RUN_TEST(test_a_new_trip_starts_with_fresh_release_conditions);
    RUN_TEST(test_buffer_back_at_band_for_1s_releases_in_on_use_and_stop_on_use);
    RUN_TEST(test_buffer_at_spring_rest_does_not_release);
    RUN_TEST(test_a_buffer_resting_slightly_high_releases_and_the_resume_latches_again);
    RUN_TEST(test_buffer_bounce_restarts_the_release_timer);
    RUN_TEST(test_away_from_on_use_the_buffer_must_reach_85);
    RUN_TEST(test_release_level_follows_the_printer_state);
    RUN_TEST(test_release_timer_survives_ms_counter_wrap);
    RUN_TEST(test_brake_decision_per_control);
    RUN_TEST(test_resume_with_before_on_use_does_not_push_into_a_held_spool);
    RUN_TEST(test_before_on_use_pushes_again_from_the_pass_it_releases);
    RUN_TEST(test_run_brakes_a_jammed_channel_in_every_control_that_pushes);
    RUN_TEST(test_a_latched_channel_that_is_not_active_is_braked_in_its_idle_control);
    RUN_TEST(test_an_unlatched_or_active_idle_channel_is_not_affected);
    RUN_TEST(test_push_limit_full_force_is_a_push_above_800);
    RUN_TEST(test_push_limit_brakes_at_20s);
    RUN_TEST(test_push_limit_restarts_below_full_force);
    RUN_TEST(test_push_limit_saturates_at_20s);
    RUN_TEST(test_push_limit_accumulates_across_the_anti_stall_rests);
    RUN_TEST(test_push_limit_counts_at_any_buffer_level);
    RUN_TEST(test_printer_unload_and_reload_do_not_release_an_uncleared_tangle);
    RUN_TEST(test_fixed_tangle_resumes_normally_without_a_printer_command);
    RUN_TEST(test_fixed_tangle_resumes_normally_after_a_pause);
    RUN_TEST(test_releasing_the_latch_by_lifting_the_buffer_does_not_unload_the_channel);
    RUN_TEST(test_a_sag_after_the_release_by_lifting_does_not_unload_the_channel);
    RUN_TEST(test_no_lift_of_a_latched_channel_that_is_not_active_unloads_it);
    RUN_TEST(test_the_pass_that_releases_the_latch_is_held_off_too);
    RUN_TEST(test_the_auto_unload_still_unloads_a_channel_that_was_never_latched);
    return UNITY_END();
}
