// Host tests for src/jam_latch.h, driven through jam_latch_pass() the way Motion_control_run calls
// it once per main-loop pass. The on_use jam latch must trip only when the buffer stays below 40%
// for 500 ms (not on the first low reading, not at boot before the BMCU has pushed), also when the
// 20 s high-PWM latch has already braked the channel; the printer's resume must release it only
// once the buffer has come back to 40% since the trip, so a printer that retries by itself keeps
// getting 0xF06F; the buffer releases it at the on_use band's low edge while the printer is in
// on_use/stop_on_use and only at 85% in any other state; a tangle that is still there after a
// release must latch again. While latched, the BMCU must not push the filament of the printer's
// active channel in any printer state: its on_use control and hold_load in before_on_use are
// braked (jam_latch_brakes).
// The 20 s full-force push limit (jam_push_limit_pass) must count full-force push time at any
// buffer level, restart below full force, brake the channel at exactly 20 s and saturate there.

#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "jam_latch.h"

// A1 (standard and soft_load): MC_ON_USE_TARGET_PCT - MC_ON_USE_BAND_LO_DELTA.
#define BAND_LO_PCT (52.0f - 0.2f)
#define TARGET_PCT 52.0f
#define BAND_HI_PCT 60.0f  // MC_ON_USE_BAND_HI_PCT
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
}

void tearDown(void) {}

// What run() runs for the channel once motor_motion_switch has followed printer command m: the on_use
// control for on_use, hold_load for before_on_use (active channel, filament at the switch).
// Everything else it runs for the active channel cannot push a latched channel: stop_on_use brakes,
// idle and send_out are stopped, the pull-back states only retract. A channel that is not active
// runs the idle control, which is not braked and can push (below 30%, or the DM autoload in it);
// these cases do not model it.
static jam_ctrl_t bmcu_ctrl(_filament_motion m)
{
    if (!(active && filament)) return JAM_CTRL_OTHER;
    if (m == ON_USE) return JAM_CTRL_ON_USE;
    if (m == BEFORE_ON_USE) return JAM_CTRL_BEFORE_ON_USE;
    return JAM_CTRL_OTHER;
}

// run() on this pass: the on_use control or hold_load runs (and may push) unless it is braked.
static bool motor_pushes(jam_ctrl_t c)
{
    return ((c == JAM_CTRL_ON_USE) || (c == JAM_CTRL_BEFORE_ON_USE)) && !jam_latch_brakes(c, brake, jam);
}

// One main-loop pass for the channel: jam_latch_pass() in Motion_control_run, then
// motor_motion_switch, which puts the BMCU into its on_use control when the printer commands on_use
// for the active channel and filament is at the switch, then run(). So the BMCU follows the printer
// one pass later, as jam_latch_pass() sees it, and run() sees the latch as this pass left it.
static jam_event_t pass(_filament_motion m, float pct)
{
    jam_in_t in;
    in.on_use_ctrl = bmcu_on_use;
    in.filament = filament;
    in.active = active;
    in.motion = m;
    in.pct = pct;
    in.now_ms = now;

    const jam_event_t ev = jam_latch_pass(&st, &brake, &jam, &in, BAND_LO_PCT);
    if (ev == JAM_EVENT_TRIP) trips++;

    bmcu_on_use = active && filament && (m == ON_USE);
    pushing = motor_pushes(bmcu_ctrl(m));
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
    // A slack filament lets the buffer return to its rest position: that alone is no release.
    trip_now();
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, REST_PCT, 60000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(ON_USE, BAND_LO_PCT - 0.1f, 60000u));
    TEST_ASSERT_EQUAL_UINT8(1u, jam);
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
    // hold_load in before_on_use: the jam latch brakes it.
    TEST_ASSERT_FALSE(jam_latch_brakes(JAM_CTRL_BEFORE_ON_USE, 0u, 0u));
    TEST_ASSERT_TRUE(jam_latch_brakes(JAM_CTRL_BEFORE_ON_USE, 1u, 1u));
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
    RUN_TEST(test_buffer_bounce_restarts_the_release_timer);
    RUN_TEST(test_away_from_on_use_the_buffer_must_reach_85);
    RUN_TEST(test_release_level_follows_the_printer_state);
    RUN_TEST(test_release_timer_survives_ms_counter_wrap);
    RUN_TEST(test_brake_decision_per_control);
    RUN_TEST(test_resume_with_before_on_use_does_not_push_into_a_held_spool);
    RUN_TEST(test_before_on_use_pushes_again_from_the_pass_it_releases);
    RUN_TEST(test_push_limit_full_force_is_a_push_above_800);
    RUN_TEST(test_push_limit_brakes_at_20s);
    RUN_TEST(test_push_limit_restarts_below_full_force);
    RUN_TEST(test_push_limit_saturates_at_20s);
    RUN_TEST(test_push_limit_counts_at_any_buffer_level);
    RUN_TEST(test_printer_unload_and_reload_do_not_release_an_uncleared_tangle);
    RUN_TEST(test_fixed_tangle_resumes_normally_without_a_printer_command);
    RUN_TEST(test_fixed_tangle_resumes_normally_after_a_pause);
    return UNITY_END();
}
