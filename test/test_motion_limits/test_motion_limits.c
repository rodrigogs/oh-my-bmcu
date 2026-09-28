// Host tests for src/motion_limits.h: the unload pull back, the redetect push after it and the DM
// Stage-2 autoload stages must end when their time or distance budget is used up, or when the gear
// does not move at high PWM, so the motor stops and motor_motion_switch serves the printer again.
// Normal 95 mm SOLO unloads, long retracts and a redetect ended by re-inserted filament must end
// exactly when they did before. A pull stopped by a limit must leave the unload fault (red LED).
// The send (load) stops on a stalled gear or after 10 m as before, with no time budget, and a
// normal A1 load is never stopped. The idle control's push brakes on a stalled gear at any PWM it
// pushes with, or after 10 s, until it stops pushing, red unless it stopped at the deadband's edge;
// a push that takes up the buffer's slack never.
//
// Each test drives the decision functions in 1 ms main-loop passes, the way Motion_control.cpp
// calls them. The simulated gear gives both of the firmware's position sources: the AS5600 count
// (as5600_count, what the guard reads, and through ml_travel_m every distance a motor state decides
// on) and the float32 odometer filament[].meters, updated per read the way AS5600_distance_updata
// does it (only reported to the printer). pwm is the PWM of the previous pass.

#include <stdint.h>
#include <unity.h>

#include "motion_limits.h"

// ---- adapted from platformio.ini: AMS_RETRACT_LEN of env:a1_solo_autoload_rgboff ----
#define SOLO_RETRACT_M 0.095f   // AMS_RETRACT_LEN of env:a1_solo_autoload_rgboff
// ---- adapted from build_all_firmwares.sh: its longest AMS_RETRACT_LEN ----
#define LONG_RETRACT_M 0.90f    // longest AMS_RETRACT_LEN built by build_all_firmwares.sh
// ---- adapted from Motion_control.cpp: DM_AUTO_S2_TARGET_M (a C++ constant) ----
// ---- anchor: DM_AUTO_S2_TARGET_M ----
#define DM_S2_LEN_M    0.120f   // DM_AUTO_S2_TARGET_M
#define PULL_CNT0      0xFFFFF000u  // pulls count up: they wrap the uint32_t count after 23.6 mm
#define FEED_CNT0      0x00000800u  // feeds count down: they wrap it after 11.8 mm

static motion_guard g;
static uint64_t now;

// ---- Simulated gear ----
static double   s_travel_mm;  // exact gear travel since the start of the test, + = feeding
static int32_t  s_read_cnt;   // whole counts read so far (relative to the start)
static uint32_t s_cnt;        // as5600_count
static float    s_meters;     // filament[].meters

static int32_t floor_i32(double x)
{
    const int32_t i = (int32_t)x;
    return (x < (double)i) ? (i - 1) : i;
}

static void gear_reset(uint32_t cnt0, float meters0)
{
    s_travel_mm = 0.0;
    s_read_cnt  = 0;
    s_cnt       = cnt0;
    s_meters    = meters0;
}

// ---- adapted from Motion_control.cpp: AS5600_distance_updata's count and odometer update ----
// ---- anchor: AS5600_distance_updata from /case AS5600_TRACK_MOVE:/ to /as5600_count\[i\] \+=/ ----
// One AS5600 read after the gear moved v_mm_s for 1 ms (v > 0 feeds, v < 0 pulls). The angle falls
// when meters rises (kAS5600_MM_PER_CNT < 0). Both sources get the same whole-count step.
static void gear_step(float v_mm_s)
{
    s_travel_mm += (double)v_mm_s * 0.001;
    const int32_t c    = floor_i32(-s_travel_mm / (double)ML_MM_PER_CNT);
    const int32_t diff = c - s_read_cnt;
    s_read_cnt = c;
    s_cnt += (uint32_t)diff;
    const float dist_mm = (float)diff * -ML_MM_PER_CNT;
    s_meters += dist_mm * 0.001f;
}

void setUp(void)
{
    now = 5000000ull;                // well after boot
    gear_reset(PULL_CNT0, 1.2345f);  // meters starts at 1.0 and moves with every feed and pull
}

void tearDown(void) {}

// ---- adapted from Motion_control.cpp: the pull back's speed command (PULL_V_FAST, PULL_V_END, PULL_RAMP_M) ----
// ---- anchor: PULL_V_FAST ----
// ---- anchor: PULL_V_END ----
// ---- anchor: PULL_RAMP_M ----
// ---- anchor: motor_motion_filamnet_pull_back_to_online_key from /remain = target - d/ to /= -v;/ ----
// The pull back's speed command in motor_motion_filamnet_pull_back_to_online_key: 60 mm/s, linear
// down to 12 mm/s over the last 15 mm (PULL_V_FAST, PULL_V_END, PULL_RAMP_M).
static float pull_cmd_mm_s(float remain_m)
{
    float k = remain_m / 0.015f;
    if (k < 0.0f) k = 0.0f;
    if (k > 1.0f) k = 1.0f;
    return 12.0f + (60.0f - 12.0f) * k;
}

// ---- adapted from Motion_control.cpp: the pull's speed PID against a held filament, simplified ----
// ---- anchor: MOTOR_PID ----
// ---- anchor: _MOTOR_CONTROL from /MOTOR_PID PID_speed/ to /MOTOR_PID PID_speed/ ----
// ---- anchor: run from /speed_set = g_pull_speed_set\[CHx\]/ to /PID_speed\.caculate\(/ ----
// ---- anchor: run from /const bool pull_mode =/ to /if \(x < \(float\)-PWM_lim\)/ ----
// ---- anchor: run from /const bool use_ramping =/ to /if \(x > hi\) x = hi;/ ----
// The speed PID from standstill against a held filament: the error is the full 60 mm/s (P 2, I 20),
// with the 500 PWM floor and the 2500 PWM/s soft-start ramp, clamped at 1000.
static float pull_pwm_blocked(uint32_t t_ms)
{
    float x = 120.0f + 1.2f * (float)t_ms;
    if (x < 500.0f) x = 500.0f;
    const float ramp = 2.5f * (float)t_ms;
    if (x > ramp) x = ramp;
    if (x > 1000.0f) x = 1000.0f;
    return x;
}

// ---- adapted from Motion_control.cpp: the pull back's end before motion_limits.h ----
// What the firmware did before: the pull ended only on the target or empty switches, the redetect
// only when a switch saw filament.
static bool old_pull_back_done(float target_m, float pulled_m, uint8_t ks)
{
    return (target_m <= 0.0f) || (pulled_m >= target_m) || (ks == 0u);
}

// Result of the last run_*() that ended.
static ml_result s_end;

// Pulls at speed_mm_s (0 = use the firmware's speed command) with a fixed PWM until the pull ends
// or max_ms passes. pulled_m is ml_travel_m from the count at the start, as the firmware measures
// it. Checks every pass against the old decision while the new limits are not due. Returns the pass
// (ms since the start) the pull ended in, or 0 if it did not.
static uint32_t run_pull(float target_m, float speed_mm_s, float pwm, uint32_t max_ms, bool expect_as_before)
{
    const uint32_t start_cnt = s_cnt;
    ml_pull_back_start(&g, now, s_cnt, target_m);

    for (uint32_t t = 1; t <= max_ms; t++)
    {
        now++;
        const float v = (speed_mm_s > 0.0f) ? speed_mm_s : pull_cmd_mm_s(target_m - ml_travel_m(s_cnt, start_cnt));
        gear_step(-v);

        const float d = ml_travel_m(s_cnt, start_cnt);
        const ml_result r = ml_pull_back_check(&g, now, s_cnt, pwm, target_m, d, 1u);
        if (expect_as_before) TEST_ASSERT_EQUAL(old_pull_back_done(target_m, d, 1u), r != ML_OK);
        if (r != ML_OK)
        {
            s_end = r;
            return t;
        }
    }
    s_end = ML_OK;
    return 0u;
}

static void test_time_budget_is_distance_at_12mm_s_plus_2s(void)
{
    TEST_ASSERT_EQUAL_UINT32(9917u, ml_time_budget_ms(SOLO_RETRACT_M));   // 7917 + 2000
    TEST_ASSERT_EQUAL_UINT32(10333u, ml_time_budget_ms(0.100f));          // jam-latch pull back
    TEST_ASSERT_EQUAL_UINT32(12000u, ml_time_budget_ms(DM_S2_LEN_M));
    TEST_ASSERT_EQUAL_UINT32(77000u, ml_time_budget_ms(LONG_RETRACT_M));
    TEST_ASSERT_EQUAL_UINT32(2000u, ml_time_budget_ms(0.0f));
    TEST_ASSERT_EQUAL_UINT32(2000u, ml_time_budget_ms(-1.0f));
}

static void test_distances_are_whole_counts_modulo_2_32(void)
{
    TEST_ASSERT_EQUAL_UINT32(16515u, ml_m_to_cnt(SOLO_RETRACT_M));   // 95.0 mm / 5.7524 um
    TEST_ASSERT_EQUAL_UINT32(156456u, ml_m_to_cnt(LONG_RETRACT_M));
    TEST_ASSERT_EQUAL_UINT32(0u, ml_m_to_cnt(0.0f));
    TEST_ASSERT_EQUAL_UINT32(0u, ml_m_to_cnt(-1.0f));

    // The stall step is the first whole count at or above 1 mm.
    TEST_ASSERT_TRUE((float)ML_STALL_MOVE_CNT * ML_MM_PER_CNT >= 1.0f);
    TEST_ASSERT_TRUE((float)(ML_STALL_MOVE_CNT - 1u) * ML_MM_PER_CNT < 1.0f);

    // Across the wrap, in both directions.
    TEST_ASSERT_EQUAL_UINT32(10u, ml_cnt_dist(5u, 0xFFFFFFFBu));
    TEST_ASSERT_EQUAL_UINT32(10u, ml_cnt_dist(0xFFFFFFFBu, 5u));
    TEST_ASSERT_EQUAL_UINT32(0u, ml_cnt_dist(123u, 123u));
    TEST_ASSERT_EQUAL_UINT32(0x80000000u, ml_cnt_dist(0x80000000u, 0u));
}

static void test_travel_m_is_the_count_distance_at_any_position(void)
{
    // One count is 5.7524 um. The SOLO retract (16515 counts) and the 10 m send cap convert back
    // to within half a count of their length.
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ml_cnt_to_m(0u));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-10f, 5.7524e-6f, ml_cnt_to_m(1u));
    TEST_ASSERT_FLOAT_WITHIN(3.0e-6f, SOLO_RETRACT_M, ml_cnt_to_m(ml_m_to_cnt(SOLO_RETRACT_M)));
    TEST_ASSERT_FLOAT_WITHIN(3.0e-6f, 10.0f, ml_cnt_to_m(ml_m_to_cnt(10.0f)));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 12353.0f, ml_cnt_to_m(0x80000000u));  // the largest distance

    // The same 16515 counts forwards and backwards, from any position, across the wrap too.
    const float solo_m = ml_cnt_to_m(16515u);
    const uint32_t starts[] = {0u, 5u, 0x7FFFFFF0u, 0x80000000u, PULL_CNT0, 0xFFFFFFFFu};
    for (uint32_t i = 0; i < sizeof(starts) / sizeof(starts[0]); i++)
    {
        TEST_ASSERT_EQUAL_FLOAT(solo_m, ml_travel_m(starts[i] + 16515u, starts[i]));
        TEST_ASSERT_EQUAL_FLOAT(solo_m, ml_travel_m(starts[i] - 16515u, starts[i]));
        TEST_ASSERT_EQUAL_FLOAT(0.0f, ml_travel_m(starts[i], starts[i]));
    }
    TEST_ASSERT_EQUAL_FLOAT(ml_cnt_to_m(1u), ml_travel_m(0u, 0xFFFFFFFFu));
    TEST_ASSERT_EQUAL_FLOAT(ml_cnt_to_m(1u), ml_travel_m(0xFFFFFFFFu, 0u));
}

static void test_solo_unload_ends_at_target_as_before(void)
{
    // 80 mm at 60 mm/s plus the 15 mm slow-down: about 1.84 s, far inside the 9.9 s budget.
    const uint32_t t = run_pull(SOLO_RETRACT_M, 0.0f, 650.0f, 20000u, true);
    TEST_ASSERT_UINT32_WITHIN(20u, 1836u, t);
    TEST_ASSERT_EQUAL(ML_END, s_end);
    // The first pass at or past 16515 counts (95.0 mm); the 12 mm/s end speed is 2.1 counts a pass.
    TEST_ASSERT_EQUAL_UINT32(16516u, ml_cnt_dist(s_cnt, PULL_CNT0));
}

static void test_solo_unload_at_full_pwm_ends_at_target_as_before(void)
{
    // Heavy drag: the PID sits at 1000 PWM, but the gear moves, so this is no stall.
    const uint32_t t = run_pull(SOLO_RETRACT_M, 0.0f, 1000.0f, 20000u, true);
    TEST_ASSERT_UINT32_WITHIN(20u, 1836u, t);
    TEST_ASSERT_EQUAL(ML_END, s_end);
}

static void test_long_retract_ends_at_target_as_before(void)
{
    // 885 mm at 60 mm/s plus the slow-down: about 15.25 s, the budget is 77 s.
    const uint32_t t = run_pull(LONG_RETRACT_M, 0.0f, 900.0f, 100000u, true);
    TEST_ASSERT_UINT32_WITHIN(20u, 15253u, t);
    TEST_ASSERT_EQUAL(ML_END, s_end);
}

static void test_slow_but_moving_pull_still_reaches_target(void)
{
    // 14 mm/s (just above the budget speed) at 1000 PWM: 95 mm take 6786 ms, inside 9.9 s.
    const uint32_t t = run_pull(SOLO_RETRACT_M, 14.0f, 1000.0f, 20000u, true);
    TEST_ASSERT_UINT32_WITHIN(1u, 6786u, t);
    TEST_ASSERT_EQUAL(ML_END, s_end);
}

static void test_pull_ends_when_switches_empty_as_before(void)
{
    const uint32_t start_cnt = s_cnt;
    ml_pull_back_start(&g, now, s_cnt, SOLO_RETRACT_M);
    for (uint32_t t = 1; t <= 300u; t++)
    {
        now++;
        gear_step(-60.0f);
        const uint8_t ks = (t < 300u) ? 1u : 0u;  // filament pulled out of the BMCU at 300 ms
        const float d = ml_travel_m(s_cnt, start_cnt);
        const ml_result r = ml_pull_back_check(&g, now, s_cnt, 1000.0f, SOLO_RETRACT_M, d, ks);
        TEST_ASSERT_EQUAL(old_pull_back_done(SOLO_RETRACT_M, d, ks), r != ML_OK);
        if (t == 300u) TEST_ASSERT_EQUAL(ML_END, r);
    }
}

static void test_zero_target_ends_at_once_as_before(void)
{
    ml_pull_back_start(&g, now, s_cnt, 0.0f);
    TEST_ASSERT_EQUAL(ML_END, ml_pull_back_check(&g, now + 1u, s_cnt, 0.0f, 0.0f, 0.0f, 1u));
}

static void test_blocked_pull_stops_1s_after_reaching_800_pwm(void)
{
    // The filament is held: the gear does not turn, the PID winds up to 1000 PWM.
    const uint32_t start = s_cnt;
    ml_pull_back_start(&g, now, start, SOLO_RETRACT_M);

    uint32_t t800 = 0u, t_done = 0u;
    float pwm_prev = 0.0f;  // PWM applied since the previous pass (x_prev)
    for (uint32_t t = 1; t <= 60000u && !t_done; t++)
    {
        now++;
        const uint32_t p = ((t & 1u) != 0u) ? (start + 1u) : (start - 1u);  // +-1 count of jitter
        const ml_result r = ml_pull_back_check(&g, now, p, pwm_prev, SOLO_RETRACT_M, 0.0f, 1u);
        if (r != ML_OK)
        {
            TEST_ASSERT_EQUAL(ML_STALL, r);
            t_done = t;
        }
        pwm_prev = pull_pwm_blocked(t);
        if (!t800 && pwm_prev >= ML_STALL_PWM) t800 = t;
    }
    TEST_ASSERT_EQUAL_UINT32(567u, t800);  // (800 - 120) / 1.2 ms
    TEST_ASSERT_EQUAL_UINT32(t800 + ML_STALL_MS, t_done);
}

// One guard check of the state under test, as Motion_control.cpp makes it every pass.
typedef ml_result (*guard_check_fn)(uint32_t pos_cnt, float pwm);

static ml_result check_pull(uint32_t pos_cnt, float pwm)
{
    return ml_pull_back_check(&g, now, pos_cnt, pwm, SOLO_RETRACT_M, 0.0f, 1u);  // target not reached
}

static ml_result check_redetect(uint32_t pos_cnt, float pwm)
{
    return ml_redetect_check(&g, now, pos_cnt, pwm, 0u);
}

static ml_result check_dm_s2(uint32_t pos_cnt, float pwm)
{
    return motion_guard_check(&g, now, pos_cnt, pwm);
}

// Moves at speed_mm_s (v > 0 feeds, v < 0 pulls) for move_ms passes, then the gear is blocked and
// holds its position, all at pwm. Returns the pass the guard ended the state in, 0 if it did not.
static uint32_t run_move_then_block(guard_check_fn check, float speed_mm_s, uint32_t move_ms, float pwm)
{
    for (uint32_t t = 1; t <= 60000u; t++)
    {
        now++;
        if (t <= move_ms) gear_step(speed_mm_s);
        const ml_result r = check(s_cnt, pwm);
        if (r != ML_OK)
        {
            s_end = r;
            return t;
        }
    }
    s_end = ML_OK;
    return 0u;
}

static void test_pull_that_moves_then_blocks_stops_1s_after_the_block(void)
{
    // 40.8 mm at 60 mm/s (680 passes), then the filament is held at 1000 PWM. At 60 mm/s the stall
    // window restarts every 17 passes (1.02 mm, the first pass with at least 174 counts of travel),
    // so it last restarts in the block's pass, 680 = 40 x 17.
    ml_pull_back_start(&g, now, s_cnt, SOLO_RETRACT_M);
    TEST_ASSERT_EQUAL_UINT32(680u + ML_STALL_MS, run_move_then_block(check_pull, -60.0f, 680u, 1000.0f));
    TEST_ASSERT_EQUAL(ML_STALL, s_end);
    TEST_ASSERT_EQUAL_UINT32(7092u, ml_cnt_dist(s_cnt, PULL_CNT0));  // 40.8 mm, across the wrap

    // Held after 40.0 mm (667 passes): the window last restarted in pass 663 (39 x 17), so the stall
    // comes 4 ms before block + ML_STALL_MS. That is the 1 mm resolution of the stall check.
    setUp();
    ml_pull_back_start(&g, now, s_cnt, SOLO_RETRACT_M);
    TEST_ASSERT_EQUAL_UINT32(663u + ML_STALL_MS, run_move_then_block(check_pull, -60.0f, 667u, 1000.0f));
    TEST_ASSERT_EQUAL(ML_STALL, s_end);
}

static void test_stall_needs_high_pwm(void)
{
    // Blocked, but only 700 PWM: not a stall by definition. The time budget still ends it.
    const uint32_t t = run_pull(SOLO_RETRACT_M, 0.0001f, 700.0f, 60000u, false);
    TEST_ASSERT_EQUAL_UINT32(9917u, t);
    TEST_ASSERT_EQUAL(ML_TIME, s_end);
}

static void test_creeping_pull_stops_at_time_budget(void)
{
    // 5 mm/s at 1000 PWM: moves 1 mm every 200 ms (no stall) but would need 19 s for 95 mm.
    const uint32_t start_cnt = s_cnt;
    const uint32_t t = run_pull(SOLO_RETRACT_M, 5.0f, 1000.0f, 60000u, false);
    TEST_ASSERT_EQUAL_UINT32(9917u, t);
    TEST_ASSERT_EQUAL(ML_TIME, s_end);
    TEST_ASSERT_FLOAT_WITHIN(0.0005f, 0.0496f, ml_travel_m(s_cnt, start_cnt));
}

static void test_long_retract_time_budget(void)
{
    const uint32_t t = run_pull(LONG_RETRACT_M, 5.0f, 1000.0f, 200000u, false);
    TEST_ASSERT_EQUAL_UINT32(77000u, t);
    TEST_ASSERT_EQUAL(ML_TIME, s_end);
}

static void test_stall_threshold_is_1mm_per_s(void)
{
    // 0.8 mm/s at high PWM is a stall (0.8 mm in the 1 s window).
    uint32_t t = run_pull(SOLO_RETRACT_M, 0.8f, 1000.0f, 60000u, false);
    TEST_ASSERT_EQUAL_UINT32(ML_STALL_MS, t);
    TEST_ASSERT_EQUAL(ML_STALL, s_end);

    // 1.5 mm/s moves 1 mm in 667 ms: no stall, the time budget ends it.
    setUp();
    t = run_pull(SOLO_RETRACT_M, 1.5f, 1000.0f, 60000u, false);
    TEST_ASSERT_EQUAL_UINT32(9917u, t);
    TEST_ASSERT_EQUAL(ML_TIME, s_end);
}

static void test_pull_at_5000m_odometer_is_not_a_false_stall(void)
{
    // meters is 5000 m after a long uptime. Its float32 step there is 0.49 mm, so the 0.06 mm of
    // each 1 ms read of a 60 mm/s pull is lost and meters stands still. The pull's target check and
    // the guard count AS5600 steps: a heavy-drag pull at 1000 PWM is no stall and ends at 95 mm, as
    // at a small odometer. (Measured with meters, it never reached its target, and the time budget
    // ended it after 595 mm of gear travel.)
    gear_reset(PULL_CNT0, 5000.0f);
    const uint32_t t = run_pull(SOLO_RETRACT_M, 0.0f, 1000.0f, 60000u, true);
    TEST_ASSERT_EQUAL_FLOAT(5000.0f, s_meters);
    TEST_ASSERT_UINT32_WITHIN(20u, 1836u, t);
    TEST_ASSERT_EQUAL(ML_END, s_end);
    TEST_ASSERT_EQUAL_UINT32(16516u, ml_cnt_dist(s_cnt, PULL_CNT0));  // 95 mm, across the wrap

    // The same pull that is then held stops ML_STALL_MS after the block, as at a small odometer.
    gear_reset(PULL_CNT0, 5000.0f);
    ml_pull_back_start(&g, now, s_cnt, SOLO_RETRACT_M);
    TEST_ASSERT_EQUAL_UINT32(680u + ML_STALL_MS, run_move_then_block(check_pull, -60.0f, 680u, 1000.0f));
    TEST_ASSERT_EQUAL(ML_STALL, s_end);
    TEST_ASSERT_EQUAL_FLOAT(5000.0f, s_meters);
}

static void test_pull_end_does_not_depend_on_the_odometer(void)
{
    // The same SOLO pull (the firmware's speed command, from near the count wrap) ends in the same
    // pass at the same count at any odometer. Measured with meters it ended 0.5 mm long at 20 m,
    // 1.0-2.1 mm short at 50-300 m, and not at its target at 600 m or more.
    const uint32_t t_ref = run_pull(SOLO_RETRACT_M, 0.0f, 650.0f, 20000u, true);
    const uint32_t cnt_ref = ml_cnt_dist(s_cnt, PULL_CNT0);
    TEST_ASSERT_EQUAL(ML_END, s_end);

    const float odometers_m[] = {20.0f, 50.0f, 130.0f, 300.0f, 600.0f, 5000.0f};
    for (uint32_t i = 0; i < sizeof(odometers_m) / sizeof(odometers_m[0]); i++)
    {
        gear_reset(PULL_CNT0, odometers_m[i]);
        TEST_ASSERT_EQUAL_UINT32(t_ref, run_pull(SOLO_RETRACT_M, 0.0f, 650.0f, 20000u, true));
        TEST_ASSERT_EQUAL(ML_END, s_end);
        TEST_ASSERT_EQUAL_UINT32(cnt_ref, ml_cnt_dist(s_cnt, PULL_CNT0));
    }
}

static void test_bus_offline_gap_is_not_counted(void)
{
    // Blocked at 1000 PWM for 900 ms, then the bus is offline for 5 s (motors stopped, no checks).
    ml_pull_back_start(&g, now, s_cnt, SOLO_RETRACT_M);
    for (uint32_t t = 1; t <= 900u; t++)
    {
        now++;
        TEST_ASSERT_EQUAL(ML_OK, ml_pull_back_check(&g, now, s_cnt, 1000.0f, SOLO_RETRACT_M, 0.0f, 1u));
    }
    now += 5000u;
    // The first pass after the gap starts a new stall window: 1 s more, not an instant stop.
    uint32_t t_done = 0u;
    for (uint32_t t = 1; t <= 5000u && !t_done; t++)
    {
        if (ml_pull_back_check(&g, now, s_cnt, 1000.0f, SOLO_RETRACT_M, 0.0f, 1u) == ML_STALL) t_done = t;
        now++;
    }
    TEST_ASSERT_EQUAL_UINT32(ML_STALL_MS + 1u, t_done);

    // Creeping for 5 s, then a 30 s gap: the budget ends 4917 ms after the gap, not at once.
    setUp();
    const uint32_t start_cnt = s_cnt;
    ml_pull_back_start(&g, now, s_cnt, SOLO_RETRACT_M);
    for (uint32_t t = 1; t <= 5000u; t++)
    {
        now++;
        gear_step(-5.0f);
        TEST_ASSERT_EQUAL(ML_OK, ml_pull_back_check(&g, now, s_cnt, 1000.0f, SOLO_RETRACT_M,
                                                    ml_travel_m(s_cnt, start_cnt), 1u));
    }
    now += 30000u;
    t_done = 0u;
    for (uint32_t t = 0; t <= 10000u && !t_done; t++)
    {
        gear_step(-5.0f);
        if (ml_pull_back_check(&g, now, s_cnt, 1000.0f, SOLO_RETRACT_M, ml_travel_m(s_cnt, start_cnt), 1u) == ML_TIME)
            t_done = t;
        now++;
    }
    TEST_ASSERT_EQUAL_UINT32(4917u, t_done);
}

// ---- adapted from Motion_control.cpp: a pull back pass while the ADC stream is stale ----
// ---- anchor: motor_motion_filamnet_pull_back_to_online_key from /g_pull_speed_set\[i\] = -v;/ to /filament_motion_pull, 100/ ----
// ---- anchor: _MOTOR_CONTROL from /if \(motion == _motion\) return;/ to /if \(motion == _motion\) return;/ ----
// ---- anchor: _MOTOR_CONTROL from /const bool keep_pwm =/ to /x_prev\[CHx\] = 0\.0f;/ ----
// ---- anchor: motor_brake_now ----
// ---- anchor: motor_motion_run from /if \(g_adc_stale\)/ to /return;/ ----
// motor_motion_filamnet_pull_back_to_online_key checks the pull back with x_prev, then sets the
// pull, which zeroes x_prev when it changes the motion (set_motion, keep_pwm false); then the stale
// block brakes the channel through motor_brake_now, which sets stop and zeroes x_prev too.
static bool s_pulling;  // MOTOR_CONTROL[i].motion == filament_motion_pull

static ml_result stale_pull_back_pass(float *x_prev)
{
    const ml_result r = ml_pull_back_check(&g, now, s_cnt, *x_prev, SOLO_RETRACT_M, 0.0f, 1u);
    if (!s_pulling) *x_prev = 0.0f;  // set_motion(pull): a change of motion only after a brake
    *x_prev   = 0.0f;                // motor_brake_now: stop, x_prev 0
    s_pulling = false;
    return r;
}

static void test_stale_adc_during_a_pull_back_is_not_a_stall(void)
{
    // A pull back at 1000 PWM, then the ADC stream is stale for 1.5 s: every pass brakes the motor
    // and the gear stands still. The guard reads the last pull PWM on the first stale pass and PWM 0
    // from the second on (set_motion from stop and the brake both zero x_prev), so the stall window
    // restarts on every pass; the time budget still runs.
    ml_pull_back_start(&g, now, s_cnt, SOLO_RETRACT_M);
    s_pulling = true;
    float x_prev = 1000.0f;
    for (uint32_t t = 1; t <= 1500u; t++)
    {
        now++;
        TEST_ASSERT_EQUAL(ML_OK, stale_pull_back_pass(&x_prev));
    }
    TEST_ASSERT_EQUAL_UINT32(1500u, g.run_ms);

    // Why the zeroing matters, not a state of the firmware: if a braked channel kept the last pull
    // PWM in x_prev, the same frozen count would end the pull back as ML_STALL, ML_STALL_MS after the
    // stream went stale.
    setUp();
    ml_pull_back_start(&g, now, s_cnt, SOLO_RETRACT_M);
    TEST_ASSERT_EQUAL_UINT32(ML_STALL_MS, run_move_then_block(check_pull, 0.0f, 0u, 1000.0f));
    TEST_ASSERT_EQUAL(ML_STALL, s_end);
}

static void test_budget_is_not_cut_by_32bit_ms(void)
{
    // Uptime past 2^32 ms: differences are 64-bit, nothing wraps.
    now = 0x100000000ull - 3000u;
    const uint32_t t = run_pull(SOLO_RETRACT_M, 5.0f, 1000.0f, 60000u, false);
    TEST_ASSERT_EQUAL_UINT32(9917u, t);
    TEST_ASSERT_EQUAL(ML_TIME, s_end);
}

// Redetect: pushes at 900 PWM while ks == 0. Returns the pass it ended in, 0 if it did not.
static uint32_t run_redetect(float speed_mm_s, float pwm, uint32_t ks_back_at, uint32_t max_ms)
{
    ml_redetect_start(&g, now, s_cnt, SOLO_RETRACT_M);
    for (uint32_t t = 1; t <= max_ms; t++)
    {
        now++;
        gear_step(speed_mm_s);
        const uint8_t ks = (ks_back_at && t >= ks_back_at) ? 2u : 0u;
        const ml_result r = ml_redetect_check(&g, now, s_cnt, pwm, ks);
        if (ks != 0u) TEST_ASSERT_EQUAL(ML_END, r);  // a switch that sees filament always ends it
        if (r != ML_OK)
        {
            s_end = r;
            return t;
        }
    }
    s_end = ML_OK;
    return 0u;
}

static void test_redetect_on_empty_channel_stops_after_retract_length(void)
{
    // Filament pulled out of the BMCU mid-unload: the gear spins free (120 mm/s) and never finds
    // it. It stops after 95 mm of gear travel (16515 counts) instead of spinning for good.
    gear_reset(FEED_CNT0, 1.2345f);
    const uint32_t t = run_redetect(120.0f, 900.0f, 0u, 60000u);
    TEST_ASSERT_EQUAL_UINT32(792u, t);  // 95 mm / 120 mm/s = 791.7 ms
    TEST_ASSERT_EQUAL(ML_DIST, s_end);
    TEST_ASSERT_EQUAL_UINT32(16522u, ml_cnt_dist(s_cnt, FEED_CNT0));  // across the wrap
}

static void test_redetect_reinserted_filament_ends_as_before(void)
{
    // The user feeds the filament back after 500 ms (60 mm of gear travel): a switch sees it.
    TEST_ASSERT_EQUAL_UINT32(500u, run_redetect(120.0f, 900.0f, 500u, 60000u));
    TEST_ASSERT_EQUAL(ML_END, s_end);
}

static void test_redetect_first_pass_with_filament_ends_as_before(void)
{
    // Normal unload: the switches still see filament after the pull, so redetect ends at once.
    TEST_ASSERT_EQUAL_UINT32(1u, run_redetect(0.0f, 0.0f, 1u, 60000u));
    TEST_ASSERT_EQUAL(ML_END, s_end);
}

static void test_redetect_blocked_gear_stops(void)
{
    TEST_ASSERT_EQUAL_UINT32(ML_STALL_MS, run_redetect(0.0f, 900.0f, 0u, 60000u));
    TEST_ASSERT_EQUAL(ML_STALL, s_end);
}

static void test_redetect_that_moves_then_blocks_stops_1s_after_the_block(void)
{
    // The gear turns 40.8 mm at 60 mm/s and is then blocked at the redetect's 900 PWM (it caught
    // filament that no switch sees, and that jams): the stall comes ML_STALL_MS after the block,
    // long before the 95 mm distance cap or the 9.9 s budget.
    gear_reset(FEED_CNT0, 1.2345f);
    ml_redetect_start(&g, now, s_cnt, SOLO_RETRACT_M);
    TEST_ASSERT_EQUAL_UINT32(680u + ML_STALL_MS, run_move_then_block(check_redetect, 60.0f, 680u, 900.0f));
    TEST_ASSERT_EQUAL(ML_STALL, s_end);
}

static void test_redetect_without_drive_ends_at_budget(void)
{
    // Motor direction unknown (dir 0): PWM 0, nothing moves. The budget still ends it.
    TEST_ASSERT_EQUAL_UINT32(9917u, run_redetect(0.0f, 0.0f, 0u, 60000u));
    TEST_ASSERT_EQUAL(ML_TIME, s_end);
}

static void test_redetect_slow_push_ends_at_budget(void)
{
    // 8 mm/s: no stall, 95 mm would take 11.9 s.
    TEST_ASSERT_EQUAL_UINT32(9917u, run_redetect(8.0f, 900.0f, 0u, 60000u));
    TEST_ASSERT_EQUAL(ML_TIME, s_end);
}

// DM Stage-2: returns the pass the guard stopped the stage in, 0 if it did not.
static uint32_t run_dm_s2(float speed_mm_s, float pwm, uint32_t max_ms)
{
    ml_dm_s2_start(&g, now, s_cnt, DM_S2_LEN_M);
    for (uint32_t t = 1; t <= max_ms; t++)
    {
        now++;
        gear_step(speed_mm_s);
        const ml_result r = motion_guard_check(&g, now, s_cnt, pwm);
        if (r != ML_OK)
        {
            s_end = r;
            return t;
        }
    }
    s_end = ML_OK;
    return 0u;
}

static void test_dm_s2_normal_push_is_not_limited(void)
{
    // 120 mm at 60 mm/s (2 s) ends on its own distance in the firmware; the guard stays quiet.
    gear_reset(FEED_CNT0, 1.2345f);
    TEST_ASSERT_EQUAL_UINT32(0u, run_dm_s2(60.0f, 900.0f, 2000u));
    // A retract (negative travel) of the same kind as well.
    TEST_ASSERT_EQUAL_UINT32(0u, run_dm_s2(-60.0f, -900.0f, 2000u));
}

static void test_dm_s2_blocked_gear_stops(void)
{
    TEST_ASSERT_EQUAL_UINT32(ML_STALL_MS, run_dm_s2(0.0f, -900.0f, 60000u));
    TEST_ASSERT_EQUAL(ML_STALL, s_end);
}

static void test_dm_s2_push_that_moves_then_blocks_stops_1s_after_the_block(void)
{
    // Stage-2 push, blocked after 40.8 mm (the buffer does not rise): ML_STALL_MS after the block.
    gear_reset(FEED_CNT0, 1.2345f);
    ml_dm_s2_start(&g, now, s_cnt, DM_S2_LEN_M);
    TEST_ASSERT_EQUAL_UINT32(680u + ML_STALL_MS, run_move_then_block(check_dm_s2, 60.0f, 680u, 900.0f));
    TEST_ASSERT_EQUAL(ML_STALL, s_end);
}

static void test_dm_s2_retract_that_never_relaxes_the_buffer_stops_at_budget(void)
{
    // S2_RETRACT waits for the buffer to drop to 50.2 %. If it never does while the gear turns
    // (slipping on the filament), the 12 s budget ends it. There is no distance limit for Stage-2.
    const uint32_t t = run_dm_s2(-100.0f, -900.0f, 60000u);
    TEST_ASSERT_EQUAL_UINT32(12000u, t);
    TEST_ASSERT_EQUAL(ML_TIME, s_end);
}

// ---- adapted from Motion_control.cpp: the DM_AUTO_S2_PUSH countdown ----
// ---- anchor: run from /case DM_AUTO_S2_PUSH:/ to /dm_auto_remain_m\[CHx\] = r;/ ----
// The Stage-2 push's own countdown (DM_AUTO_S2_PUSH in Motion_control.cpp): each pass subtracts
// the gear travel since the previous pass, from the count, from the 120 mm left. Returns the pass
// the countdown reached 0 in, 0 if it did not.
static uint32_t run_dm_s2_countdown(float speed_mm_s, uint32_t max_ms)
{
    float remain_m = DM_S2_LEN_M;
    uint32_t last_cnt = s_cnt;
    for (uint32_t t = 1; t <= max_ms; t++)
    {
        now++;
        gear_step(speed_mm_s);
        float r = remain_m - ml_travel_m(s_cnt, last_cnt);
        last_cnt = s_cnt;
        if (r < 0.0f) r = 0.0f;
        remain_m = r;
        if (remain_m <= 0.0f) return t;
    }
    return 0u;
}

static void test_dm_s2_countdown_is_120mm_at_any_odometer(void)
{
    // 120 mm (20861 counts) at 60 mm/s: 2000 passes, through the count wrap 11.8 mm in.
    gear_reset(FEED_CNT0, 1.2345f);
    TEST_ASSERT_EQUAL_UINT32(2000u, run_dm_s2_countdown(60.0f, 20000u));
    TEST_ASSERT_EQUAL_UINT32(20861u, ml_cnt_dist(s_cnt, FEED_CNT0));

    // At a 5000 m odometer meters stands still (see above); the countdown is the same. Counted from
    // meters it pushed 118.0-120.7 mm at odometers up to 600 m and never ended at 5000 m.
    gear_reset(FEED_CNT0, 5000.0f);
    TEST_ASSERT_EQUAL_UINT32(2000u, run_dm_s2_countdown(60.0f, 20000u));
    TEST_ASSERT_EQUAL_UINT32(20861u, ml_cnt_dist(s_cnt, FEED_CNT0));
    TEST_ASSERT_EQUAL_FLOAT(5000.0f, s_meters);
}

// ---- Send (filament_motion_send) ----

// ---- adapted from Motion_control.cpp: the send's speed PID against a blocked gear, simplified ----
// ---- anchor: MOTOR_PID ----
// ---- anchor: _MOTOR_CONTROL from /MOTOR_PID PID_speed/ to /MOTOR_PID PID_speed/ ----
// ---- anchor: run from /constexpr uint64_t SEND_SOFTSTART_MS/ to /PID_speed\.caculate\(/ ----
// ---- anchor: run from /const bool pull_mode =/ to /if \(x < \(float\)-PWM_lim\)/ ----
// ---- anchor: run from /const bool use_ramping =/ to /if \(x > hi\) x = hi;/ ----
// The send's first stage against a gear that does not turn, in 1 ms passes: the speed command
// (10 mm/s rising to 60 mm/s over the 300 ms soft start), its speed PID (P 2, I 20, the integral
// clamped at 1000) with the gear at 0 mm/s, the 500 PWM floor (pwm_zero), the 1000 PWM clamp and the
// 18000 PWM/s ramp. Returns the PWM of pass t (t >= 1).
typedef struct
{
    float i_save;
    float x;
} send_pid_model;

static float send_pwm_blocked(send_pid_model *m, uint32_t t_ms)
{
    const float v_set = (t_ms < 300u) ? (10.0f + 50.0f * (float)t_ms / 300.0f) : 60.0f;
    m->i_save += 20.0f * v_set * 0.001f;
    if (m->i_save > 1000.0f) m->i_save = 1000.0f;
    float x = 2.0f * v_set + m->i_save;
    if (x < 500.0f) x = 500.0f;
    if (x > 1000.0f) x = 1000.0f;
    if (x > m->x + 18.0f) x = m->x + 18.0f;
    m->x = x;
    return x;
}

static ml_result check_send(uint32_t pos_cnt, float pwm)
{
    return motion_guard_check(&g, now, pos_cnt, pwm);
}

// ---- adapted from Motion_control.cpp: the send's end before its guard ----
// What the firmware did before: the send stopped (send_len_abort) only after SEND_MAX_M (10 m) of
// gear travel.
static bool old_send_done(uint32_t pos_cnt, uint32_t start_cnt)
{
    return ml_travel_m(pos_cnt, start_cnt) >= 10.0f;
}

// Sends at speed_mm_s and pwm for up to passes 1 ms passes, checking every pass; returns the pass
// the guard ended the send in (s_end: why), 0 if it did not. The guard must have been started.
static uint32_t run_send(float speed_mm_s, float pwm, uint32_t passes)
{
    for (uint32_t t = 1; t <= passes; t++)
    {
        now++;
        gear_step(speed_mm_s);
        const ml_result r = check_send(s_cnt, pwm);
        if (r != ML_OK)
        {
            s_end = r;
            return t;
        }
    }
    s_end = ML_OK;
    return 0u;
}

static void test_send_cap_is_10m_of_gear_travel_as_before(void)
{
    // The gear spins at 120 mm/s and never gets the buffer to its stop (it slips on the filament, or
    // the filament goes nowhere): the send stops after 10 m, in the same pass as before, and never
    // on time, although 10 m take 83 s. From near the count wrap. (The count cap is one count short
    // of where the float comparison tripped, 1738397: a pass would have to land on 1738396.)
    TEST_ASSERT_EQUAL_FLOAT(10.0f, ML_SEND_MAX_M);
    gear_reset(FEED_CNT0, 1.2345f);
    const uint32_t start_cnt = s_cnt;
    ml_send_start(&g, now, s_cnt);
    uint32_t t_done = 0u;
    for (uint32_t t = 1; t <= 100000u && !t_done; t++)
    {
        now++;
        gear_step(120.0f);
        const ml_result r = check_send(s_cnt, 1000.0f);
        TEST_ASSERT_EQUAL(old_send_done(s_cnt, start_cnt), r != ML_OK);
        if (r != ML_OK)
        {
            TEST_ASSERT_EQUAL(ML_DIST, r);
            t_done = t;
        }
    }
    TEST_ASSERT_EQUAL_UINT32(83334u, t_done);  // 10 m / 120 mm/s = 83333.3 ms
    // The first pass at or past 1738396 counts (10 m); 120 mm/s is 20.9 counts a pass.
    TEST_ASSERT_EQUAL_UINT32(1738396u, ml_m_to_cnt(ML_SEND_MAX_M));
    TEST_ASSERT_EQUAL_UINT32(1738411u, ml_cnt_dist(s_cnt, FEED_CNT0));
}

static void test_send_has_no_time_budget(void)
{
    // The printer decides how long a send lasts. A slow feed (5 mm/s at 1000 PWM: 1 mm every 200 ms,
    // no stall) goes on for 30 min (9 m); the pull back's rule would have ended it after 835 s.
    ml_send_start(&g, now, s_cnt);
    TEST_ASSERT_EQUAL_UINT32(0u, run_send(5.0f, 1000.0f, 1800000u));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.0f, ml_travel_m(s_cnt, PULL_CNT0));

    // Held for 2 h with the buffer at its stop (no drive, the gear still), in 100 ms passes.
    setUp();
    ml_send_start(&g, now, s_cnt);
    for (uint32_t t = 1; t <= 72000u; t++)
    {
        now += 100u;
        TEST_ASSERT_EQUAL(ML_OK, check_send(s_cnt, 0.0f));
    }
}

static void test_blocked_send_stops_1s_after_reaching_800_pwm(void)
{
    // A tangle on the spool: the gear does not turn from the start (+-1 count of jitter), the buffer
    // never rises, the PID winds up to 1000 PWM. Before, that went on as long as the printer sent
    // send_out; now the send stops ML_STALL_MS after the PWM reached 800.
    const uint32_t start = s_cnt;
    ml_send_start(&g, now, start);

    send_pid_model m = {0.0f, 0.0f};
    uint32_t t800 = 0u, t1000 = 0u, t_done = 0u;
    float pwm_prev = 0.0f;  // PWM applied since the previous pass (x_prev)
    for (uint32_t t = 1; t <= 60000u && !t_done; t++)
    {
        now++;
        const uint32_t p = ((t & 1u) != 0u) ? (start + 1u) : (start - 1u);
        const ml_result r = check_send(p, pwm_prev);
        TEST_ASSERT_FALSE(old_send_done(p, start));  // before: never
        if (r != ML_OK)
        {
            TEST_ASSERT_EQUAL(ML_STALL, r);
            t_done = t;
        }
        pwm_prev = send_pwm_blocked(&m, t);
        if (!t800 && pwm_prev >= ML_STALL_PWM) t800 = t;
        if (!t1000 && pwm_prev >= 1000.0f) t1000 = t;
    }
    TEST_ASSERT_EQUAL_UINT32(692u, t800);   // 500 PWM floor until 442 ms, then +1.2 PWM a pass
    TEST_ASSERT_EQUAL_UINT32(t800 + ML_STALL_MS, t_done);
    TEST_ASSERT_EQUAL_UINT32(858u, t1000);

    // The printer's next command after the stop, or the next send_out after it, starts a new guard:
    // the same 1.7 s against the same tangle, and a full load once the spool is free.
    ml_send_start(&g, now, s_cnt);
    TEST_ASSERT_EQUAL_UINT32(ML_STALL_MS, run_move_then_block(check_send, 0.0f, 0u, 1000.0f));
    TEST_ASSERT_EQUAL(ML_STALL, s_end);
    ml_send_start(&g, now, s_cnt);
    TEST_ASSERT_EQUAL_UINT32(0u, run_send(60.0f, 1000.0f, 20000u));
}

static void test_send_that_feeds_then_blocks_stops_1s_after_the_block(void)
{
    // 40.8 mm at 60 mm/s, then the spool catches and the gear stops at 1000 PWM.
    gear_reset(FEED_CNT0, 1.2345f);
    ml_send_start(&g, now, s_cnt);
    TEST_ASSERT_EQUAL_UINT32(680u + ML_STALL_MS, run_move_then_block(check_send, 60.0f, 680u, 1000.0f));
    TEST_ASSERT_EQUAL(ML_STALL, s_end);

    // The same stall at 0.8 mm/s, and none at 1.5 mm/s (1 mm every 667 ms), as for the pull back.
    setUp();
    ml_send_start(&g, now, s_cnt);
    TEST_ASSERT_EQUAL_UINT32(ML_STALL_MS, run_send(0.8f, 1000.0f, 60000u));
    TEST_ASSERT_EQUAL(ML_STALL, s_end);
    setUp();
    ml_send_start(&g, now, s_cnt);
    TEST_ASSERT_EQUAL_UINT32(0u, run_send(1.5f, 1000.0f, 60000u));
}

static void test_normal_a1_load_is_not_limited(void)
{
    // A whole load on the A1, with the guard checked on every pass as the firmware does it:
    gear_reset(FEED_CNT0, 1.2345f);
    ml_send_start(&g, now, s_cnt);

    // the 300 ms soft start (10 to 60 mm/s) and 20 s at 60 mm/s through the tubes (1.2 m, more than
    // the A1's path), at 1000 PWM (heavy drag, the worst case);
    for (uint32_t t = 1; t <= 300u; t++)
    {
        now++;
        gear_step(10.0f + 50.0f * (float)t / 300.0f);
        TEST_ASSERT_EQUAL(ML_OK, check_send(s_cnt, 1000.0f));
    }
    TEST_ASSERT_EQUAL_UINT32(0u, run_send(60.0f, 1000.0f, 20000u));

    // the tip at the extruder: the gear slows from 60 to 2 mm/s over 600 ms while it pushes the
    // buffer to 85 % (18.6 mm), still at 1000 PWM;
    for (uint32_t t = 1; t <= 600u; t++)
    {
        now++;
        gear_step(60.0f - 58.0f * (float)t / 600.0f);
        TEST_ASSERT_EQUAL(ML_OK, check_send(s_cnt, 1000.0f));
    }

    // hold_load then brings the buffer to 90 %. A buffer whose end is at 85 % never gets there: its
    // 5-point error keeps the on_use anti-stall going for 60 s with the gear still: 732 PWM for
    // 150 ms, the 850 PWM kick up to 0.8 s, then a 500 ms rest, over and over. With +-1 count of
    // jitter the speed reads 5.75 mm/s on every other pass, so the anti-stall never counts and the
    // hold stays at 732 PWM, under ML_STALL_PWM, for another 60 s;
    const uint32_t hold_cnt = s_cnt;
    for (uint32_t t = 0; t < 60000u; t++)
    {
        now++;
        const uint32_t c = t % 1300u;
        const float pwm = (c < 150u) ? 732.0f : ((c < 800u) ? 850.0f : 0.0f);
        TEST_ASSERT_EQUAL(ML_OK, check_send(hold_cnt, pwm));
    }
    for (uint32_t t = 0; t < 60000u; t++)
    {
        now++;
        const uint32_t p = ((t & 1u) != 0u) ? (hold_cnt + 1u) : hold_cnt;
        TEST_ASSERT_EQUAL(ML_OK, check_send(p, 732.0f));
    }

    // and a buffer at its 95 % hard stop is braked (0 PWM) for another 60 s, still in send_out.
    for (uint32_t t = 0; t < 60000u; t++)
    {
        now++;
        TEST_ASSERT_EQUAL(ML_OK, check_send(hold_cnt, 0.0f));
    }
}

// ---- Idle control push (filament_motion_pressure_ctrl_idle) ----

// ---- adapted from Motion_control.cpp: the rounded buffer reading MC_PULL_pct ----
// ---- anchor: MC_PULL_ONLINE_read from /int pct = \(int\)\(pct_f \+ 0\.5f\);/ to /MC_PULL_pct\[i\] = \(uint8_t\)pct;/ ----
static uint8_t idle_pct(float pct_f)
{
    int pct = (int)(pct_f + 0.5f);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (uint8_t)pct;
}

// ---- adapted from Motion_control.cpp: the idle control's push with filament at the switches ----
// ---- anchor: MC_PULL_ONLINE_read from /MC_PULL_DEADBAND_PCT_HIGH\) MC_PULL_stu/ to /MC_PULL_DEADBAND_PCT_LOW\)/ ----
// ---- anchor: MC_PULL_DEADBAND_PCT_LOW ----
// ---- anchor: MC_PULL_PIDP_PCT ----
// ---- anchor: MC_HOLD_PWM_MIN ----
// ---- anchor: run from /normalny idle z filamentem/ to /PID_pressure\.clear\(\);/ ----
// ---- anchor: run from /const bool pull_mode =/ to /if \(x < \(float\)-PWM_lim\)/ ----
// With the rounded buffer reading below MC_PULL_DEADBAND_PCT_LOW (30 %, MC_PULL_stu -1) the pressure
// PID (P MC_PULL_PIDP_PCT 25, no I or D) drives towards 50 %, raised to the 420 PWM hold floor and
// clamped at the idle control's 800 PWM. Returns the push PWM (its magnitude), 0 when it does not push.
static float idle_push_pwm(float pct_f)
{
    if (idle_pct(pct_f) >= 30u) return 0.0f;
    float x = 25.0f * (50.0f - pct_f);
    if (x < 420.0f) x = 420.0f;
    if (x > 800.0f) x = 800.0f;
    return x;
}

static ml_idle_push_t ip;
static float ip_pwm;              // PWM applied since the previous pass (x_prev): 0 when braked or not pushing
static ml_idle_push_act ip_act;   // what the last pass did

static void idle_reset(void)
{
    ml_idle_push_reset(&ip);
    ip_pwm = 0.0f;
    ip_act = ML_IDLE_PUSH_RUN;
}

// One 1 ms pass of the idle control with filament at the switches, with the gear at pos_cnt: run()
// decides with the buffer at pct_f. Returns true when it brakes (ip_act: with or without the red).
static bool idle_pass_at(float pct_f, uint32_t pos_cnt)
{
    now++;
    const float cmd = idle_push_pwm(pct_f);
    ip_act = ml_idle_push_pass(&ip, cmd > 0.0f, idle_pct(pct_f), now, pos_cnt, ip_pwm);
    ip_pwm = (ip_act != ML_IDLE_PUSH_RUN) ? 0.0f : cmd;
    return ip_act != ML_IDLE_PUSH_RUN;
}

// The same, after the gear moved v_mm_s (feeding) since the previous pass.
static bool idle_pass(float pct_f, float v_mm_s)
{
    gear_step(v_mm_s);
    return idle_pass_at(pct_f, s_cnt);
}

// passes idle passes at pct_f and v_mm_s; returns the first one that brakes, 0 if none does.
static uint32_t run_idle(float pct_f, float v_mm_s, uint32_t passes)
{
    for (uint32_t t = 1; t <= passes; t++)
        if (idle_pass(pct_f, v_mm_s)) return t;
    return 0u;
}

// The same with the gear held (+-1 count of jitter around where it is).
static uint32_t run_idle_held(float pct_f, uint32_t passes)
{
    const uint32_t start = s_cnt;
    for (uint32_t t = 1; t <= passes; t++)
        if (idle_pass_at(pct_f, ((t & 1u) != 0u) ? (start + 1u) : (start - 1u))) return t;
    return 0u;
}

static void test_idle_push_stall_level_is_the_hold_floor(void)
{
    // The idle control pushes at 512.5 PWM at the edge of its deadband (29.49 %, which rounds to 29)
    // and reaches its 800 PWM clamp at 18 %. ML_STALL_PWM (800) would only see a buffer at or below
    // 18 %; the idle stall level is the 420 PWM hold floor, under every push it makes.
    TEST_ASSERT_EQUAL_FLOAT(0.0f, idle_push_pwm(29.5f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 512.75f, idle_push_pwm(29.49f));
    TEST_ASSERT_EQUAL_FLOAT(800.0f, idle_push_pwm(18.0f));
    TEST_ASSERT_TRUE(idle_push_pwm(18.1f) < ML_STALL_PWM);
    TEST_ASSERT_EQUAL_FLOAT(420.0f, ML_IDLE_STALL_PWM);
    TEST_ASSERT_TRUE(idle_push_pwm(29.49f) >= ML_IDLE_STALL_PWM);
    TEST_ASSERT_EQUAL_UINT32(10000u, ML_IDLE_PUSH_MAX_MS);
    // The edge band: rounded 28 and 29 %, 512.5-562.5 PWM; from 27.49 % (rounded 27) the red.
    TEST_ASSERT_EQUAL_UINT8(28u, ML_IDLE_EDGE_PCT);
    TEST_ASSERT_EQUAL_UINT8(28u, idle_pct(27.5f));
    TEST_ASSERT_EQUAL_UINT8(27u, idle_pct(27.49f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 562.5f, idle_push_pwm(27.5f));
}

static void test_blocked_idle_push_stops_1s_after_it_starts(void)
{
    // A parked channel whose filament is held tight (a tangle on the spool, the filament end fixed
    // to an empty spool's core): the buffer stays below 30 % and the gear does not turn (+-1 count
    // of jitter). Before, the idle control pushed for good, at 512-800 PWM depending on the buffer.
    // Now it brakes ML_STALL_MS after the first push pass, at the edge of the deadband as at 0 %:
    // with the red below 28 %, without it at 28-29 %.
    const float levels[] = {29.4f, 27.5f, 27.49f, 25.0f, 18.0f, 0.0f};
    const ml_idle_push_act acts[] = {ML_IDLE_PUSH_BRAKE, ML_IDLE_PUSH_BRAKE, ML_IDLE_PUSH_FAULT,
                                     ML_IDLE_PUSH_FAULT, ML_IDLE_PUSH_FAULT, ML_IDLE_PUSH_FAULT};
    for (uint32_t i = 0; i < sizeof(levels) / sizeof(levels[0]); i++)
    {
        setUp();
        idle_reset();
        TEST_ASSERT_EQUAL_UINT32(1u + ML_STALL_MS, run_idle_held(levels[i], 60000u));
        TEST_ASSERT_EQUAL_INT(acts[i], ip_act);
    }

    // 40.8 mm at 60 mm/s (the buffer does not rise: the filament goes on into the tube), then the
    // spool catches: ML_STALL_MS after the gear's last full 1 mm. The window restarts about every 17
    // passes at 60 mm/s; with the push's guard started after the first pass's step, it last
    // restarted 16 passes before the block.
    setUp();
    idle_reset();
    TEST_ASSERT_EQUAL_UINT32(0u, run_idle(20.0f, 60.0f, 680u));
    TEST_ASSERT_EQUAL_UINT32(ML_STALL_MS - 16u, run_idle(20.0f, 0.0f, 60000u));
    TEST_ASSERT_EQUAL_INT(ML_IDLE_PUSH_FAULT, ip_act);
}

static void test_idle_push_that_moves_but_never_ends_stops_at_its_budget(void)
{
    // The gear turns but the buffer never comes back to 30 %: it reads low (a sensor or calibration
    // fault), or the gear slips on the filament, or the filament goes on into the printer's path
    // without pushing the buffer up. At 60 mm/s that is 600 mm in 10 s; before, it never ended.
    idle_reset();
    TEST_ASSERT_EQUAL_UINT32(1u + ML_IDLE_PUSH_MAX_MS, run_idle(20.0f, 60.0f, 60000u));
    TEST_ASSERT_EQUAL_INT(ML_IDLE_PUSH_FAULT, ip_act);
    // A creeping gear (2 mm/s at the 800 PWM clamp: no stall) as well.
    setUp();
    idle_reset();
    TEST_ASSERT_EQUAL_UINT32(1u + ML_IDLE_PUSH_MAX_MS, run_idle(10.0f, 2.0f, 60000u));
    TEST_ASSERT_EQUAL_INT(ML_IDLE_PUSH_FAULT, ip_act);
    // A buffer that rests just under 30 % (its reading centred on 1.65 V, not on the calibrated
    // rest) with free filament fed slowly at 512-537 PWM: braked after 10 s too, without the red.
    setUp();
    idle_reset();
    TEST_ASSERT_EQUAL_UINT32(1u + ML_IDLE_PUSH_MAX_MS, run_idle(29.0f, 1.5f, 60000u));
    TEST_ASSERT_EQUAL_INT(ML_IDLE_PUSH_BRAKE, ip_act);
}

static void test_normal_idle_pushes_are_not_limited(void)
{
    // Every normal push ends when the buffer is back at 30 %: it only takes up the slack the buffer
    // shows, while the printer does not draw filament from an idle channel.
    // - After the BMCU's pull back (the printer's unload, a channel switch), also one stopped by a
    //   stall with the filament still held in the tool head: the buffer at its low end, the push
    //   feeds until it is back at 30 %. Modelled as 1 % of buffer a mm (a 100 mm buffer, far more
    //   than the BMCU's), at 12 mm/s (ML_V_MIN_MM_S, heavy drag) from 0 %: 30 mm, 2.5 s.
    idle_reset();
    float pct = 0.0f;
    uint32_t t = 0u;
    while (idle_push_pwm(pct) > 0.0f)
    {
        TEST_ASSERT_FALSE(idle_pass(pct, 12.0f));
        pct += 0.012f;
        t++;
    }
    TEST_ASSERT_UINT32_WITHIN(2u, 2458u, t);
    TEST_ASSERT_FALSE(idle_pass(pct, 12.0f));  // in the deadband: no push, no limit

    // - The same at 1.5 mm/s (1 mm every 667 ms, no stall) for 9.9 s, just inside the budget.
    setUp();
    idle_reset();
    TEST_ASSERT_EQUAL_UINT32(0u, run_idle(10.0f, 1.5f, ML_IDLE_PUSH_MAX_MS - 100u));
    TEST_ASSERT_EQUAL_UINT32(0u, run_idle(40.0f, 0.0f, 1u));

    // - The printer draws filament through a parked or idle channel (its end-of-print moves, a
    //   filament the printer pulled): the push follows in bursts, each ending back at 30 %. 300 ms
    //   pushing at 29 % with the gear feeding, 200 ms in the deadband, for 10 min.
    setUp();
    idle_reset();
    for (uint32_t i = 0; i < 1200u; i++)
    {
        TEST_ASSERT_EQUAL_UINT32(0u, run_idle(29.0f, 20.0f, 300u));
        TEST_ASSERT_EQUAL_UINT32(0u, run_idle(31.0f, 0.0f, 200u));
    }

    // - A hand that presses the buffer down (the DM buffer gesture: below 10 % for at most 2 s)
    //   while the gear still feeds, then lets it go back to the middle.
    setUp();
    idle_reset();
    TEST_ASSERT_EQUAL_UINT32(0u, run_idle(5.0f, 30.0f, 2000u));
    TEST_ASSERT_EQUAL_UINT32(0u, run_idle(50.0f, 0.0f, 100u));
}

static void test_idle_push_that_stops_at_the_deadband_edge_shows_no_fault(void)
{
    // The slack after a pull back taken up as above, but the gear stops at 28 %, short of the
    // deadband: spool drag and the gearbox need more than the PID's 550 PWM there, with the tension
    // downstream almost gone. Braked 1 s later (the current this guard is for), with no red: the
    // channel is held as if in the deadband, for as long as the buffer stays there.
    idle_reset();
    float pct = 0.0f;
    while (pct < 28.0f)
    {
        TEST_ASSERT_FALSE(idle_pass(pct, 12.0f));
        pct += 0.012f;
    }
    // The stall window last restarted within the gear's last 1 mm (84 passes at 12 mm/s).
    const uint32_t t_brake = run_idle_held(pct, 60000u);
    TEST_ASSERT_TRUE((t_brake > ML_STALL_MS - 84u) && (t_brake <= ML_STALL_MS));
    TEST_ASSERT_EQUAL_INT(ML_IDLE_PUSH_BRAKE, ip_act);
    for (uint32_t i = 0; i < 60000u; i++)
    {
        TEST_ASSERT_TRUE(idle_pass_at((i & 1u) ? 27.5f : 29.49f, s_cnt));
        TEST_ASSERT_EQUAL_INT(ML_IDLE_PUSH_BRAKE, ip_act);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, ip_pwm);
    }

    // The buffer back in the deadband clears it, as the red.
    TEST_ASSERT_FALSE(idle_pass(30.0f, 0.0f));
    TEST_ASSERT_EQUAL_UINT8(0u, ip.fault);

    // Braked at the edge, then more filament is drawn (the buffer falls to 20 %): a new push at the
    // higher PWM, not held by the edge brake, with its whole budget. A gear that then turns takes up
    // the slack; one that is still held is braked again 1 s after the new push starts, with the red.
    setUp();
    idle_reset();
    TEST_ASSERT_EQUAL_UINT32(1u + ML_STALL_MS, run_idle_held(28.0f, 60000u));
    TEST_ASSERT_EQUAL_INT(ML_IDLE_PUSH_BRAKE, ip_act);
    TEST_ASSERT_EQUAL_UINT32(0u, run_idle(20.0f, 30.0f, ML_IDLE_PUSH_MAX_MS));
    TEST_ASSERT_EQUAL_FLOAT(idle_push_pwm(20.0f), ip_pwm);
    TEST_ASSERT_FALSE(idle_pass(30.0f, 0.0f));
    TEST_ASSERT_EQUAL_UINT32(1u + ML_STALL_MS, run_idle_held(28.0f, 60000u));
    TEST_ASSERT_EQUAL_INT(ML_IDLE_PUSH_BRAKE, ip_act);
    TEST_ASSERT_EQUAL_UINT32(1u + ML_STALL_MS, run_idle_held(20.0f, 60000u));
    TEST_ASSERT_EQUAL_INT(ML_IDLE_PUSH_FAULT, ip_act);

    // The red is not lowered to the edge brake by a buffer that rises into the band: it holds until
    // the push ends.
    TEST_ASSERT_TRUE(idle_pass_at(29.0f, s_cnt));
    TEST_ASSERT_EQUAL_INT(ML_IDLE_PUSH_FAULT, ip_act);
    TEST_ASSERT_FALSE(idle_pass(29.5f, 0.0f));
}

static void test_idle_push_fault_holds_until_the_control_stops_pushing(void)
{
    // Stalled: braked (0 PWM) and red on every pass while the buffer stays below 30 %, for as long
    // as that lasts, and the gear is still not moving.
    idle_reset();
    TEST_ASSERT_EQUAL_UINT32(1u + ML_STALL_MS, run_idle(20.0f, 0.0f, 60000u));
    for (uint32_t i = 0; i < 60000u; i++)
    {
        TEST_ASSERT_TRUE(idle_pass((i & 1u) ? 20.0f : 29.4f, 0.0f));
        TEST_ASSERT_EQUAL_INT(ML_IDLE_PUSH_FAULT, ip_act);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, ip_pwm);
    }

    // The buffer back in the deadband (the spool freed, the filament fed or the buffer let go): the
    // fault clears on that pass, and the next push starts over with the whole budget and window.
    TEST_ASSERT_FALSE(idle_pass(30.0f, 0.0f));
    TEST_ASSERT_EQUAL_UINT32(0u, run_idle(20.0f, 60.0f, ML_IDLE_PUSH_MAX_MS));
    TEST_ASSERT_EQUAL_UINT32(1u, run_idle(20.0f, 60.0f, 1u));

    // Above 70 % the idle control retracts: not a push, so that clears it as well.
    TEST_ASSERT_TRUE(idle_pass(20.0f, 0.0f));
    TEST_ASSERT_FALSE(idle_pass(75.0f, 0.0f));
    TEST_ASSERT_FALSE(idle_pass(20.0f, 0.0f));

    // No filament at the switches (the idle control's key-0 branches, which do not push through it)
    // or the printer moving the channel (set_motion): ml_idle_push_reset. The next push starts over.
    setUp();
    idle_reset();
    TEST_ASSERT_EQUAL_UINT32(1u + ML_STALL_MS, run_idle(20.0f, 0.0f, 60000u));
    ml_idle_push_reset(&ip);
    ip_pwm = 0.0f;
    TEST_ASSERT_EQUAL_UINT32(1u + ML_STALL_MS, run_idle(20.0f, 0.0f, 60000u));
    TEST_ASSERT_EQUAL_INT(ML_IDLE_PUSH_RUN, ml_idle_push_pass(&ip, false, 50u, now + 1u, s_cnt, 0.0f));
    TEST_ASSERT_EQUAL_UINT8(0u, ip.fault);
}

// ---- Unfinished unload fault (status LED) ----

static void test_only_a_limit_is_an_unload_fault(void)
{
    TEST_ASSERT_TRUE(ml_is_limit(ML_TIME));
    TEST_ASSERT_TRUE(ml_is_limit(ML_DIST));
    TEST_ASSERT_TRUE(ml_is_limit(ML_STALL));
    TEST_ASSERT_FALSE(ml_is_limit(ML_OK));
    TEST_ASSERT_FALSE(ml_is_limit(ML_END));  // target reached or switches empty: a normal unload
}

static void test_unload_fault_stays_until_filament_out_or_next_move(void)
{
    // Set by a stalled pull, kept through the redetect and idle while the filament is still at the
    // switches, also while the printer works with other slots (the channel stays idle).
    TEST_ASSERT_TRUE(ml_unload_fault_kept(true, 1u, false));
    TEST_ASSERT_TRUE(ml_unload_fault_kept(true, 2u, false));
    // The user pulls the filament out of the BMCU.
    TEST_ASSERT_FALSE(ml_unload_fault_kept(true, 0u, false));
    // The printer loads, prints from or unloads this channel again.
    TEST_ASSERT_FALSE(ml_unload_fault_kept(true, 1u, true));
    // It is never set by the keep rule itself.
    TEST_ASSERT_FALSE(ml_unload_fault_kept(false, 1u, false));
}

static void test_stalled_pull_sets_the_fault_and_a_good_retry_clears_it(void)
{
    // The firmware's sequence: pb_unload_fault = ml_is_limit(end of the pull), then the keep rule
    // on every pass. A held filament stops the pull: fault set, the channel goes idle, fault kept.
    bool fault = false;
    ml_pull_back_start(&g, now, s_cnt, SOLO_RETRACT_M);
    TEST_ASSERT_EQUAL_UINT32(ML_STALL_MS, run_move_then_block(check_pull, -60.0f, 0u, 1000.0f));
    fault = ml_is_limit(s_end);
    TEST_ASSERT_TRUE(fault);
    fault = ml_unload_fault_kept(fault, 1u, false);
    TEST_ASSERT_TRUE(fault);

    // The user frees the filament and unloads again from the screen: the new pull back clears the
    // fault when it starts (channel busy), and it ends normally, so it stays clear.
    fault = ml_unload_fault_kept(fault, 1u, true);
    TEST_ASSERT_FALSE(fault);
    run_pull(SOLO_RETRACT_M, 0.0f, 650.0f, 20000u, true);
    fault = ml_is_limit(s_end);
    TEST_ASSERT_FALSE(fault);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_time_budget_is_distance_at_12mm_s_plus_2s);
    RUN_TEST(test_distances_are_whole_counts_modulo_2_32);
    RUN_TEST(test_travel_m_is_the_count_distance_at_any_position);
    RUN_TEST(test_solo_unload_ends_at_target_as_before);
    RUN_TEST(test_solo_unload_at_full_pwm_ends_at_target_as_before);
    RUN_TEST(test_long_retract_ends_at_target_as_before);
    RUN_TEST(test_slow_but_moving_pull_still_reaches_target);
    RUN_TEST(test_pull_ends_when_switches_empty_as_before);
    RUN_TEST(test_zero_target_ends_at_once_as_before);
    RUN_TEST(test_blocked_pull_stops_1s_after_reaching_800_pwm);
    RUN_TEST(test_pull_that_moves_then_blocks_stops_1s_after_the_block);
    RUN_TEST(test_stall_needs_high_pwm);
    RUN_TEST(test_creeping_pull_stops_at_time_budget);
    RUN_TEST(test_long_retract_time_budget);
    RUN_TEST(test_stall_threshold_is_1mm_per_s);
    RUN_TEST(test_pull_at_5000m_odometer_is_not_a_false_stall);
    RUN_TEST(test_pull_end_does_not_depend_on_the_odometer);
    RUN_TEST(test_bus_offline_gap_is_not_counted);
    RUN_TEST(test_stale_adc_during_a_pull_back_is_not_a_stall);
    RUN_TEST(test_budget_is_not_cut_by_32bit_ms);
    RUN_TEST(test_redetect_on_empty_channel_stops_after_retract_length);
    RUN_TEST(test_redetect_reinserted_filament_ends_as_before);
    RUN_TEST(test_redetect_first_pass_with_filament_ends_as_before);
    RUN_TEST(test_redetect_blocked_gear_stops);
    RUN_TEST(test_redetect_that_moves_then_blocks_stops_1s_after_the_block);
    RUN_TEST(test_redetect_without_drive_ends_at_budget);
    RUN_TEST(test_redetect_slow_push_ends_at_budget);
    RUN_TEST(test_dm_s2_normal_push_is_not_limited);
    RUN_TEST(test_dm_s2_blocked_gear_stops);
    RUN_TEST(test_dm_s2_push_that_moves_then_blocks_stops_1s_after_the_block);
    RUN_TEST(test_dm_s2_retract_that_never_relaxes_the_buffer_stops_at_budget);
    RUN_TEST(test_dm_s2_countdown_is_120mm_at_any_odometer);
    RUN_TEST(test_send_cap_is_10m_of_gear_travel_as_before);
    RUN_TEST(test_send_has_no_time_budget);
    RUN_TEST(test_blocked_send_stops_1s_after_reaching_800_pwm);
    RUN_TEST(test_send_that_feeds_then_blocks_stops_1s_after_the_block);
    RUN_TEST(test_normal_a1_load_is_not_limited);
    RUN_TEST(test_idle_push_stall_level_is_the_hold_floor);
    RUN_TEST(test_blocked_idle_push_stops_1s_after_it_starts);
    RUN_TEST(test_idle_push_that_moves_but_never_ends_stops_at_its_budget);
    RUN_TEST(test_normal_idle_pushes_are_not_limited);
    RUN_TEST(test_idle_push_that_stops_at_the_deadband_edge_shows_no_fault);
    RUN_TEST(test_idle_push_fault_holds_until_the_control_stops_pushing);
    RUN_TEST(test_only_a_limit_is_an_unload_fault);
    RUN_TEST(test_unload_fault_stays_until_filament_out_or_next_move);
    RUN_TEST(test_stalled_pull_sets_the_fault_and_a_good_retry_clears_it);
    return UNITY_END();
}
