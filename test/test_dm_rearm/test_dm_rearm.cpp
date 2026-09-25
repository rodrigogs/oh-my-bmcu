// Host tests for src/dm_rearm.h, driven through dm_rearm_pass() in 1 ms main-loop passes the way
// motor_motion_run calls it for a channel whose switches are wired. In idle, the DM autoload runs
// Stage-2 (120 mm pushed open-loop at 900 PWM) whenever the key reads 'both' and the channel is not
// loaded, so these tests check when a loaded channel stops being loaded: a key that flickers or
// chatters away from 'both' (with no gear motion, with an idle buffer correction, or while the
// printer moves the filament) must keep it loaded; filament taken out past both switches, or
// retracted by the gear 10 mm out of 'both' in idle, must arm Stage-2 again; a normal first
// insertion and a boot with the key at 'external only' must autoload as before, and a printer load
// that ends at 'both' must mark the channel loaded.

#include <stdint.h>
#include <unity.h>

#include "dm_rearm.h"
#include "motion_limits.h" // ML_MM_PER_CNT, the AS5600 scale Motion_control.cpp asserts

#define KS_NONE DM_KEY_NONE
#define KS_BOTH DM_KEY_BOTH
// ---- adapted from Motion_control.cpp: dm_key_to_state()'s other two key states ----
#define KS_EXT  2u  // external switch only
#define KS_OTHER 3u // any other key voltage between 'none' and 'external only'

static const _filament_motion IDLE = _filament_motion::idle;
static const _filament_motion SEND_OUT = _filament_motion::send_out;
static const _filament_motion ON_USE = _filament_motion::on_use;
static const _filament_motion BEFORE_PULL_BACK = _filament_motion::before_pull_back;
static const _filament_motion PULL_BACK = _filament_motion::pull_back;
static const _filament_motion BEFORE_ON_USE = _filament_motion::before_on_use;
static const _filament_motion STOP_ON_USE = _filament_motion::stop_on_use;

static uint64_t now;          // ms (time_now)
static uint8_t loaded;        // dm_loaded[ch]
static uint64_t away_t0;      // dm_loaded_drop_t0_ms[ch]
static uint32_t away_cnt;     // dm_loaded_drop_cnt[ch]
static uint8_t ks;            // MC_ONLINE_key_stu[ch]
static _filament_motion printer; // A.filament[ch].motion
static bool active;           // A.now_filament_num == ch
static bool unloading;        // filament_pulling_back / filament_redetect
static uint32_t cnt;          // as5600_count[ch]
static double retract_mm;     // gear travel since gear_reset(), + = retract
static uint32_t cnt0;
static int events[4];         // count of each dm_rearm_event
static int32_t last_event_ms; // now of the last event other than DM_REARM_NONE, relative to mark
static uint64_t mark;

// ---- adapted from Motion_control.cpp: when run()'s DM block starts Stage-2 from IDLE ----
// Stage-2 would start on this pass: the DM block in run() only runs for a channel in idle that is
// not loaded, and from IDLE it goes to S2_PUSH when the key reads 'both'.
static bool stage2_armed(void) { return (loaded == 0u) && (ks == KS_BOTH); }

static void gear_reset(uint32_t c0)
{
    cnt0 = c0;
    cnt = c0;
    retract_mm = 0.0;
}

// The gear moves v_mm_s (> 0 retracts: as5600_count rises) for 1 ms, read once.
static void gear_step(double v_mm_s)
{
    retract_mm += v_mm_s * 0.001;
    const double c = retract_mm / (double)ML_MM_PER_CNT;
    const int32_t whole = (c >= 0.0) ? (int32_t)c : -(int32_t)(-c);
    cnt = cnt0 + (uint32_t)whole;
}

void setUp(void)
{
    now = 5000u;
    loaded = 1u; // e.g. after a Stage-2, or a boot with the key at 'both'
    away_t0 = 0u;
    away_cnt = 0u;
    ks = KS_BOTH;
    printer = IDLE;
    active = false;
    unloading = false;
    gear_reset(0x12345678u);
    for (int i = 0; i < 4; i++) events[i] = 0;
    last_event_ms = -1;
    mark = now;
}

void tearDown(void) {}

// ---- adapted from Motion_control.cpp: motor_motion_run's dm_rearm_pass() call for one channel ----
// One main-loop pass: the AS5600 read (gear_step) comes first in Motion_control_run.
static dm_rearm_event pass(uint8_t key, double v_mm_s)
{
    ks = key;
    gear_step(v_mm_s);
    const dm_host_t host = dm_host_from_motion(active, printer, unloading);
    const dm_rearm_event ev = dm_rearm_pass(&loaded, &away_t0, &away_cnt, ks, host, now, cnt);
    events[ev]++;
    if (ev != DM_REARM_NONE) last_event_ms = (int32_t)(now - mark);
    now++;
    return ev;
}

// ms passes at a constant key and gear speed. Returns the ms offset (0 = first pass of this call)
// of the first event other than DM_REARM_NONE, or -1.
static int32_t hold(uint8_t key, double v_mm_s, uint32_t ms)
{
    int32_t first = -1;
    for (uint32_t i = 0; i < ms; i++)
        if ((pass(key, v_mm_s) != DM_REARM_NONE) && (first < 0)) first = (int32_t)i;
    return first;
}

static void set_printer(bool is_active, _filament_motion m)
{
    active = is_active;
    printer = m;
}

// ---- Flicker and chatter: no re-arm ----

void test_flicker_to_external_only_keeps_the_channel_loaded(void)
{
    // The finding: a parked, loaded channel whose key reads 'external only' for 100 ms or more.
    const uint32_t lens[] = {99u, 100u, 101u, 150u, 1000u, 60000u};
    for (unsigned i = 0; i < sizeof(lens) / sizeof(lens[0]); i++)
    {
        TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 0.0, lens[i]));
        TEST_ASSERT_EQUAL_UINT8(1u, loaded);
        TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 0.0, 1u));
        TEST_ASSERT_FALSE(stage2_armed());
        TEST_ASSERT_EQUAL_UINT8(1u, loaded);
    }
    TEST_ASSERT_EQUAL_INT(0, events[DM_REARM_RETRACTED]);
}

void test_flicker_to_the_other_key_state_keeps_the_channel_loaded(void)
{
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_OTHER, 0.0, 5000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 0.0, 1u));
    TEST_ASSERT_FALSE(stage2_armed());
    TEST_ASSERT_EQUAL_UINT8(1u, loaded);
}

void test_chatter_around_the_both_threshold_keeps_the_channel_loaded(void)
{
    // 10 s of the key toggling between 'both' and the band below it, at several rates.
    const uint32_t periods[] = {1u, 7u, 50u, 99u, 150u, 400u};
    for (unsigned p = 0; p < sizeof(periods) / sizeof(periods[0]); p++)
    {
        for (uint32_t t = 0; t < 10000u; t++)
        {
            const uint8_t key = (((t / periods[p]) & 1u) != 0u) ? KS_BOTH : ((p & 1u) ? KS_OTHER : KS_EXT);
            TEST_ASSERT_EQUAL(DM_REARM_NONE, pass(key, 0.0));
            TEST_ASSERT_FALSE(stage2_armed());
        }
    }
    TEST_ASSERT_EQUAL_UINT8(1u, loaded);
}

void test_flicker_during_an_idle_buffer_correction_keeps_the_channel_loaded(void)
{
    // The idle control moves the gear only while the buffer is outside 30-70%. Moves back and forth
    // that add up to less than 10 mm of retract since the key left 'both' do not count.
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 30.0, 200u));   // 6 mm back
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, -30.0, 200u));  // 6 mm forward
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 30.0, 300u));   // 9 mm back
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 0.0, 2000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 0.0, 1u));
    TEST_ASSERT_EQUAL_UINT8(1u, loaded);
    TEST_ASSERT_FALSE(stage2_armed());
}

void test_feeding_away_from_both_is_not_a_retract(void)
{
    // e.g. filament pushed forward by hand or by the idle control: the gear position falls
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, -60.0, 5000u)); // 300 mm forward
    TEST_ASSERT_EQUAL_UINT8(1u, loaded);
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 0.0, 1u));
    TEST_ASSERT_FALSE(stage2_armed());
}

void test_a_retract_is_counted_from_the_last_pass_at_both(void)
{
    // Two 6 mm retracts separated by one pass at 'both' are two excursions, neither reaching 10 mm.
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 30.0, 200u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 0.0, 1u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 30.0, 200u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 0.0, 1000u));
    TEST_ASSERT_EQUAL_UINT8(1u, loaded);
}

void test_printer_moves_do_not_count_as_a_retract(void)
{
    // The key misread during the printer's unload: 95 mm retracted by the pull back, then idle.
    set_printer(true, BEFORE_PULL_BACK);
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 20.0, 1000u));
    set_printer(true, PULL_BACK);
    unloading = true; // filament_pulling_back
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 60.0, 1583u));
    set_printer(true, IDLE); // the pull back went on while the printer already reads idle
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 60.0, 500u));
    unloading = false;
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 0.0, 1u));
    TEST_ASSERT_EQUAL_UINT8(1u, loaded);
    TEST_ASSERT_FALSE(stage2_armed());

    // ... and while loading or printing
    const _filament_motion ms[] = {SEND_OUT, BEFORE_ON_USE, ON_USE, STOP_ON_USE};
    for (unsigned i = 0; i < sizeof(ms) / sizeof(ms[0]); i++)
    {
        set_printer(true, ms[i]);
        TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 60.0, 1000u));
        TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 60.0, 10u));
        TEST_ASSERT_EQUAL_UINT8(1u, loaded);
    }
}

void test_a_retract_the_printer_started_must_go_on_10mm_in_idle(void)
{
    // The excursion only starts once the printer has let go of the channel.
    set_printer(true, PULL_BACK);
    unloading = true;
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 60.0, 500u));
    unloading = false;
    set_printer(true, IDLE);
    mark = now;
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 0.0, 1000u));
    TEST_ASSERT_EQUAL_UINT8(1u, loaded);
    // 10 mm at 12 mm/s in idle: 834 ms after the gear started (833.6 ms per 10.003 mm)
    TEST_ASSERT_INT32_WITHIN(2, 834, hold(KS_EXT, 12.0, 2000u) + 1);
    TEST_ASSERT_EQUAL_UINT8(0u, loaded);
}

// ---- Real events: re-arm ----

void test_retract_out_of_both_in_idle_arms_stage2(void)
{
    // e.g. the auto-unload (850 PWM) pulling out of 'both'. 10 mm at 60 mm/s: 167 ms.
    const int32_t at = hold(KS_EXT, 60.0, 1000u);
    TEST_ASSERT_INT32_WITHIN(1, 167, at);
    TEST_ASSERT_EQUAL_INT(1, events[DM_REARM_RETRACTED]);
    TEST_ASSERT_EQUAL_UINT8(0u, loaded);
    // pushed back in by hand: Stage-2 runs, as before
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 0.0, 1u));
    TEST_ASSERT_TRUE(stage2_armed());
}

void test_auto_unload_tail_rearms_even_at_12mm_s(void)
{
    // The auto-unload pulls for 1.5 s after the key leaves 'both' (AUTO_UNLOAD_EMPTY_MS).
    const int32_t at = hold(KS_EXT, 12.0, 1500u);
    TEST_ASSERT_TRUE(at >= 0);
    TEST_ASSERT_TRUE(at < 1500);
    TEST_ASSERT_EQUAL_UINT8(0u, loaded);
}

void test_retract_threshold_is_10mm(void)
{
    TEST_ASSERT_TRUE((double)DM_REARM_RETRACT_CNT * (double)ML_MM_PER_CNT >= 10.0);
    TEST_ASSERT_TRUE((double)(DM_REARM_RETRACT_CNT - 1u) * (double)ML_MM_PER_CNT < 10.0);

    // Step the count directly: 1738 counts (9.998 mm) never re-arm, 1739 do.
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 0.0, 1u));
    cnt0 += DM_REARM_RETRACT_CNT - 1u;
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 0.0, 5000u));
    TEST_ASSERT_EQUAL_UINT8(1u, loaded);
    cnt0 += 1u;
    TEST_ASSERT_EQUAL_INT32(0, hold(KS_EXT, 0.0, 10u));
    TEST_ASSERT_EQUAL_UINT8(0u, loaded);
}

void test_a_fast_retract_still_needs_100ms_away_from_both(void)
{
    mark = now;
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 0.0, 1u)); // excursion starts (pass 0)
    cnt0 += 4u * DM_REARM_RETRACT_CNT;                   // 40 mm at once
    TEST_ASSERT_EQUAL_INT32(99, hold(KS_EXT, 0.0, 200u)); // pass 100
    TEST_ASSERT_EQUAL_INT32(100, last_event_ms);
    TEST_ASSERT_EQUAL_UINT8(0u, loaded);
}

void test_retract_across_the_count_wrap(void)
{
    gear_reset(0xFFFFFF00u); // retracting counts up through 0
    TEST_ASSERT_INT32_WITHIN(1, 167, hold(KS_EXT, 60.0, 1000u));
    TEST_ASSERT_EQUAL_UINT8(0u, loaded);

    // feeding counts down through 0: no re-arm
    loaded = 1u;
    gear_reset(0x00000100u);
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, -60.0, 1000u));
    TEST_ASSERT_EQUAL_UINT8(1u, loaded);
}

void test_filament_removed_past_both_switches_rearms(void)
{
    // pulled out: 'both' -> 'external only' -> 'none', one pass at 'none' is enough (as before)
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 0.0, 50u));
    TEST_ASSERT_EQUAL_INT32(0, hold(KS_NONE, 0.0, 1u));
    TEST_ASSERT_EQUAL_INT(1, events[DM_REARM_EMPTY]);
    TEST_ASSERT_EQUAL_UINT8(0u, loaded);
    // re-inserted: Stage-1 at 'external only' (not loaded), then Stage-2 at 'both'
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, -60.0, 500u));
    TEST_ASSERT_EQUAL_UINT8(0u, loaded);
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 0.0, 1u));
    TEST_ASSERT_TRUE(stage2_armed());
}

void test_none_unloads_in_any_printer_state(void)
{
    const _filament_motion ms[] = {IDLE, SEND_OUT, ON_USE, BEFORE_PULL_BACK, PULL_BACK, BEFORE_ON_USE, STOP_ON_USE};
    for (unsigned i = 0; i < sizeof(ms) / sizeof(ms[0]); i++)
    {
        loaded = 1u;
        set_printer(true, ms[i]);
        TEST_ASSERT_EQUAL(DM_REARM_EMPTY, pass(KS_NONE, 0.0));
        TEST_ASSERT_EQUAL_UINT8(0u, loaded);
        TEST_ASSERT_EQUAL_UINT64(0u, away_t0);
    }
}

// ---- Insertion and boot: as before ----

void test_first_insertion_autoloads_as_before(void)
{
    // boot with no filament: dm_loaded = 0
    loaded = 0u;
    TEST_ASSERT_EQUAL_INT32(0, hold(KS_NONE, 0.0, 1000u));
    TEST_ASSERT_EQUAL_UINT8(0u, loaded);
    // external switch touched, Stage-1 pushes to the inner switch
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, -60.0, 800u));
    TEST_ASSERT_EQUAL_UINT8(0u, loaded);
    // 'both': Stage-2 starts and keeps running on every pass at 'both'
    for (int i = 0; i < 2000; i++)
    {
        TEST_ASSERT_EQUAL(DM_REARM_NONE, pass(KS_BOTH, -60.0));
        TEST_ASSERT_TRUE(stage2_armed());
    }
    // Stage-2 done (run() sets dm_loaded): the key flickering afterwards changes nothing
    loaded = 1u;
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 0.0, 500u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 0.0, 1u));
    TEST_ASSERT_FALSE(stage2_armed());
}

void test_insertion_straight_to_both_autoloads_as_before(void)
{
    loaded = 0u;
    TEST_ASSERT_EQUAL_INT32(0, hold(KS_NONE, 0.0, 10u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, -200.0, 20u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 0.0, 1u));
    TEST_ASSERT_TRUE(stage2_armed());
}

void test_boot_with_external_only_then_pushed_in_autoloads_as_before(void)
{
    // Motion_control_init: dm_loaded = (ks == both), so 0 here
    loaded = 0u;
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 0.0, 10000u));
    TEST_ASSERT_EQUAL_UINT8(0u, loaded);
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, -30.0, 300u)); // pushed in by hand
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 0.0, 1u));
    TEST_ASSERT_TRUE(stage2_armed());
}

void test_boot_with_external_only_then_printer_load_is_loaded_after_the_print(void)
{
    loaded = 0u;
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 0.0, 1000u));

    // The printer loads the channel: send_out pushes past the inner switch. Not loaded yet.
    set_printer(true, SEND_OUT);
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, -60.0, 300u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, -60.0, 10000u));
    TEST_ASSERT_EQUAL_UINT8(0u, loaded);

    // The load ends (before_on_use): loaded
    set_printer(true, BEFORE_ON_USE);
    TEST_ASSERT_EQUAL_INT32(0, hold(KS_BOTH, 0.0, 100u));
    TEST_ASSERT_EQUAL_INT(1, events[DM_REARM_LOADED]);
    TEST_ASSERT_EQUAL_UINT8(1u, loaded);

    // print, unload (the key stays at 'both'), idle: no Stage-2 push
    set_printer(true, ON_USE);
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, -3.0, 60000u));
    set_printer(true, BEFORE_PULL_BACK);
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 20.0, 1000u));
    set_printer(true, PULL_BACK);
    unloading = true;
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 60.0, 1583u));
    unloading = false;
    set_printer(true, IDLE);
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 0.0, 1000u));
    TEST_ASSERT_FALSE(stage2_armed());
    TEST_ASSERT_EQUAL_UINT8(1u, loaded);
}

void test_printer_load_marks_loaded_only_at_both(void)
{
    const _filament_motion ms[] = {BEFORE_ON_USE, ON_USE, STOP_ON_USE};
    for (unsigned i = 0; i < sizeof(ms) / sizeof(ms[0]); i++)
    {
        loaded = 0u;
        set_printer(true, ms[i]);
        TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 0.0, 1000u));
        TEST_ASSERT_EQUAL_INT32(-1, hold(KS_OTHER, 0.0, 1000u));
        TEST_ASSERT_EQUAL_UINT8(0u, loaded);
        TEST_ASSERT_EQUAL(DM_REARM_LOADED, pass(KS_BOTH, 0.0));
        TEST_ASSERT_EQUAL_UINT8(1u, loaded);
    }

    // not while the printer only loads, unloads, or serves another channel
    const _filament_motion other[] = {IDLE, SEND_OUT, BEFORE_PULL_BACK, PULL_BACK};
    for (unsigned i = 0; i < sizeof(other) / sizeof(other[0]); i++)
    {
        loaded = 0u;
        set_printer(true, other[i]);
        TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 0.0, 1000u));
        TEST_ASSERT_EQUAL_UINT8(0u, loaded);
    }
    loaded = 0u;
    set_printer(false, ON_USE);
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 0.0, 1000u));
    TEST_ASSERT_EQUAL_UINT8(0u, loaded);
}

void test_boot_with_both_stays_loaded(void)
{
    // Motion_control_init: dm_loaded = 1 with the key at 'both'
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 0.0, 1000u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_EXT, 0.0, 300u));
    TEST_ASSERT_EQUAL_INT32(-1, hold(KS_BOTH, 0.0, 1000u));
    TEST_ASSERT_EQUAL_UINT8(1u, loaded);
    TEST_ASSERT_EQUAL_INT(0, events[DM_REARM_RETRACTED] + events[DM_REARM_EMPTY] + events[DM_REARM_LOADED]);
}

void test_host_state_mapping(void)
{
    TEST_ASSERT_EQUAL(DM_HOST_IDLE, dm_host_from_motion(true, IDLE, false));
    TEST_ASSERT_EQUAL(DM_HOST_MOVING, dm_host_from_motion(true, SEND_OUT, false));
    TEST_ASSERT_EQUAL(DM_HOST_LOADED, dm_host_from_motion(true, ON_USE, false));
    TEST_ASSERT_EQUAL(DM_HOST_MOVING, dm_host_from_motion(true, BEFORE_PULL_BACK, false));
    TEST_ASSERT_EQUAL(DM_HOST_MOVING, dm_host_from_motion(true, PULL_BACK, false));
    TEST_ASSERT_EQUAL(DM_HOST_LOADED, dm_host_from_motion(true, BEFORE_ON_USE, false));
    TEST_ASSERT_EQUAL(DM_HOST_LOADED, dm_host_from_motion(true, STOP_ON_USE, false));

    const _filament_motion ms[] = {IDLE, SEND_OUT, ON_USE, BEFORE_PULL_BACK, PULL_BACK, BEFORE_ON_USE, STOP_ON_USE};
    for (unsigned i = 0; i < sizeof(ms) / sizeof(ms[0]); i++)
    {
        TEST_ASSERT_EQUAL(DM_HOST_IDLE, dm_host_from_motion(false, ms[i], false));
        TEST_ASSERT_EQUAL(DM_HOST_MOVING, dm_host_from_motion(true, ms[i], true));
        TEST_ASSERT_EQUAL(DM_HOST_MOVING, dm_host_from_motion(false, ms[i], true));
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_flicker_to_external_only_keeps_the_channel_loaded);
    RUN_TEST(test_flicker_to_the_other_key_state_keeps_the_channel_loaded);
    RUN_TEST(test_chatter_around_the_both_threshold_keeps_the_channel_loaded);
    RUN_TEST(test_flicker_during_an_idle_buffer_correction_keeps_the_channel_loaded);
    RUN_TEST(test_feeding_away_from_both_is_not_a_retract);
    RUN_TEST(test_a_retract_is_counted_from_the_last_pass_at_both);
    RUN_TEST(test_printer_moves_do_not_count_as_a_retract);
    RUN_TEST(test_a_retract_the_printer_started_must_go_on_10mm_in_idle);
    RUN_TEST(test_retract_out_of_both_in_idle_arms_stage2);
    RUN_TEST(test_auto_unload_tail_rearms_even_at_12mm_s);
    RUN_TEST(test_retract_threshold_is_10mm);
    RUN_TEST(test_a_fast_retract_still_needs_100ms_away_from_both);
    RUN_TEST(test_retract_across_the_count_wrap);
    RUN_TEST(test_filament_removed_past_both_switches_rearms);
    RUN_TEST(test_none_unloads_in_any_printer_state);
    RUN_TEST(test_first_insertion_autoloads_as_before);
    RUN_TEST(test_insertion_straight_to_both_autoloads_as_before);
    RUN_TEST(test_boot_with_external_only_then_pushed_in_autoloads_as_before);
    RUN_TEST(test_boot_with_external_only_then_printer_load_is_loaded_after_the_print);
    RUN_TEST(test_printer_load_marks_loaded_only_at_both);
    RUN_TEST(test_boot_with_both_stays_loaded);
    RUN_TEST(test_host_state_mapping);
    return UNITY_END();
}
