// Host tests for src/auto_unload.h, driven through auto_unload_pass() in 1 ms main-loop passes the
// way motor_motion_run calls it for a channel whose AS5600 reads are good. Online, the auto-unload
// (buffer lifted to 80% and let back to neutral within 1 s, in the idle control: 850 PWM retract
// until the buffer drops below 35%, 1.5 s after the key left 'both', or 15 s) and the manual empty
// pull (no filament at the switches, buffer above 80%: 700 PWM) must behave exactly as the inline
// code before them. Offline (error != 0: before the first heartbeat, after a lost link), neither may
// drive: a running auto-unload ends with its state cleared and needs a new lift once the link is
// back, and the manual pull runs again only while the link is up.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "auto_unload.h"

#define KS_NONE 0u
#define KS_BOTH 1u
#define KS_EXT  2u

static uint64_t now;
static auto_unload_t st;
static au_in_t in;

void setUp(void)
{
    now = 20000u;
    memset(&st, 0, sizeof(st));
    in.online = true;
    in.inserted = true;
    in.idle_ctrl = true;
    in.pct = 50.0f;
    in.ks = KS_BOTH;
    in.now_ms = now;
}

void tearDown(void) {}

static au_drive_t pass(void)
{
    in.now_ms = now;
    const au_drive_t d = auto_unload_pass(&st, &in);
    now++;
    return d;
}

// `ms` passes at buffer level pct; returns the drive of the last one.
static au_drive_t hold(float pct, uint32_t ms)
{
    au_drive_t d = AU_DRIVE_NONE;
    in.pct = pct;
    for (uint32_t i = 0; i < ms; i++) d = pass();
    return d;
}

// Passes at pct until the drive changes from `from`, at most max_ms. Returns the ms, or -1.
static int32_t until_not(au_drive_t from, float pct, uint32_t max_ms)
{
    in.pct = pct;
    for (uint32_t i = 0; i < max_ms; i++)
        if (pass() != from) return (int32_t)i;
    return -1;
}

// The gesture: buffer lifted to 90% for 300 ms, then let back to 50%. The auto-unload starts on the
// first pass back in the neutral band.
static void lift_and_release(void)
{
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(90.0f, 300u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(70.0f, 50u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_UNLOAD, hold(50.0f, 1u));
}

// ---- Online: as before ----

static void test_lift_and_release_starts_the_auto_unload(void)
{
    lift_and_release();
    TEST_ASSERT_EQUAL_UINT8(1u, st.active);
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_UNLOAD, hold(50.0f, 5000u));
}

static void test_no_auto_unload_without_the_gesture(void)
{
    // Back to neutral more than 1 s after the buffer was last at 80% or above (held up, it re-arms
    // every second), or never lifted, or not in the idle control.
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(90.0f, 300u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(70.0f, 1100u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(50.0f, 1000u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(60.0f, 10000u));

    in.idle_ctrl = false;
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(90.0f, 300u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(50.0f, 1000u));
}

static void test_the_release_may_come_up_to_1000ms_after_the_lift(void)
{
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(90.0f, 1u)); // armed on this pass
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(70.0f, 999u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_UNLOAD, hold(50.0f, 1u)); // 1000 ms later

    setUp();
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(90.0f, 1u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(70.0f, 1000u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(50.0f, 1u)); // 1001 ms later
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(50.0f, 1000u));
}

static void test_auto_unload_ends_and_needs_a_new_lift(void)
{
    // Buffer below 35%.
    lift_and_release();
    TEST_ASSERT_EQUAL_INT32(0, until_not(AU_DRIVE_UNLOAD, 34.9f, 10u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(50.0f, 3000u));

    // 1.5 s after the key left 'both'.
    setUp();
    lift_and_release();
    in.ks = KS_EXT;
    TEST_ASSERT_EQUAL_INT32((int32_t)AUTO_UNLOAD_EMPTY_MS, until_not(AU_DRIVE_UNLOAD, 50.0f, 5000u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(50.0f, 3000u));

    // 15 s at most.
    setUp();
    lift_and_release();
    TEST_ASSERT_EQUAL_INT32((int32_t)AUTO_UNLOAD_MAX_MS - 1, until_not(AU_DRIVE_UNLOAD, 50.0f, 20000u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(50.0f, 3000u));

    // A new lift starts it again.
    lift_and_release();
}

static void test_leaving_the_idle_control_does_not_stop_a_running_auto_unload(void)
{
    // As before: once running, only its own ends stop it (the printer's command changes the motion,
    // not the auto-unload).
    lift_and_release();
    in.idle_ctrl = false;
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_UNLOAD, hold(50.0f, 1000u));
}

static void test_manual_empty_pull_while_the_buffer_is_held_up(void)
{
    in.ks = KS_NONE;
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(80.0f, 100u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_EMPTY_PULL, hold(80.1f, 1u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_EMPTY_PULL, hold(95.0f, 5000u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(79.0f, 1u));

    // In any motor state, but not on a channel that is not wired, nor with filament at the switch.
    in.idle_ctrl = false;
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_EMPTY_PULL, hold(95.0f, 1u));
    in.inserted = false;
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(95.0f, 1u));
    in.inserted = true;
    in.ks = KS_EXT;
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(95.0f, 1u));
}

// ---- Offline ----

static void test_offline_nothing_starts(void)
{
    // Before the first heartbeat (or with the link lost): the gesture, and the buffer held up with
    // no filament at the switches, drive nothing.
    in.online = false;
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(90.0f, 300u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(50.0f, 1000u));
    TEST_ASSERT_EQUAL_UINT8(0u, st.active);

    in.ks = KS_NONE;
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(95.0f, 60000u));
}

static void test_link_lost_ends_a_running_auto_unload(void)
{
    // It used to go on at 850 PWM for up to 15 s.
    lift_and_release();
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_UNLOAD, hold(50.0f, 2000u));
    in.online = false;
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(50.0f, 1u));
    TEST_ASSERT_EQUAL_UINT8(0u, st.active);
    TEST_ASSERT_EQUAL_UINT8(0u, st.arm);
    TEST_ASSERT_EQUAL_UINT8(0u, st.blocked);
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(50.0f, 20000u));

    // Back online it does not resume by itself; a new lift starts it.
    in.online = true;
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(50.0f, 20000u));
    lift_and_release();
}

static void test_short_link_loss_during_an_auto_unload(void)
{
    // One pass offline is enough to end it (the bus link latches lost after its 1 s timeout).
    lift_and_release();
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_UNLOAD, hold(50.0f, 100u));
    in.online = false;
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, pass());
    in.online = true;
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(50.0f, 1000u));
}

static void test_link_lost_stops_a_manual_empty_pull(void)
{
    in.ks = KS_NONE;
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_EMPTY_PULL, hold(95.0f, 500u));
    in.online = false;
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(95.0f, 1u));
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_NONE, hold(95.0f, 10000u));
    // With the link back, the buffer still held up pulls again.
    in.online = true;
    TEST_ASSERT_EQUAL_INT(AU_DRIVE_EMPTY_PULL, hold(95.0f, 1u));
}

// ---- Online: the same decisions as the inline code before auto_unload.h ----

// ---- adapted from Motion_control.cpp: motor_motion_run's auto-unload pass before auto_unload.h ----
// The inline code this header replaced (arrays for one channel, constants written out), kept as the
// reference for the comparison below; it is not firmware code any more, so it is not checked.
static uint8_t  ref_arm, ref_active, ref_blocked;
static uint64_t ref_arm_t0_ms, ref_active_t0_ms, ref_empty_t0_ms;

static au_drive_t ref_pass(bool inserted, bool idle_ctrl, float pct_f, uint8_t ks_now, uint64_t time_now)
{
    if (!inserted || (!ref_active && !idle_ctrl))
    {
        ref_arm = 0u;
        ref_active = 0u;
        ref_blocked = 0u;
        ref_arm_t0_ms = 0ull;
        ref_active_t0_ms = 0ull;
        ref_empty_t0_ms = 0ull;
    }
    else
    {
        const float pct = pct_f;
        const uint8_t ks = ks_now;

        if (pct >= 80.0f)
        {
            ref_blocked = 0u;
            if (!ref_arm && !ref_active)
            {
                ref_arm = 1u;
                ref_arm_t0_ms = time_now;
            }
        }

        if (ref_arm && !ref_active)
        {
            const uint64_t dt = time_now - ref_arm_t0_ms;
            if ((pct > 45.0f) && (pct < 55.0f))
            {
                if (!ref_blocked && dt <= 1000ull)
                {
                    ref_active = 1u;
                    ref_active_t0_ms = time_now;
                    ref_empty_t0_ms = 0ull;
                    ref_blocked = 1u;
                }
                ref_arm = 0u;
                ref_arm_t0_ms = 0ull;
            }
            else if (dt > 1000ull)
            {
                ref_arm = 0u;
                ref_arm_t0_ms = 0ull;
            }
        }

        if (ref_active)
        {
            if (pct < 35.0f)
            {
                ref_active = 0u;
                ref_active_t0_ms = 0ull;
                ref_empty_t0_ms = 0ull;
                ref_blocked = 1u;
            }
            else if (ks == 1u)
            {
                ref_empty_t0_ms = 0ull;
                if ((time_now - ref_active_t0_ms) >= 15000ull)
                {
                    ref_active = 0u;
                    ref_active_t0_ms = 0ull;
                    ref_empty_t0_ms = 0ull;
                    ref_blocked = 1u;
                }
            }
            else
            {
                if (ref_empty_t0_ms == 0ull)
                {
                    ref_empty_t0_ms = time_now;
                }
                else if ((time_now - ref_empty_t0_ms) >= 1500ull)
                {
                    ref_active = 0u;
                    ref_active_t0_ms = 0ull;
                    ref_empty_t0_ms = 0ull;
                    ref_blocked = 1u;
                }
            }
        }
    }

    const bool manual_empty_pull = inserted && (ks_now == 0u) && (pct_f > 80.0f) && (ref_active == 0u);
    if (ref_active) return AU_DRIVE_UNLOAD;
    if (manual_empty_pull) return AU_DRIVE_EMPTY_PULL;
    return AU_DRIVE_NONE;
}

static uint32_t rng = 12345u;
static uint32_t rnd(uint32_t n)
{
    rng = rng * 1664525u + 1013904223u;
    return (rng >> 8) % n;
}

static void test_online_decisions_match_the_code_before(void)
{
    // Two million passes of a buffer that wanders, jumps and is lifted and released at random, the
    // key and the motor state changing at random: every drive and every state field must match.
    ref_arm = ref_active = ref_blocked = 0u;
    ref_arm_t0_ms = ref_active_t0_ms = ref_empty_t0_ms = 0u;
    float pct = 50.0f;
    uint32_t unloads = 0u, pulls = 0u;

    for (uint32_t i = 0; i < 2000000u; i++)
    {
        const uint32_t r = rnd(10000u);
        if (r < 30u) pct = 90.0f;                       // lift
        else if (r < 60u) pct = 50.0f;                  // release to neutral
        else if (r < 70u) pct = (float)rnd(101u);       // jump
        else pct += ((float)rnd(201u) - 100.0f) * 0.01f; // wander
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 100.0f) pct = 100.0f;
        if (rnd(3000u) == 0u) in.ks = (uint8_t)rnd(4u);
        if (rnd(5000u) == 0u) in.idle_ctrl = !in.idle_ctrl;
        if (rnd(50000u) == 0u) in.inserted = !in.inserted;
        if (rnd(20u) == 0u) now += rnd(300u);            // uneven passes

        in.pct = pct;
        const au_drive_t want = ref_pass(in.inserted, in.idle_ctrl, pct, in.ks, now);
        const au_drive_t got = pass();
        TEST_ASSERT_EQUAL_INT(want, got);
        TEST_ASSERT_EQUAL_UINT8(ref_arm, st.arm);
        TEST_ASSERT_EQUAL_UINT8(ref_active, st.active);
        TEST_ASSERT_EQUAL_UINT8(ref_blocked, st.blocked);
        TEST_ASSERT_TRUE(ref_arm_t0_ms == st.arm_t0_ms);
        TEST_ASSERT_TRUE(ref_active_t0_ms == st.active_t0_ms);
        TEST_ASSERT_TRUE(ref_empty_t0_ms == st.empty_t0_ms);
        if (got == AU_DRIVE_UNLOAD) unloads++;
        if (got == AU_DRIVE_EMPTY_PULL) pulls++;
    }
    // The walk exercised both.
    TEST_ASSERT_TRUE(unloads > 10000u);
    TEST_ASSERT_TRUE(pulls > 10000u);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_lift_and_release_starts_the_auto_unload);
    RUN_TEST(test_no_auto_unload_without_the_gesture);
    RUN_TEST(test_the_release_may_come_up_to_1000ms_after_the_lift);
    RUN_TEST(test_auto_unload_ends_and_needs_a_new_lift);
    RUN_TEST(test_leaving_the_idle_control_does_not_stop_a_running_auto_unload);
    RUN_TEST(test_manual_empty_pull_while_the_buffer_is_held_up);
    RUN_TEST(test_offline_nothing_starts);
    RUN_TEST(test_link_lost_ends_a_running_auto_unload);
    RUN_TEST(test_short_link_loss_during_an_auto_unload);
    RUN_TEST(test_link_lost_stops_a_manual_empty_pull);
    RUN_TEST(test_online_decisions_match_the_code_before);
    return UNITY_END();
}
