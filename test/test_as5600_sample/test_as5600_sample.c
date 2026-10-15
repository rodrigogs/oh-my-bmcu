// Host tests for src/as5600_sample.h: a failed AS5600 read (out of the driver's valid mask, which
// since V11 leaves raw_angle at the last valid angle; V10.5's driver set it to 0), an answered read
// above 12 bits or an impossible jump must never be counted as filament movement, and the next good
// read must be diffed against the last accepted one, over the time since it. (The motor direction
// test is V11's MOTOR_get_dir, in Motion_control.cpp, since the merge; the fork's probe is gone.)

#include <stdbool.h>
#include <stdint.h>
#include <unity.h>

#include "as5600_sample.h"
#include "motion_limits.h" // ML_MM_PER_CNT, the AS5600 scale Motion_control.cpp asserts

// SysTick at 18 MHz, as on the CH32V203 (time_hw_tpus / time_hw_tpms).
#define TPUS 18u
#define TPMS (1000u * TPUS)

// ===== distance/speed tracker =====

// ---- adapted from Motion_control.cpp: AS5600_distance_updata's use of as5600_track_sample(), in counts ----
// ---- anchor: AS5600_distance_updata ----
// One channel as AS5600_distance_updata runs it: counts moved (what as5600_odometer_count adds up),
// sum of |moves|, speed in counts per ms (held on a skip, 0 on a new baseline). t: the read's
// SysTick (sample_ticks, taken after the I2C read). ok is the channel's bit in updata_angle()'s
// valid mask (and the magnet not offline); raw is MC_AS5600.raw_angle[i].
typedef struct
{
    as5600_track_t s;
    uint32_t t;
    int32_t  total;
    int32_t  abs_total;
    float    speed;
    int      moves;
    int      jumps;
} chan_t;

static chan_t ch;

static void chan_init(uint32_t t0)
{
    as5600_track_reset(&ch.s);
    ch.t = t0;
    ch.total = 0;
    ch.abs_total = 0;
    ch.speed = 0.0f;
    ch.moves = 0;
    ch.jumps = 0;
}

// One poll, `ms` after the previous one.
static uint8_t poll_after(uint32_t ms, bool good, bool ok, uint16_t raw)
{
    int32_t diff = 12345;
    uint32_t dt = 12345u;
    ch.t += ms * TPMS;
    const uint8_t r = as5600_track_sample(&ch.s, good, ok, raw, ch.t, TPUS, &diff, &dt);
    switch (r)
    {
    case AS5600_TRACK_MOVE:
        ch.total += diff;
        ch.abs_total += (diff < 0) ? -diff : diff;
        ch.speed = (float)diff * (float)TPMS / (float)dt;
        ch.moves++;
        break;
    case AS5600_TRACK_SKIP:
    case AS5600_TRACK_JUMP:
        TEST_ASSERT_EQUAL_INT32(0, diff);
        if (r == AS5600_TRACK_JUMP) ch.jumps++;
        break;
    default:
        TEST_ASSERT_EQUAL_INT32(0, diff);
        ch.speed = 0.0f;
        break;
    }
    return r;
}

static uint8_t poll(bool ok, uint16_t raw) { return poll_after(1u, true, ok, raw); }

// ---- adapted from many_soft_AS5600.cpp: updata_angle() for one channel (V11) ----
// ---- anchor: updata_angle ----
// A read is valid (its bit in the returned mask) when the sensor answered and the data word is
// within 12 bits; only then does raw_angle take it. A failed read leaves raw_angle at the last valid
// angle. The magnet is taken as present.
static uint16_t raw_angle; // MC_AS5600.raw_angle[i]

static uint8_t driver_poll(bool answered, uint16_t data)
{
    const bool valid = answered && (data <= 4095u);
    if (valid) raw_angle = data;
    return poll(valid, raw_angle);
}

void setUp(void) { chan_init(0x10000000u); }
void tearDown(void) {}

static void test_angle_diff_takes_the_short_way_round(void)
{
    TEST_ASSERT_EQUAL_INT32(11, as5600_angle_diff(5, 4090));
    TEST_ASSERT_EQUAL_INT32(-11, as5600_angle_diff(4090, 5));
    TEST_ASSERT_EQUAL_INT32(2048, as5600_angle_diff(2148, 100));
    TEST_ASSERT_EQUAL_INT32(-2047, as5600_angle_diff(2149, 100));
    TEST_ASSERT_EQUAL_INT32(0, as5600_angle_diff(777, 777));
}

static void test_max_step_is_150_counts_per_ms_of_elapsed_time(void)
{
    TEST_ASSERT_EQUAL_INT32(150, as5600_max_step(0u));      // floor: one 1 ms poll period
    TEST_ASSERT_EQUAL_INT32(150, as5600_max_step(1000u));
    TEST_ASSERT_EQUAL_INT32(225, as5600_max_step(1500u));
    TEST_ASSERT_EQUAL_INT32(300, as5600_max_step(2000u));
    TEST_ASSERT_EQUAL_INT32(1500, as5600_max_step(10000u));
    TEST_ASSERT_EQUAL_INT32(2048, as5600_max_step(13652u)); // half a turn: all the wrap resolves
    TEST_ASSERT_EQUAL_INT32(2048, as5600_max_step(13653u));
    TEST_ASSERT_EQUAL_INT32(2048, as5600_max_step(0xFFFFFFFFu));
}

static void test_raw_angle_is_12_bits(void)
{
    TEST_ASSERT_TRUE(as5600_raw_ok(0));
    TEST_ASSERT_TRUE(as5600_raw_ok(4095));
    TEST_ASSERT_FALSE(as5600_raw_ok(4096));
    TEST_ASSERT_FALSE(as5600_raw_ok(0xF000));
    TEST_ASSERT_FALSE(as5600_raw_ok(0xFFFF));
}

static void test_first_good_read_is_the_baseline(void)
{
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_REBASE, poll(true, 1234));
    TEST_ASSERT_EQUAL_INT32(0, ch.total);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 1240));
    TEST_ASSERT_EQUAL_INT32(6, ch.total);
}

// The defect: fails 1 and 2 pass the health gate and used to be taken as angle 0.
static void test_failed_read_is_skipped_and_next_read_diffs_against_last_good(void)
{
    poll(true, 1000);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 1010));
    TEST_ASSERT_EQUAL_FLOAT(10.0f, ch.speed);

    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_SKIP, poll(false, 0));
    TEST_ASSERT_EQUAL_INT32(10, ch.total);       // not 10 + (0 - 1010)
    TEST_ASSERT_EQUAL_FLOAT(10.0f, ch.speed);    // held, no +-11.8 m/s spike

    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 1030));
    TEST_ASSERT_EQUAL_INT32(30, ch.total);
    TEST_ASSERT_EQUAL_INT32(30, ch.abs_total);   // no fake legs (old code: 10 + 1010 + 1030)
    TEST_ASSERT_EQUAL_FLOAT(10.0f, ch.speed);    // 20 counts over the 2 ms since the last good read
}

static void test_two_failed_reads_in_a_row_are_both_skipped(void)
{
    poll(true, 2000);
    poll(true, 1990);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_SKIP, poll(false, 0));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_SKIP, poll(false, 0));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 1960));
    TEST_ASSERT_EQUAL_INT32(-40, ch.total);
    TEST_ASSERT_EQUAL_INT32(40, ch.abs_total);
    TEST_ASSERT_EQUAL_FLOAT(-10.0f, ch.speed);
}

// Through V11's driver: a NACK or a data word above 12 bits is out of the valid mask and leaves
// raw_angle at the last valid angle, which is not taken as a new one; the next valid read is diffed
// against the last accepted read, over the time since it.
static void test_failed_reads_through_the_v11_driver_are_skipped(void)
{
    raw_angle = 0u;
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_SKIP, driver_poll(false, 0u)); // no read yet: no baseline
    TEST_ASSERT_EQUAL_UINT8(0u, ch.s.valid);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_REBASE, driver_poll(true, 1000u));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, driver_poll(true, 1010u));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_SKIP, driver_poll(false, 0u));
    TEST_ASSERT_EQUAL_UINT16(1010u, raw_angle);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_SKIP, driver_poll(true, 0xFFFFu));
    TEST_ASSERT_EQUAL_UINT16(1010u, raw_angle);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, driver_poll(true, 1040u));
    TEST_ASSERT_EQUAL_INT32(40, ch.total);
    TEST_ASSERT_EQUAL_INT32(40, ch.abs_total);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, ch.speed); // 30 counts over the 3 ms since the last good read
}

static void test_failed_read_is_never_a_baseline(void)
{
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_SKIP, poll(false, 0));
    TEST_ASSERT_EQUAL_UINT8(0u, ch.s.valid);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_REBASE, poll(true, 3000));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 3010));
    TEST_ASSERT_EQUAL_INT32(10, ch.total);
    TEST_ASSERT_EQUAL_INT32(10, ch.abs_total);
}

// After the gate trips (3 fails) the baseline is dropped, and the first read after recovery only
// sets a new one: movement while the sensor was out is not guessed.
static void test_gate_off_drops_the_baseline(void)
{
    poll(true, 100);
    poll(true, 110);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_OFF, poll_after(1u, false, false, 0));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ch.speed);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_REBASE, poll_after(1u, true, true, 1500));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 1505));
    TEST_ASSERT_EQUAL_INT32(15, ch.total);
}

// A read that was answered but is corrupted: rejected, and the next read counts from the last
// good one.
static void test_impossible_jump_is_rejected(void)
{
    poll(true, 1000);
    poll(true, 1010);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_JUMP, poll(true, 3000));
    TEST_ASSERT_EQUAL_FLOAT(10.0f, ch.speed);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 1030));
    TEST_ASSERT_EQUAL_INT32(30, ch.total);
    TEST_ASSERT_EQUAL_INT32(30, ch.abs_total);
}

// An answered read above 12 bits is a failed read, like a NACK, even when the move it would give is
// within the jump limit (0x1000 is 4096 = angle 0 after the wrap, 10 counts from 4086).
static void test_garbage_above_12_bits_is_skipped(void)
{
    poll(true, 4086);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_SKIP, poll(true, 0x1000));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_SKIP, poll(true, 0xFFFF));
    TEST_ASSERT_EQUAL_INT32(4086, ch.s.angle);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 4095));
    TEST_ASSERT_EQUAL_INT32(9, ch.total);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, ch.speed); // 9 counts over the 3 ms since the last good read
}

// Nor is it ever a baseline, for example the first read after the gate recovers (it used to be
// taken, which then cost 3 ms of real motion to the jump rebase).
static void test_garbage_above_12_bits_is_never_a_baseline(void)
{
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_SKIP, poll(true, 0xFFFF));
    TEST_ASSERT_EQUAL_UINT8(0u, ch.s.valid);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_REBASE, poll(true, 1000));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 1010));
    TEST_ASSERT_EQUAL_INT32(10, ch.total);
}

static void test_limit_is_exactly_150_counts_per_ms(void)
{
    poll(true, 1000);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 1150));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 1000));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_JUMP, poll(true, 1151));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_JUMP, poll(true, 699));  // 301 back over 2 ms
    TEST_ASSERT_EQUAL_INT32(0, ch.total);
}

// If the baseline itself was bad (here the first read after a recovery), every later read looks
// like a jump; after AS5600_JUMP_REBASE of them tracking restarts, without a fake move.
static void test_three_jumps_in_a_row_restart_from_the_current_read(void)
{
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_REBASE, poll(true, 3000)); // corrupted
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_JUMP, poll(true, 1000));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_JUMP, poll(true, 1010));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_REBASE, poll(true, 1020));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ch.speed);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 1030));
    TEST_ASSERT_EQUAL_INT32(10, ch.total);
    TEST_ASSERT_EQUAL_INT32(10, ch.abs_total);
}

// Corrupted reads that are not in a row do not add up to a rebase: an accepted move clears the count.
static void test_jumps_between_moves_never_rebase(void)
{
    poll(true, 1000);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_JUMP, poll(true, 3000));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 1010));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_JUMP, poll(true, 3000));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 1020));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_JUMP, poll(true, 3000));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 1030));
    TEST_ASSERT_EQUAL_INT32(30, ch.total);
    TEST_ASSERT_EQUAL_INT32(30, ch.abs_total);
}

// The rebase after 3 jumps also restarts the clock: the next move is timed from it, not from the
// read before the jumps.
static void test_move_after_a_jump_rebase_is_timed_from_the_rebase(void)
{
    poll(true, 1000);
    poll(true, 3000);
    poll(true, 3000);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_REBASE, poll(true, 3000));
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 3010));
    TEST_ASSERT_EQUAL_INT32(10, ch.total);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, ch.speed); // 10 counts in 1 ms, not in the 4 ms since 1000
}

// A poll on the same tick as the last accepted read has no elapsed time: it is skipped, with no
// move and dt = 0 (the speed would divide by it), and the baseline and its time are kept.
static void test_poll_on_the_same_tick_is_skipped(void)
{
    poll(true, 1000);
    poll(true, 1010);

    int32_t diff = 12345;
    uint32_t dt = 12345u;
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_SKIP, as5600_track_sample(&ch.s, true, true, 1015, ch.t, TPUS, &diff, &dt));
    TEST_ASSERT_EQUAL_INT32(0, diff);
    TEST_ASSERT_EQUAL_UINT32(0u, dt);
    TEST_ASSERT_EQUAL_INT32(1010, ch.s.angle);
    TEST_ASSERT_EQUAL_UINT32(ch.t, ch.s.t);

    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 1020));
    TEST_ASSERT_EQUAL_INT32(20, ch.total);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, ch.speed);
}

// The speed is held over failed reads and jumps for at most 8 polls in a row: 2 failed reads (the
// 3rd trips the gate) around each of 2 jumps (the 3rd rebases, speed 0).
static void test_speed_is_held_for_at_most_8_polls(void)
{
    poll(true, 1000);
    poll(true, 1010);
    const uint8_t held[8] = {AS5600_TRACK_SKIP, AS5600_TRACK_SKIP, AS5600_TRACK_JUMP,
                             AS5600_TRACK_SKIP, AS5600_TRACK_SKIP, AS5600_TRACK_JUMP,
                             AS5600_TRACK_SKIP, AS5600_TRACK_SKIP};
    for (int k = 0; k < 8; k++)
    {
        const bool ok = (held[k] == AS5600_TRACK_JUMP);
        TEST_ASSERT_EQUAL_UINT8(held[k], poll(ok, ok ? 3000 : 0));
        TEST_ASSERT_EQUAL_FLOAT(10.0f, ch.speed);
    }
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_REBASE, poll(true, 3000));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ch.speed);
    TEST_ASSERT_EQUAL_INT32(10, ch.total);
}

// Real motion at a given speed (mm/s), 1 ms polls, for `n` polls: every read is accepted and the
// counted total is exactly the gear travel.
static void run_at_speed(float mm_s, int n)
{
    const float cnt_per_ms = mm_s / ML_MM_PER_CNT / 1000.0f;
    float pos = 1234.0f;
    poll(true, (uint16_t)pos);
    for (int k = 0; k < n; k++)
    {
        pos += cnt_per_ms;
        const int32_t a = ((int32_t)pos) & 4095;
        TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, (uint16_t)a));
    }
    TEST_ASSERT_EQUAL_INT32((int32_t)pos - 1234, ch.total);
    TEST_ASSERT_EQUAL_INT(0, ch.jumps);
}

static void test_commanded_speed_is_never_rejected(void)
{
    run_at_speed(60.0f, 5000); // PULL_V_FAST / send speed, 5 s, many turns
}

static void test_open_loop_speeds_are_never_rejected(void)
{
    run_at_speed(393.0f, 2000); // a 1000 rpm drive wheel
    chan_init(0x10000000u);
    run_at_speed(850.0f, 2000); // just under the 863 mm/s limit
}

static void test_reverse_speed_is_never_rejected(void)
{
    run_at_speed(-850.0f, 2000);
}

// A slow main-loop pass (for example a flash write) is not a jump: the limit grows with time.
static void test_slow_pass_allows_a_proportional_move(void)
{
    poll(true, 1000);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll_after(10u, true, true, 1683)); // 393 mm/s for 10 ms
    TEST_ASSERT_EQUAL_INT32(683, ch.total);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 68.3f, ch.speed);
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_JUMP, poll_after(10u, true, true, 1683 + 1501));
}

static void test_sysTick_wrap_is_handled(void)
{
    chan_init(0u - TPMS - 1500u);
    poll(true, 500);                                              // 1500 ticks before the wrap
    TEST_ASSERT_EQUAL_UINT8(AS5600_TRACK_MOVE, poll(true, 590)); // elapsed crosses 2^32
    TEST_ASSERT_EQUAL_INT32(90, ch.total);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, ch.speed);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_angle_diff_takes_the_short_way_round);
    RUN_TEST(test_max_step_is_150_counts_per_ms_of_elapsed_time);
    RUN_TEST(test_raw_angle_is_12_bits);
    RUN_TEST(test_first_good_read_is_the_baseline);
    RUN_TEST(test_failed_read_is_skipped_and_next_read_diffs_against_last_good);
    RUN_TEST(test_two_failed_reads_in_a_row_are_both_skipped);
    RUN_TEST(test_failed_reads_through_the_v11_driver_are_skipped);
    RUN_TEST(test_failed_read_is_never_a_baseline);
    RUN_TEST(test_gate_off_drops_the_baseline);
    RUN_TEST(test_impossible_jump_is_rejected);
    RUN_TEST(test_garbage_above_12_bits_is_skipped);
    RUN_TEST(test_garbage_above_12_bits_is_never_a_baseline);
    RUN_TEST(test_limit_is_exactly_150_counts_per_ms);
    RUN_TEST(test_three_jumps_in_a_row_restart_from_the_current_read);
    RUN_TEST(test_jumps_between_moves_never_rebase);
    RUN_TEST(test_move_after_a_jump_rebase_is_timed_from_the_rebase);
    RUN_TEST(test_poll_on_the_same_tick_is_skipped);
    RUN_TEST(test_speed_is_held_for_at_most_8_polls);
    RUN_TEST(test_commanded_speed_is_never_rejected);
    RUN_TEST(test_open_loop_speeds_are_never_rejected);
    RUN_TEST(test_reverse_speed_is_never_rejected);
    RUN_TEST(test_slow_pass_allows_a_proportional_move);
    RUN_TEST(test_sysTick_wrap_is_handled);
    return UNITY_END();
}
