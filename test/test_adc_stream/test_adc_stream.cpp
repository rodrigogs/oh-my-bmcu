// Host tests for src/adc_stream.h: when Motion_control_run treats the ADC readings as stale and stops
// every channel. A live stream must never read stale, however late a main-loop pass comes (a flash
// erase, the worst NVM job, a stall up to the watchdog's longest timeout): the poll that ends the
// stall stamps the halves the DMA completed meanwhile. A stream that stopped must read stale
// ADC_STREAM_STALE_MS after its last half, stay stale for as long as it is stopped (also past the
// 238.6 s wrap of a 32-bit SysTick difference), and read fresh again from the pass that processes
// its first new half. A stream that never delivered a half since boot is stale. A stale stream is
// restarted (ADC_DMA_restart_if_stale) once its readings are ADC_STREAM_RESTART_MS old, and again
// every ADC_STREAM_RESTART_MS for as long as it stays stopped; a live one never is.
//
// ADC_DMA.cpp needs the CH32 SDK and is not built on the host. The stream is simulated at the
// firmware's clock: SysTick and ADCCLK both at 18 MHz (144 MHz HCLK / 8), a half every 21504 ticks.
// C++, as jam_latch.h (for its trip time) maps ams.h's _filament_motion.

#include <stdint.h>
#include <unity.h>

#include "adc_stream.h"
#include "jam_latch.h"

#define TPMS 18000u                       // time_hw_tpms: SysTick at 144 MHz / 8
#define HALF_TICKS (32u * 8u * 84u)       // 32 scans x 8 conversions x (71.5 + 12.5) ADCCLK cycles
#define READ_TICKS 90u                    // poll to ADC_DMA_age_ticks(): a few microseconds
#define MS(x) ((uint64_t)(x) * TPMS)

void setUp(void) {}
void tearDown(void) {}

// ---- Simulated stream and main loop ----
static uint64_t s_now;        // SysTick (time_ticks64)
static bool     s_alive;      // the ADCs convert and the DMA copies
static uint64_t s_next_half;  // when the DMA completes its next half
static bool     s_flag;       // HT or TC set, not yet processed
static uint64_t s_stamp;      // g_half_ticks
static uint64_t s_restart;    // g_restart_ticks
static uint32_t s_restarts;   // adc_dma_restart calls
static bool     s_revives;    // a restart brings the stream back

static void stream_boot(bool alive)
{
    s_now       = 0u;
    s_alive     = alive;
    s_next_half = HALF_TICKS;
    s_flag      = false;
    s_stamp     = 0u;
    s_restart   = 0u;
    s_restarts  = 0u;
    s_revives   = false;
}

static void stream_advance(uint64_t ticks)
{
    const uint64_t end = s_now + ticks;
    while (s_alive && (s_next_half <= end))
    {
        s_flag = true;
        s_next_half += HALF_TICKS;
    }
    s_now = end;
}

static void stream_stop(void) { s_alive = false; }

static void stream_resume(void)
{
    s_alive     = true;
    s_next_half = s_now + HALF_TICKS;
}

// One Motion_control_run: MC_PULL_ONLINE_read polls, then the age is read. Returns g_adc_stale.
// ---- adapted from ADC_DMA.cpp: ADC_DMA_poll stamps each half it processes (g_half_ticks) ----
// ---- anchor: ADC_DMA_poll ----
// ---- anchor: process_half_update_filter from /g_half_ticks = time_ticks64\(\);/ to /g_half_ticks = time_ticks64\(\);/ ----
// ---- anchor: ADC_DMA_age_ticks ----
static bool pass(void)
{
    if (s_flag)
    {
        s_flag  = false;
        s_stamp = s_now;
    }
    return adc_stream_stale(adc_stream_age_ticks(s_now + READ_TICKS, s_stamp), TPMS);
}

// pass(), then Motion_control_run's restart of a stale stream. A restart starts the DMA at the
// start of the buffer with its flags clear; if it revives the stream, the first new half comes
// HALF_TICKS later. Returns g_adc_stale.
// ---- adapted from Motion_control.cpp: Motion_control_run restarts a stale stream right after the age read ----
// ---- anchor: Motion_control_run from /g_adc_stale = adc_stream_stale/ to /ADC_DMA_restart_if_stale\(\);/ ----
// ---- adapted from ADC_DMA.cpp: ADC_DMA_restart_if_stale and adc_dma_restart (g_restart_ticks) ----
// ---- anchor: ADC_DMA_restart_if_stale ----
// ---- anchor: adc_dma_restart ----
static bool pass_restart(void)
{
    const bool stale = pass();
    if (stale)
    {
        const uint64_t now = s_now + READ_TICKS;
        if (adc_stream_restart_due(adc_stream_age_ticks(now, s_stamp), now - s_restart, TPMS))
        {
            s_flag    = false;
            s_restart = now;
            s_restarts++;
            if (s_revives) stream_resume();
        }
    }
    return stale;
}

// ---- Threshold ----

static void test_the_threshold_is_83_halves_and_below_the_jam_trip(void)
{
    TEST_ASSERT_EQUAL_UINT32(21504u, HALF_TICKS);
    TEST_ASSERT_EQUAL_UINT32(1194u, (uint32_t)(HALF_TICKS * 1000u / TPMS)); // us per half
    TEST_ASSERT_EQUAL_UINT32(100u, ADC_STREAM_STALE_MS);
    TEST_ASSERT_EQUAL_UINT32(83u, (uint32_t)(MS(ADC_STREAM_STALE_MS) / HALF_TICKS));
    // Frozen readings alone cannot complete a jam trip before the channels stop.
    TEST_ASSERT_TRUE(ADC_STREAM_STALE_MS < JAM_TRIP_MS);
}

static void test_stale_only_past_the_threshold(void)
{
    TEST_ASSERT_FALSE(adc_stream_stale(0u, TPMS));
    TEST_ASSERT_FALSE(adc_stream_stale(HALF_TICKS, TPMS));
    TEST_ASSERT_FALSE(adc_stream_stale(MS(ADC_STREAM_STALE_MS), TPMS));
    TEST_ASSERT_TRUE(adc_stream_stale(MS(ADC_STREAM_STALE_MS) + 1u, TPMS));
    TEST_ASSERT_TRUE(adc_stream_stale(UINT64_MAX, TPMS));

    // time_hw_tpms before time_hw_init (1000)
    TEST_ASSERT_FALSE(adc_stream_stale(100000u, 1000u));
    TEST_ASSERT_TRUE(adc_stream_stale(100001u, 1000u));
}

static void test_age_is_the_tick_difference_and_0_for_a_later_stamp(void)
{
    TEST_ASSERT_EQUAL_UINT64(0u, adc_stream_age_ticks(5u, 5u));
    TEST_ASSERT_EQUAL_UINT64(7u, adc_stream_age_ticks(12u, 5u));
    // Motion_control_run takes now_ticks64 before the poll stamps a half.
    TEST_ASSERT_EQUAL_UINT64(0u, adc_stream_age_ticks(5u, 6u));
    TEST_ASSERT_FALSE(adc_stream_stale(adc_stream_age_ticks(5u, 6u), TPMS));
    TEST_ASSERT_EQUAL_UINT64(0u, adc_stream_age_ticks(0u, UINT64_MAX));
    // Past a 32-bit wrap (2^32 ticks, 238.6 s at 18 MHz) the age keeps growing.
    TEST_ASSERT_EQUAL_UINT64(0x100000000ull + 3u, adc_stream_age_ticks(0x100000005ull, 2u));
}

// ---- Live stream ----

static void test_a_live_stream_never_reads_stale_across_main_loop_stalls(void)
{
    // 1 ms passes, and passes that come late: a page erase (16 ms), the worst NVM job (about 36 ms),
    // WDG_FEED_GAP_MAX_MS (100 ms), and up to the watchdog's longest timeout (1.6 s) and beyond.
    static const uint32_t stall_ms[] = {16u, 36u, 100u, 101u, 250u, 1000u, 1600u, 5000u};
    stream_boot(true);
    stream_advance(MS(2400)); // the boot: ADC_DMA_wait_full and Motion_control_init poll on their own
    uint32_t passes = 0u;

    for (uint32_t k = 0u; k < sizeof(stall_ms) / sizeof(stall_ms[0]); k++)
    {
        for (uint32_t i = 0u; i < 1000u; i++)
        {
            TEST_ASSERT_FALSE(pass());
            passes++;
            stream_advance(MS(1));
        }
        stream_advance(MS(stall_ms[k]));
        TEST_ASSERT_FALSE_MESSAGE(pass(), "the pass after a stall");
        passes++;
        stream_advance(MS(1));
    }
    TEST_ASSERT_EQUAL_UINT32(8008u, passes);

    // Passes faster than the halves (0.1 ms): most find no new half, and none reads stale.
    for (uint32_t i = 0u; i < 100000u; i++)
    {
        TEST_ASSERT_FALSE(pass());
        stream_advance(MS(1) / 10u);
    }
}

// ---- Stopped stream ----

static void test_a_stopped_stream_reads_stale_100ms_after_its_last_half(void)
{
    stream_boot(true);
    stream_advance(MS(3000));
    TEST_ASSERT_FALSE(pass());

    // The last half before the stop is the one that completes at or before the stop.
    stream_advance(MS(1) / 2u);
    const uint64_t last_half = s_next_half - HALF_TICKS;
    stream_stop();

    uint64_t first_stale = 0u;
    while (s_now < MS(4000))
    {
        stream_advance(MS(1));
        if (pass())
        {
            first_stale = s_now;
            break;
        }
    }
    // The last half is stamped by the next pass (within 1 ms), so the first stale pass comes more
    // than ADC_STREAM_STALE_MS after the half and no more than two passes later.
    TEST_ASSERT_TRUE(first_stale != 0u);
    TEST_ASSERT_TRUE(first_stale + READ_TICKS > last_half + MS(ADC_STREAM_STALE_MS));
    TEST_ASSERT_TRUE(first_stale <= last_half + MS(ADC_STREAM_STALE_MS + 2u));
}

static void test_a_stopped_stream_stays_stale_past_the_32bit_tick_wrap(void)
{
    stream_boot(true);
    stream_advance(MS(3000));
    TEST_ASSERT_FALSE(pass());
    stream_stop();
    stream_advance(MS(ADC_STREAM_STALE_MS + 1u));

    // One hour in 10 ms passes: the 32-bit difference wraps 15 times meanwhile.
    for (uint32_t i = 0u; i < 360000u; i++)
    {
        TEST_ASSERT_TRUE(pass());
        stream_advance(MS(10));
    }
    TEST_ASSERT_TRUE(s_now - s_stamp > 15u * 0x100000000ull);
}

static void test_the_stream_is_fresh_again_from_its_first_new_half(void)
{
    stream_boot(true);
    stream_advance(MS(3000));
    TEST_ASSERT_FALSE(pass());
    stream_stop();
    stream_advance(MS(500));
    TEST_ASSERT_TRUE(pass());

    // A restart (ADC_DMA_poll after a transfer error): stale until the first new half is processed.
    stream_resume();
    const uint64_t first_half = s_next_half;
    while (s_now + MS(1) / 10u < first_half)
    {
        stream_advance(MS(1) / 10u);
        TEST_ASSERT_TRUE(pass());
    }
    stream_advance(first_half - s_now);
    TEST_ASSERT_FALSE(pass());
    for (uint32_t i = 0u; i < 1000u; i++)
    {
        stream_advance(MS(1));
        TEST_ASSERT_FALSE(pass());
    }
}

static void test_a_stream_that_never_delivered_a_half_is_stale(void)
{
    // ADC_DMA_init found no data (the warm-up waits 350 ms, ADC_DMA_wait_full 2 s): g_half_ticks is
    // still 0 at the first Motion_control_run.
    stream_boot(false);
    stream_advance(MS(2400));
    TEST_ASSERT_TRUE(pass());
    stream_advance(MS(ADC_STREAM_STALE_MS));
    TEST_ASSERT_TRUE(pass());

    // The same stream with halves: fresh at the same point.
    stream_boot(true);
    stream_advance(MS(2400));
    TEST_ASSERT_FALSE(pass());
}

// ---- Restart of a stale stream ----

static void test_a_restart_is_due_only_past_500ms_of_age_and_since_the_last(void)
{
    TEST_ASSERT_EQUAL_UINT32(500u, ADC_STREAM_RESTART_MS);
    TEST_ASSERT_TRUE(ADC_STREAM_RESTART_MS > ADC_STREAM_STALE_MS);
    const uint64_t r = MS(ADC_STREAM_RESTART_MS);

    // A live stream: its age is at most a half (plus a late pass), however long since a restart.
    TEST_ASSERT_FALSE(adc_stream_restart_due(0u, UINT64_MAX, TPMS));
    TEST_ASSERT_FALSE(adc_stream_restart_due(HALF_TICKS, UINT64_MAX, TPMS));
    TEST_ASSERT_FALSE(adc_stream_restart_due(MS(ADC_STREAM_STALE_MS) + 1u, UINT64_MAX, TPMS));

    // Exactly 500 ms is not due; both must be past it.
    TEST_ASSERT_FALSE(adc_stream_restart_due(r, r, TPMS));
    TEST_ASSERT_FALSE(adc_stream_restart_due(r + 1u, r, TPMS));
    TEST_ASSERT_FALSE(adc_stream_restart_due(r, r + 1u, TPMS));
    TEST_ASSERT_TRUE(adc_stream_restart_due(r + 1u, r + 1u, TPMS));
    TEST_ASSERT_TRUE(adc_stream_restart_due(UINT64_MAX, UINT64_MAX, TPMS));

    // time_hw_tpms before time_hw_init (1000)
    TEST_ASSERT_FALSE(adc_stream_restart_due(500000u, 500001u, 1000u));
    TEST_ASSERT_TRUE(adc_stream_restart_due(500001u, 500001u, 1000u));
}

static void test_a_live_stream_is_never_restarted(void)
{
    static const uint32_t stall_ms[] = {16u, 36u, 100u, 500u, 501u, 1600u, 5000u};
    stream_boot(true);
    stream_advance(MS(2400));

    for (uint32_t k = 0u; k < sizeof(stall_ms) / sizeof(stall_ms[0]); k++)
    {
        for (uint32_t i = 0u; i < 1000u; i++)
        {
            TEST_ASSERT_FALSE(pass_restart());
            stream_advance(MS(1));
        }
        stream_advance(MS(stall_ms[k]));
        TEST_ASSERT_FALSE(pass_restart());
        stream_advance(MS(1));
    }
    TEST_ASSERT_EQUAL_UINT32(0u, s_restarts);
}

static void test_a_dead_stream_is_restarted_every_500ms_while_it_stays_dead(void)
{
    stream_boot(true);
    stream_advance(MS(3000));
    TEST_ASSERT_FALSE(pass_restart());
    stream_advance(MS(1) / 2u);
    const uint64_t last_half = s_next_half - HALF_TICKS;
    stream_stop();

    // The first restart: the first pass more than 500 ms after the last half.
    while ((s_restarts == 0u) && (s_now < MS(5000)))
    {
        stream_advance(MS(1));
        TEST_ASSERT_TRUE(pass_restart() || (s_now + READ_TICKS <= last_half + MS(ADC_STREAM_STALE_MS + 2u)));
    }
    TEST_ASSERT_EQUAL_UINT32(1u, s_restarts);
    TEST_ASSERT_TRUE(s_restart > last_half + MS(ADC_STREAM_RESTART_MS));
    TEST_ASSERT_TRUE(s_restart <= last_half + MS(ADC_STREAM_RESTART_MS + 2u));

    // Still dead: stale on every pass, the next restart 501 passes later (500 ms is not due), and so on.
    uint64_t prev = s_restart;
    uint32_t since = 0u;
    for (uint32_t i = 0u; i < 5010u; i++)
    {
        stream_advance(MS(1));
        since++;
        TEST_ASSERT_TRUE(pass_restart());
        if (s_restart != prev)
        {
            TEST_ASSERT_EQUAL_UINT32(ADC_STREAM_RESTART_MS + 1u, since);
            TEST_ASSERT_EQUAL_UINT64(MS(ADC_STREAM_RESTART_MS + 1u), s_restart - prev);
            prev  = s_restart;
            since = 0u;
        }
    }
    TEST_ASSERT_EQUAL_UINT32(1u + 10u, s_restarts);
}

static void test_a_restart_that_revives_the_stream_is_the_only_one(void)
{
    stream_boot(true);
    stream_advance(MS(3000));
    TEST_ASSERT_FALSE(pass_restart());
    stream_stop();
    s_revives = true;

    while (s_restarts == 0u)
    {
        stream_advance(MS(1));
        TEST_ASSERT_TRUE(s_now < MS(4000));
        (void)pass_restart();
    }
    // Stale until the first new half, HALF_TICKS after the restart; fresh from then on.
    const uint64_t first_half = s_next_half;
    while (s_now + MS(1) / 10u < first_half)
    {
        stream_advance(MS(1) / 10u);
        TEST_ASSERT_TRUE(pass_restart());
    }
    stream_advance(first_half - s_now);
    TEST_ASSERT_FALSE(pass_restart());
    for (uint32_t i = 0u; i < 3000u; i++)
    {
        stream_advance(MS(1));
        TEST_ASSERT_FALSE(pass_restart());
    }
    TEST_ASSERT_EQUAL_UINT32(1u, s_restarts);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_the_threshold_is_83_halves_and_below_the_jam_trip);
    RUN_TEST(test_stale_only_past_the_threshold);
    RUN_TEST(test_age_is_the_tick_difference_and_0_for_a_later_stamp);
    RUN_TEST(test_a_live_stream_never_reads_stale_across_main_loop_stalls);
    RUN_TEST(test_a_stopped_stream_reads_stale_100ms_after_its_last_half);
    RUN_TEST(test_a_stopped_stream_stays_stale_past_the_32bit_tick_wrap);
    RUN_TEST(test_the_stream_is_fresh_again_from_its_first_new_half);
    RUN_TEST(test_a_stream_that_never_delivered_a_half_is_stale);
    RUN_TEST(test_a_restart_is_due_only_past_500ms_of_age_and_since_the_last);
    RUN_TEST(test_a_live_stream_is_never_restarted);
    RUN_TEST(test_a_dead_stream_is_restarted_every_500ms_while_it_stays_dead);
    RUN_TEST(test_a_restart_that_revives_the_stream_is_the_only_one);
    return UNITY_END();
}
