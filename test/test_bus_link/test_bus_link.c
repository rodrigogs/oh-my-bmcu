// Host tests for src/bus_link.h: a lost host link must stay lost however long the bus is silent
// (the old 32-bit deadlines flipped back to "online" 2^31 ticks = 119 s after the loss), a
// protocol that never sent a heartbeat must never read as lost, and ticks wrap at 2^32.

#include <stdint.h>
#include <unity.h>

#include "bus_link.h"

// SysTick runs at HCLK/8 = 18 MHz; both protocols time out after 1000 ms of heartbeat silence.
#define TPMS    18000u
#define TIMEOUT (1000u * TPMS)
#define WRAP    4294967296ull // 2^32

void setUp(void) {}
void tearDown(void) {}

static void test_not_seen_until_first_heartbeat_at_any_uptime(void)
{
    // Old AHUB code on a BambuBus printer: error while (int32_t)now > 0, none otherwise.
    static const uint32_t nows[] = {0u, 1u, TIMEOUT + 1u, 0x7FFFFFFFu, 0x80000000u, 0x80000001u, 0xFFFFFFFFu};
    bus_link_t l;
    bus_link_init(&l);
    for (unsigned i = 0; i < sizeof(nows) / sizeof(nows[0]); i++)
        TEST_ASSERT_EQUAL_INT(BUS_LINK_NOT_SEEN, bus_link_poll(&l, nows[i], TIMEOUT));
}

static void test_alive_up_to_timeout_then_lost(void)
{
    bus_link_t l;
    bus_link_init(&l);
    bus_link_heartbeat(&l, 5000u);
    TEST_ASSERT_EQUAL_INT(BUS_LINK_ALIVE, bus_link_poll(&l, 5000u, TIMEOUT));
    TEST_ASSERT_EQUAL_INT(BUS_LINK_ALIVE, bus_link_poll(&l, 5000u + TIMEOUT, TIMEOUT));
    TEST_ASSERT_EQUAL_INT(BUS_LINK_LOST, bus_link_poll(&l, 5000u + TIMEOUT + 1u, TIMEOUT));
}

static void test_lost_stays_lost_through_silence_longer_than_2_pow_31_ticks(void)
{
    const uint32_t hb = 1000u;
    bus_link_t l;
    bus_link_init(&l);
    bus_link_heartbeat(&l, hb);
    TEST_ASSERT_EQUAL_INT(BUS_LINK_LOST, bus_link_poll(&l, hb + TIMEOUT + 1u, TIMEOUT));

    // (int32_t)(now - hb) turns negative from hb + 2^31 on and is 0 again after a full wrap.
    static const uint32_t later[] = {0x80000000u, 0x80000001u, 0xC0000000u, 0xFFFFFFFFu, 0u, 1u, TIMEOUT};
    for (unsigned i = 0; i < sizeof(later) / sizeof(later[0]); i++)
        TEST_ASSERT_EQUAL_INT(BUS_LINK_LOST, bus_link_poll(&l, hb + later[i], TIMEOUT));
}

static void test_heartbeat_just_before_tick_wrap(void)
{
    const uint32_t hb = 0xFFFFFFFFu - 1000u;
    bus_link_t l;
    bus_link_init(&l);
    bus_link_heartbeat(&l, hb);
    TEST_ASSERT_EQUAL_INT(BUS_LINK_ALIVE, bus_link_poll(&l, 0xFFFFFFFFu, TIMEOUT));
    TEST_ASSERT_EQUAL_INT(BUS_LINK_ALIVE, bus_link_poll(&l, 0u, TIMEOUT));
    TEST_ASSERT_EQUAL_INT(BUS_LINK_ALIVE, bus_link_poll(&l, hb + TIMEOUT, TIMEOUT));
    TEST_ASSERT_EQUAL_INT(BUS_LINK_LOST, bus_link_poll(&l, hb + TIMEOUT + 1u, TIMEOUT));
    TEST_ASSERT_EQUAL_INT(BUS_LINK_LOST, bus_link_poll(&l, hb + 0x80000000u, TIMEOUT));
}

static void test_heartbeat_stamped_after_now_is_fresh(void)
{
    // The RX IRQ can stamp a heartbeat after bambubus_run() sampled now.
    bus_link_t l;
    bus_link_init(&l);
    bus_link_heartbeat(&l, 0x00000010u);
    TEST_ASSERT_EQUAL_INT(BUS_LINK_ALIVE, bus_link_poll(&l, 0xFFFFFF00u, TIMEOUT));
    TEST_ASSERT_EQUAL_INT(BUS_LINK_ALIVE, bus_link_poll(&l, 0x00000010u, TIMEOUT));
}

static void test_new_heartbeat_recovers_a_lost_link(void)
{
    bus_link_t l;
    bus_link_init(&l);
    bus_link_heartbeat(&l, 100u);
    TEST_ASSERT_EQUAL_INT(BUS_LINK_LOST, bus_link_poll(&l, 100u + TIMEOUT + 1u, TIMEOUT));
    TEST_ASSERT_EQUAL_INT(BUS_LINK_LOST, bus_link_poll(&l, 100u + 0x90000000u, TIMEOUT));

    const uint32_t hb = 100u + 0x90000000u + 7u;
    bus_link_heartbeat(&l, hb);
    TEST_ASSERT_EQUAL_INT(BUS_LINK_ALIVE, bus_link_poll(&l, hb + TIMEOUT, TIMEOUT));
    TEST_ASSERT_EQUAL_INT(BUS_LINK_LOST, bus_link_poll(&l, hb + TIMEOUT + 1u, TIMEOUT));
}

static void test_init_forgets_the_link(void)
{
    bus_link_t l;
    bus_link_init(&l);
    bus_link_heartbeat(&l, 42u);
    bus_link_init(&l);
    TEST_ASSERT_EQUAL_INT(BUS_LINK_NOT_SEEN, bus_link_poll(&l, 42u, TIMEOUT));
}

// Main-loop model: heartbeats every 300 ms across a tick wrap, then the printer goes silent while
// the BMCU keeps polling every 10 ms for three full wraps (~12 min). The link must be alive the
// whole time heartbeats arrive, go lost once after 1 s of silence and never come back.
static void test_polled_loop_through_wraps_goes_lost_once_and_stays_lost(void)
{
    const uint32_t step = 10u * TPMS;
    const uint32_t hb_period = 300u * TPMS;
    uint64_t now = WRAP - 60000ull * TPMS; // 60 s before the first wrap
    const uint64_t hb_stop = now + 120000ull * TPMS;
    uint64_t next_hb = now;
    uint64_t last_hb = now;

    bus_link_t l;
    bus_link_init(&l);

    for (; now < hb_stop; now += step)
    {
        if (now >= next_hb)
        {
            bus_link_heartbeat(&l, (uint32_t)now);
            last_hb = now;
            next_hb += hb_period;
        }
        TEST_ASSERT_EQUAL_INT(BUS_LINK_ALIVE, bus_link_poll(&l, (uint32_t)now, TIMEOUT));
    }

    unsigned lost_transitions = 0;
    bus_link_state_t prev = BUS_LINK_ALIVE;
    for (; now < hb_stop + 3u * WRAP; now += step)
    {
        const bus_link_state_t s = bus_link_poll(&l, (uint32_t)now, TIMEOUT);
        if (now - last_hb <= TIMEOUT)
            TEST_ASSERT_EQUAL_INT(BUS_LINK_ALIVE, s);
        else
            TEST_ASSERT_EQUAL_INT(BUS_LINK_LOST, s);
        if (s == BUS_LINK_LOST && prev != BUS_LINK_LOST) lost_transitions++;
        prev = s;
    }
    TEST_ASSERT_EQUAL_UINT(1u, lost_transitions);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_not_seen_until_first_heartbeat_at_any_uptime);
    RUN_TEST(test_alive_up_to_timeout_then_lost);
    RUN_TEST(test_lost_stays_lost_through_silence_longer_than_2_pow_31_ticks);
    RUN_TEST(test_heartbeat_just_before_tick_wrap);
    RUN_TEST(test_heartbeat_stamped_after_now_is_fresh);
    RUN_TEST(test_new_heartbeat_recovers_a_lost_link);
    RUN_TEST(test_init_forgets_the_link);
    RUN_TEST(test_polled_loop_through_wraps_goes_lost_once_and_stays_lost);
    return UNITY_END();
}
