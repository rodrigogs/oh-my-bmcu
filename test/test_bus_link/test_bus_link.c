// Host tests for src/bus_link.h: a lost host link must stay lost however long the bus is silent
// (the old 32-bit deadlines flipped back to "online" 2^31 ticks = 119 s after the loss), a
// protocol that never sent a heartbeat must never read as lost, and ticks wrap at 2^32. The
// went-lost edge that re-arms the BambuBus online-detect registration must fire once per outage.

#include <stdint.h>
#include <unity.h>

#include "bus_link.h"

// SysTick runs at HCLK/8 = 18 MHz.
#define TPMS    18000u
// ---- adapted from bambu_bus_ams.cpp: the bus_link_poll() timeout, ms_to_ticks32(1000u) (ahub_bus.cpp too) ----
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

static void test_went_lost_fires_once_per_outage(void)
{
    bus_link_t l;
    bus_link_init(&l);
    bus_link_state_t prev = BUS_LINK_NOT_SEEN;

    // Boot silence is not an outage.
    TEST_ASSERT_FALSE(bus_link_went_lost(&prev, bus_link_poll(&l, 0u, TIMEOUT)));
    TEST_ASSERT_FALSE(bus_link_went_lost(&prev, bus_link_poll(&l, 10u * TIMEOUT, TIMEOUT)));

    const uint32_t hb = 10u * TIMEOUT;
    bus_link_heartbeat(&l, hb);
    TEST_ASSERT_FALSE(bus_link_went_lost(&prev, bus_link_poll(&l, hb, TIMEOUT)));
    TEST_ASSERT_FALSE(bus_link_went_lost(&prev, bus_link_poll(&l, hb + TIMEOUT, TIMEOUT)));
    TEST_ASSERT_TRUE(bus_link_went_lost(&prev, bus_link_poll(&l, hb + TIMEOUT + 1u, TIMEOUT)));
    TEST_ASSERT_FALSE(bus_link_went_lost(&prev, bus_link_poll(&l, hb + TIMEOUT + 2u, TIMEOUT)));
    TEST_ASSERT_FALSE(bus_link_went_lost(&prev, bus_link_poll(&l, hb + 0x80000000u, TIMEOUT)));
    TEST_ASSERT_FALSE(bus_link_went_lost(&prev, bus_link_poll(&l, hb, TIMEOUT))); // a full wrap later

    // Next heartbeat, next outage: fires again.
    const uint32_t hb2 = hb + 5u;
    bus_link_heartbeat(&l, hb2);
    TEST_ASSERT_FALSE(bus_link_went_lost(&prev, bus_link_poll(&l, hb2, TIMEOUT)));
    TEST_ASSERT_TRUE(bus_link_went_lost(&prev, bus_link_poll(&l, hb2 + TIMEOUT + 1u, TIMEOUT)));
    TEST_ASSERT_FALSE(bus_link_went_lost(&prev, bus_link_poll(&l, hb2 + 2u * TIMEOUT, TIMEOUT)));
}

// ---- adapted from bambu_bus_ams.cpp: bambubus_run()'s online-detect re-arm on bus_link_went_lost() ----
// Main-loop model of the BambuBus online-detect latch (bambu_bus_ams.cpp): registered stands for
// have_registered and is cleared where bambubus_run() calls online_detect_reset(). Registered at
// boot, heartbeats every 300 ms for two minutes across a tick wrap: the latch must hold. The printer
// goes silent: the latch is re-armed once, 1 s in. The printer re-registers during the silence (a
// discovery before its heartbeats resume): that registration must hold through three more wraps.
// Heartbeats resume, then stop again: re-armed once more.
static void test_registration_latch_model(void)
{
    const uint32_t step = 10u * TPMS;
    const uint32_t hb_period = 300u * TPMS;
    uint64_t now = WRAP - 60000ull * TPMS;

    bus_link_t l;
    bus_link_init(&l);
    bus_link_state_t prev = BUS_LINK_NOT_SEEN;
    bool registered = true;
    unsigned rearms = 0;
    uint64_t last_hb = now;

    for (int phase = 0; phase < 2; phase++)
    {
        const uint64_t hb_stop = now + 120000ull * TPMS;
        uint64_t next_hb = now;
        for (; now < hb_stop; now += step)
        {
            if (now >= next_hb)
            {
                bus_link_heartbeat(&l, (uint32_t)now);
                last_hb = now;
                next_hb += hb_period;
            }
            if (bus_link_went_lost(&prev, bus_link_poll(&l, (uint32_t)now, TIMEOUT)))
            {
                registered = false;
                rearms++;
            }
            TEST_ASSERT_TRUE(registered);
        }

        const uint64_t silence_end = now + 3u * WRAP;
        bool reregistered = false;
        for (; now < silence_end; now += step)
        {
            if (bus_link_went_lost(&prev, bus_link_poll(&l, (uint32_t)now, TIMEOUT)))
            {
                TEST_ASSERT_TRUE(now - last_hb > TIMEOUT);
                TEST_ASSERT_TRUE(now - last_hb <= TIMEOUT + step);
                registered = false;
                rearms++;
            }
            if (now - last_hb > 5000ull * TPMS && !reregistered)
            {
                TEST_ASSERT_FALSE(registered);
                registered = true; // the printer's 0x05/0x01 confirm
                reregistered = true;
            }
            if (reregistered)
                TEST_ASSERT_TRUE(registered);
        }
        TEST_ASSERT_TRUE(reregistered);
        TEST_ASSERT_EQUAL_UINT((unsigned)phase + 1u, rearms);
    }
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
    RUN_TEST(test_went_lost_fires_once_per_outage);
    RUN_TEST(test_registration_latch_model);
    return UNITY_END();
}
