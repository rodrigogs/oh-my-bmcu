// Host tests for main()'s host decision (V11's inline code in src/main.cpp; the host_device_type_*
// values come from src/host_link.h): the protocol the host speaks and whether main() is offline,
// which decides whether Motion_control_run() gets error = -1 (motors stopped) and the SYS LED
// colour. Before any packet the BMCU must stay offline; a lost BambuBus link must set offline
// although the AHUB link, never seen, reports nothing; only the host's own protocol decides; and
// V11's AMS_AUTO arrange wait shows red. bambubus_run() and ahubus_run() report error while their
// link is lost (bus_link.h), and bambubus_run() reports a heartbeat on a pass that handled no packet.

#include <stdbool.h>
#include <stdint.h>
#include <unity.h>

#include "bus_link.h"
#include "host_link.h"

// SysTick runs at HCLK/8 = 18 MHz.
#define TPMS    18000u
// ---- adapted from bambu_bus_ams.cpp: the bus_link_poll() timeout, ms_to_ticks32(1000u) (ahub_bus.cpp too) ----
// ---- anchor: bambubus_run from /bus_link_poll\(/ to /bus_link_poll\(/ ----
#define TIMEOUT (1000u * TPMS)
#define WRAP    4294967296ull // 2^32

// What bambubus_run() / ahubus_run() return, as main() sees it: error, none, heartbeat, or any other
// package type (a frame the BMCU handled).
typedef enum
{
    STU_ERROR,
    STU_NONE,
    STU_HEARTBEAT,
    STU_PACKET,
} stu_t;

#define LED_RED   0x100000u // SYS_RGB.set_RGB(0x10, 0x00, 0x00)
#define LED_WHITE 0x383532u // SYS_RGB.set_RGB(0x38, 0x35, 0x32)

void setUp(void) {}
void tearDown(void) {}

// ---- adapted from main.cpp: the main loop's host type, offline decision and SYS LED ----
// ---- anchor: main from /bus_host_device_type = host_device_type_ams/ to /SYS_RGB.set_RGB\(0x38/ ----
// One pass: the error handed to Motion_control_run(), the SYS LED colour, and the boot-restore drop
// (boot_restore_drop) counted.
static int motion_error;
static uint32_t sys_led;
static unsigned restore_drops;

static int main_pass(uint16_t *host_type, stu_t bambubus_stu, stu_t ahub_stu, bool arrange_wait)
{
    if (bambubus_stu != STU_NONE && bambubus_stu != STU_ERROR)
        *host_type = host_device_type_ams;
    else if (ahub_stu != STU_NONE && ahub_stu != STU_ERROR)
        *host_type = host_device_type_ahub;

    if (ahub_stu == STU_HEARTBEAT)
        restore_drops++;

    const int error = *host_type == host_device_type_ams
        ? (bambubus_stu == STU_ERROR ? -1 : 0)
        : *host_type == host_device_type_ahub
            ? (ahub_stu == STU_ERROR ? -1 : 0) : -1;
    if (arrange_wait) sys_led = LED_RED;
    else if (error) sys_led = LED_RED;
    else sys_led = LED_WHITE;

    motion_error = error;
    return error;
}

static bool offline_pass(uint16_t *host_type, stu_t bambubus_stu, stu_t ahub_stu)
{
    return main_pass(host_type, bambubus_stu, ahub_stu, false) != 0;
}

// ---- adapted from bambu_bus_ams.cpp: bambubus_run()'s status after the packet (error, hb_unreported) ----
// ---- anchor: bambubus_run from /bool hb_new = false;/ to /return stu;/ ----
// packet: this pass handled a frame (its type is what bambubus_run() returns); hb: bus_rx_poll()
// flagged a heartbeat since the last pass, stamped at now.
typedef struct
{
    bus_link_t link;
    bool hb_unreported;
} bambubus_model_t;

static void bambubus_model_init(bambubus_model_t *m)
{
    bus_link_init(&m->link);
    m->hb_unreported = false;
}

static stu_t bambubus_pass(bambubus_model_t *m, bool packet, bool hb, uint32_t now)
{
    if (hb)
    {
        bus_link_heartbeat(&m->link, now);
        m->hb_unreported = true;
    }

    stu_t stu = packet ? STU_PACKET : STU_NONE;
    if (bus_link_poll(&m->link, now, TIMEOUT) != BUS_LINK_ALIVE)
        stu = STU_ERROR;
    else if (stu == STU_NONE && m->hb_unreported)
    {
        m->hb_unreported = false;
        stu = STU_HEARTBEAT;
    }
    return stu;
}

// ---- adapted from ahub_bus.cpp: ahubus_run()'s status (a heartbeat packet feeds the link, error once lost) ----
// ---- anchor: ahubus_run ----
static stu_t ahub_pass(bus_link_t *l, bool hb_packet, bool other_packet, uint32_t now)
{
    stu_t stu = STU_NONE;
    if (hb_packet)
    {
        stu = STU_HEARTBEAT;
        bus_link_heartbeat(l, now);
    }
    else if (other_packet)
        stu = STU_PACKET;

    if (bus_link_poll(l, now, TIMEOUT) == BUS_LINK_LOST)
        stu = STU_ERROR;
    return stu;
}

static void reset_counts(void)
{
    motion_error = 0;
    sys_led = 0u;
    restore_drops = 0u;
}

// ---- the decision ----

static void test_boot_before_any_packet_stays_offline(void)
{
    // BambuBus reports error from boot until its first heartbeat, the AHUB nothing: neither decides.
    const stu_t stus[] = {STU_NONE, STU_ERROR};
    for (unsigned b = 0; b < 2u; b++)
        for (unsigned a = 0; a < 2u; a++)
        {
            uint16_t host = host_device_type_none;
            reset_counts();
            TEST_ASSERT_EQUAL_INT(-1, main_pass(&host, stus[b], stus[a], false));
            TEST_ASSERT_EQUAL_HEX16(host_device_type_none, host);
            TEST_ASSERT_EQUAL_INT(-1, motion_error);
            TEST_ASSERT_EQUAL_HEX32(LED_RED, sys_led);
        }
}

static void test_first_bambubus_packet_or_heartbeat_sets_the_host_and_goes_online(void)
{
    const stu_t firsts[] = {STU_PACKET, STU_HEARTBEAT};
    for (unsigned i = 0; i < 2u; i++)
    {
        uint16_t host = host_device_type_none;
        reset_counts();
        TEST_ASSERT_EQUAL_INT(0, main_pass(&host, firsts[i], STU_NONE, false));
        TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);
        TEST_ASSERT_EQUAL_HEX32(LED_WHITE, sys_led);

        // Passes with nothing on the bus stay online while the link is up.
        for (int k = 0; k < 100; k++)
            TEST_ASSERT_FALSE(offline_pass(&host, STU_NONE, STU_NONE));
        TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);
    }
}

static void test_bambubus_loss_with_the_ahub_never_seen_sets_offline(void)
{
    uint16_t host = host_device_type_none;
    reset_counts();
    TEST_ASSERT_FALSE(offline_pass(&host, STU_HEARTBEAT, STU_NONE));

    // The AHUB link, never seen, reports nothing: it must not mask the lost BambuBus link.
    for (int i = 0; i < 100; i++)
    {
        TEST_ASSERT_TRUE(offline_pass(&host, STU_ERROR, STU_NONE));
        TEST_ASSERT_EQUAL_INT(-1, motion_error);
        TEST_ASSERT_EQUAL_HEX32(LED_RED, sys_led);
    }
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);

    // The next heartbeat brings it back.
    TEST_ASSERT_FALSE(offline_pass(&host, STU_HEARTBEAT, STU_NONE));
    TEST_ASSERT_EQUAL_INT(0, motion_error);
    TEST_ASSERT_EQUAL_HEX32(LED_WHITE, sys_led);
}

static void test_ahub_link_does_not_decide_for_a_bambubus_host(void)
{
    // An AHUB link that was up once and is lost now (the host was AHUB, then BambuBus).
    uint16_t host = host_device_type_ams;
    TEST_ASSERT_FALSE(offline_pass(&host, STU_NONE, STU_ERROR));
    TEST_ASSERT_FALSE(offline_pass(&host, STU_PACKET, STU_ERROR));
    TEST_ASSERT_FALSE(offline_pass(&host, STU_HEARTBEAT, STU_ERROR));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);
}

static void test_an_ahub_frame_moves_a_silent_bambubus_host_to_the_ahub(void)
{
    uint16_t host = host_device_type_ams;
    TEST_ASSERT_FALSE(offline_pass(&host, STU_NONE, STU_HEARTBEAT));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ahub, host);

    // BambuBus goes lost (no BambuBus heartbeats any more): only the AHUB link decides now.
    TEST_ASSERT_FALSE(offline_pass(&host, STU_ERROR, STU_NONE));
    TEST_ASSERT_FALSE(offline_pass(&host, STU_ERROR, STU_PACKET));
    TEST_ASSERT_TRUE(offline_pass(&host, STU_NONE, STU_ERROR));
    TEST_ASSERT_TRUE(offline_pass(&host, STU_ERROR, STU_ERROR));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ahub, host);

    // Any BambuBus frame flips it back, whatever the AHUB link reports.
    TEST_ASSERT_FALSE(offline_pass(&host, STU_PACKET, STU_ERROR));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);

    // A query (no heartbeat) moves it as well (V11 latches on any frame, the fork on a heartbeat only).
    TEST_ASSERT_FALSE(offline_pass(&host, STU_NONE, STU_PACKET));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ahub, host);
}

static void test_first_ahub_frame_goes_online_with_bambubus_never_seen(void)
{
    const stu_t firsts[] = {STU_HEARTBEAT, STU_PACKET};
    for (unsigned i = 0; i < 2u; i++)
    {
        uint16_t host = host_device_type_none;
        TEST_ASSERT_FALSE(offline_pass(&host, STU_ERROR, firsts[i]));
        TEST_ASSERT_EQUAL_HEX16(host_device_type_ahub, host);
        TEST_ASSERT_FALSE(offline_pass(&host, STU_ERROR, STU_NONE));
    }
}

static void test_bambubus_wins_a_pass_that_carries_both(void)
{
    const uint16_t starts[] = {host_device_type_none, host_device_type_ams, host_device_type_ahub};
    const stu_t frames[] = {STU_HEARTBEAT, STU_PACKET};
    for (unsigned i = 0; i < 3u; i++)
        for (unsigned b = 0; b < 2u; b++)
            for (unsigned a = 0; a < 2u; a++)
            {
                uint16_t host = starts[i];
                TEST_ASSERT_FALSE(offline_pass(&host, frames[b], frames[a]));
                TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);
            }
}

static void test_arrange_wait_shows_red_while_the_motors_run(void)
{
    // AMS_AUTO only (bambubus_arrange_wait() is always false with AMS_NUM 0..3): the unit waits for
    // the printer's arrange step. The motors are not stopped, only the LED says so.
    uint16_t host = host_device_type_none;
    reset_counts();
    TEST_ASSERT_EQUAL_INT(0, main_pass(&host, STU_PACKET, STU_NONE, true));
    TEST_ASSERT_EQUAL_HEX32(LED_RED, sys_led);
    TEST_ASSERT_EQUAL_INT(0, main_pass(&host, STU_NONE, STU_NONE, false));
    TEST_ASSERT_EQUAL_HEX32(LED_WHITE, sys_led);
}

static void test_only_an_ahub_heartbeat_drops_the_boot_restore(void)
{
    uint16_t host = host_device_type_none;
    reset_counts();
    offline_pass(&host, STU_HEARTBEAT, STU_NONE);
    offline_pass(&host, STU_PACKET, STU_PACKET);
    offline_pass(&host, STU_ERROR, STU_ERROR);
    TEST_ASSERT_EQUAL_UINT(0u, restore_drops);
    offline_pass(&host, STU_PACKET, STU_HEARTBEAT); // the AHUB frame drops it although BambuBus wins
    TEST_ASSERT_EQUAL_UINT(1u, restore_drops);
}

static void test_every_host_type_and_pair_of_reports(void)
{
    // Every host type main() can hold (and one it cannot), every pair of reports, with and without
    // the arrange wait: the rules above, stated once more.
    const uint16_t types[] = {host_device_type_none, host_device_type_ahub, host_device_type_ams, 0x1234u};
    const stu_t stus[] = {STU_ERROR, STU_NONE, STU_HEARTBEAT, STU_PACKET};
    for (unsigned t = 0; t < 4u; t++)
        for (unsigned b = 0; b < 4u; b++)
            for (unsigned a = 0; a < 4u; a++)
                for (unsigned w = 0; w < 2u; w++)
                {
                    uint16_t host = types[t];
                    const int error = main_pass(&host, stus[b], stus[a], w != 0u);

                    const bool b_frame = stus[b] == STU_HEARTBEAT || stus[b] == STU_PACKET;
                    const bool a_frame = stus[a] == STU_HEARTBEAT || stus[a] == STU_PACKET;
                    const uint16_t want = b_frame ? host_device_type_ams : a_frame ? host_device_type_ahub : types[t];
                    TEST_ASSERT_EQUAL_HEX16(want, host);

                    bool offline = true;
                    if (want == host_device_type_ams) offline = stus[b] == STU_ERROR;
                    else if (want == host_device_type_ahub) offline = stus[a] == STU_ERROR;
                    TEST_ASSERT_EQUAL_INT(offline ? -1 : 0, error);
                    TEST_ASSERT_EQUAL_HEX32((offline || w) ? LED_RED : LED_WHITE, sys_led);
                }
}

// ---- bambubus_run()'s reports ----

static void test_a_packet_before_the_first_heartbeat_is_an_error_and_stays_offline(void)
{
    const uint32_t t0 = 1000u * TPMS;
    bambubus_model_t bb;
    bambubus_model_init(&bb);
    uint16_t host = host_device_type_none;

    // Boot: a discovery the BMCU answers, but no heartbeat yet: bambubus_run() reports error.
    TEST_ASSERT_EQUAL(STU_ERROR, bambubus_pass(&bb, true, false, t0));
    TEST_ASSERT_TRUE(offline_pass(&host, STU_ERROR, STU_NONE));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_none, host);
}

static void test_first_heartbeat_on_a_packet_pass_goes_online_at_once(void)
{
    const uint32_t t0 = 1000u * TPMS;
    bambubus_model_t bb;
    bambubus_model_init(&bb);
    uint16_t host = host_device_type_none;

    TEST_ASSERT_TRUE(offline_pass(&host, bambubus_pass(&bb, true, false, t0), STU_NONE));

    // The first heartbeat is flagged on a pass that also handled a packet: the link is alive and the
    // packet's type is reported, which sets the host (the fork waited for the heartbeat report).
    const stu_t r1 = bambubus_pass(&bb, true, true, t0 + 10u);
    TEST_ASSERT_EQUAL(STU_PACKET, r1);
    TEST_ASSERT_FALSE(offline_pass(&host, r1, STU_NONE));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);

    // The heartbeat itself is reported on the first pass without a packet, once.
    TEST_ASSERT_EQUAL(STU_PACKET, bambubus_pass(&bb, true, false, t0 + 20u));
    TEST_ASSERT_EQUAL(STU_HEARTBEAT, bambubus_pass(&bb, false, false, t0 + 30u));
    TEST_ASSERT_EQUAL(STU_NONE, bambubus_pass(&bb, false, false, t0 + 40u));
}

static void test_heartbeats_on_packet_passes_keep_the_host_online(void)
{
    const uint32_t t0 = 5000u;
    bambubus_model_t bb;
    bambubus_model_init(&bb);
    uint16_t host = host_device_type_none;

    TEST_ASSERT_FALSE(offline_pass(&host, bambubus_pass(&bb, false, true, t0), STU_NONE));

    // Every later heartbeat lands on a packet pass: the link stays alive, which is what counts.
    uint32_t now = t0;
    for (int i = 0; i < 20; i++)
    {
        now += 300u * TPMS;
        const stu_t r = bambubus_pass(&bb, true, true, now);
        TEST_ASSERT_EQUAL(STU_PACKET, r);
        TEST_ASSERT_FALSE(offline_pass(&host, r, STU_NONE));
    }
}

static void test_a_bambubus_frame_moves_an_ahub_host_on_the_same_pass(void)
{
    const uint32_t t0 = 5000u;
    bambubus_model_t bb;
    bambubus_model_init(&bb);
    uint16_t host = host_device_type_ahub;

    TEST_ASSERT_FALSE(offline_pass(&host, bambubus_pass(&bb, true, true, t0), STU_NONE));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);
}

// ---- main-loop models ----

// A BambuBus printer (AHUB never seen): heartbeats every 300 ms, a packet on three passes out of
// four, for two minutes across a tick wrap; then silence for three wraps (~12 min). Online from the
// pass of the first heartbeat, offline from 1 s after the last heartbeat for good (the 119 s
// flip-flop of the old deadlines must not come back).
static void test_bambubus_loop_goes_offline_once_and_stays_offline(void)
{
    const uint32_t step = 10u * TPMS;
    const uint32_t hb_period = 300u * TPMS;
    uint64_t now = WRAP - 60000ull * TPMS;
    const uint64_t hb_stop = now + 120000ull * TPMS;
    uint64_t next_hb = now + 5u * step; // a few passes of boot first
    uint64_t last_hb = 0u;
    bool hb_seen = false;

    bambubus_model_t bb;
    bambubus_model_init(&bb);
    bus_link_t ahub;
    bus_link_init(&ahub);
    uint16_t host = host_device_type_none;
    reset_counts();

    unsigned online_passes = 0u;
    unsigned went_offline = 0u;
    bool prev_offline = true;
    unsigned pass = 0u;

    for (; now < hb_stop + 3u * WRAP; now += step, pass++)
    {
        const bool hb = (now < hb_stop) && (now >= next_hb);
        if (hb)
        {
            last_hb = now;
            hb_seen = true;
            next_hb += hb_period;
        }
        const bool packet = (pass % 4u) != 0u;

        const stu_t b = bambubus_pass(&bb, packet, hb, (uint32_t)now);
        const bool offline = offline_pass(&host, b, ahub_pass(&ahub, false, false, (uint32_t)now));

        if (hb_seen && now - last_hb <= TIMEOUT)
            TEST_ASSERT_FALSE(offline);
        else
            TEST_ASSERT_TRUE(offline);

        TEST_ASSERT_EQUAL_INT(offline ? -1 : 0, motion_error);
        TEST_ASSERT_EQUAL_HEX32(offline ? LED_RED : LED_WHITE, sys_led);
        if (!offline) online_passes++;
        if (offline && !prev_offline) went_offline++;
        prev_offline = offline;
    }

    TEST_ASSERT_TRUE(hb_seen);
    TEST_ASSERT_EQUAL_UINT(1u, went_offline);
    TEST_ASSERT_TRUE(online_passes > 11000u); // ~12000 passes of the two minutes
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);
    TEST_ASSERT_EQUAL_UINT(0u, restore_drops);
}

// An AHUB host: its heartbeats every 300 ms and a query on other passes for two minutes while
// BambuBus, never seen, reports error on every pass; then silence for a wrap. Online from the first
// AHUB frame, offline from 1 s after the last heartbeat and for good.
static void test_ahub_loop_goes_offline_once_and_stays_offline(void)
{
    const uint32_t step = 10u * TPMS;
    const uint32_t hb_period = 300u * TPMS;
    uint64_t now = WRAP - 60000ull * TPMS;
    const uint64_t hb_stop = now + 120000ull * TPMS;
    uint64_t next_hb = now + 5u * step;
    uint64_t last_hb = 0u;
    bool hb_seen = false;

    bambubus_model_t bb;
    bambubus_model_init(&bb);
    bus_link_t ahub;
    bus_link_init(&ahub);
    uint16_t host = host_device_type_none;
    reset_counts();

    unsigned went_offline = 0u;
    bool prev_offline = true;
    unsigned pass = 0u;

    for (; now < hb_stop + WRAP; now += step, pass++)
    {
        const bool hb = (now < hb_stop) && (now >= next_hb);
        if (hb)
        {
            last_hb = now;
            hb_seen = true;
            next_hb += hb_period;
        }
        const bool query = hb_seen && !hb && now < hb_stop && (pass % 7u) == 0u;

        const stu_t b = bambubus_pass(&bb, false, false, (uint32_t)now);
        TEST_ASSERT_EQUAL(STU_ERROR, b);
        const bool offline = offline_pass(&host, b, ahub_pass(&ahub, hb, query, (uint32_t)now));

        if (hb_seen && now - last_hb <= TIMEOUT)
            TEST_ASSERT_FALSE(offline);
        else
            TEST_ASSERT_TRUE(offline);

        if (offline && !prev_offline) went_offline++;
        prev_offline = offline;
    }

    TEST_ASSERT_EQUAL_UINT(1u, went_offline);
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ahub, host);
    TEST_ASSERT_TRUE(restore_drops > 300u); // every AHUB heartbeat
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_boot_before_any_packet_stays_offline);
    RUN_TEST(test_first_bambubus_packet_or_heartbeat_sets_the_host_and_goes_online);
    RUN_TEST(test_bambubus_loss_with_the_ahub_never_seen_sets_offline);
    RUN_TEST(test_ahub_link_does_not_decide_for_a_bambubus_host);
    RUN_TEST(test_an_ahub_frame_moves_a_silent_bambubus_host_to_the_ahub);
    RUN_TEST(test_first_ahub_frame_goes_online_with_bambubus_never_seen);
    RUN_TEST(test_bambubus_wins_a_pass_that_carries_both);
    RUN_TEST(test_arrange_wait_shows_red_while_the_motors_run);
    RUN_TEST(test_only_an_ahub_heartbeat_drops_the_boot_restore);
    RUN_TEST(test_every_host_type_and_pair_of_reports);
    RUN_TEST(test_a_packet_before_the_first_heartbeat_is_an_error_and_stays_offline);
    RUN_TEST(test_first_heartbeat_on_a_packet_pass_goes_online_at_once);
    RUN_TEST(test_heartbeats_on_packet_passes_keep_the_host_online);
    RUN_TEST(test_a_bambubus_frame_moves_an_ahub_host_on_the_same_pass);
    RUN_TEST(test_bambubus_loop_goes_offline_once_and_stays_offline);
    RUN_TEST(test_ahub_loop_goes_offline_once_and_stays_offline);
    return UNITY_END();
}
