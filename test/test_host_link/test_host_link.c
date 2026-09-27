// Host tests for src/host_link.h: the protocol the host speaks and whether main() is offline, which
// decides whether Motion_control_run() gets error = -1 (motors stopped) and whether pending NVM
// writes may run. Before any heartbeat the BMCU must stay offline; a lost BambuBus link must set
// offline although the AHUB link, never seen, reports nothing; an AHUB heartbeat moves the decision
// to the AHUB link; and bambubus_run() reports a heartbeat only on a pass that handled no packet.

#include <stdbool.h>
#include <stdint.h>
#include <unity.h>

#include "bus_link.h"
#include "host_link.h"

// SysTick runs at HCLK/8 = 18 MHz.
#define TPMS    18000u
// ---- adapted from bambu_bus_ams.cpp: the bus_link_poll() timeout, ms_to_ticks32(1000u) (ahub_bus.cpp too) ----
#define TIMEOUT (1000u * TPMS)
#define WRAP    4294967296ull // 2^32

static const host_link_report_t R_NONE = {false, false};
static const host_link_report_t R_HB   = {true, false};
static const host_link_report_t R_ERR  = {false, true};

void setUp(void) {}
void tearDown(void) {}

// ---- adapted from main.cpp: the main loop's use of host_link_offline() (the NVM writes only counted) ----
// One pass: the error handed to Motion_control_run(), and ams_nvm_save_run() only while online.
static int motion_error;
static unsigned nvm_runs;

static bool main_pass(uint16_t *host_type, host_link_report_t bambubus, host_link_report_t ahub)
{
    const bool offline = host_link_offline(host_type, bambubus, ahub);

    int error = 0;
    if (!offline)
        nvm_runs++;
    else
        error = -1;

    motion_error = error;
    return offline;
}

// ---- adapted from bambu_bus_ams.cpp: bambubus_run()'s status after the packet (error, hb_unreported) ----
// packet: this pass handled a frame (its type is what bambubus_run() returns); hb: the RX IRQ
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

static host_link_report_t bambubus_pass(bambubus_model_t *m, bool packet, bool hb, uint32_t now)
{
    if (hb)
    {
        bus_link_heartbeat(&m->link, now);
        m->hb_unreported = true;
    }

    host_link_report_t r = R_NONE;
    if (bus_link_poll(&m->link, now, TIMEOUT) != BUS_LINK_ALIVE)
        r.error = true;
    else if (!packet && m->hb_unreported)
    {
        m->hb_unreported = false;
        r.heartbeat = true;
    }
    return r;
}

// ---- adapted from ahub_bus.cpp: ahubus_run()'s status (a heartbeat packet feeds the link, error once lost) ----
static host_link_report_t ahub_pass(bus_link_t *l, bool hb_packet, uint32_t now)
{
    if (hb_packet) bus_link_heartbeat(l, now);

    host_link_report_t r = R_NONE;
    r.heartbeat = hb_packet;
    if (bus_link_poll(l, now, TIMEOUT) == BUS_LINK_LOST)
    {
        r.heartbeat = false;
        r.error = true;
    }
    return r;
}

// ---- adapted from main.cpp: the main loop's host type and offline before host_link.h (at 4933421) ----
typedef enum
{
    STU_ERROR,
    STU_NONE,
    STU_HEARTBEAT,
    STU_PACKET, // any other package type
} stu_t;

static bool offline_before(uint16_t *host_type, stu_t bambubus_stu, stu_t ahub_stu)
{
    if (bambubus_stu == STU_HEARTBEAT)
        *host_type = host_device_type_ams;

    if (ahub_stu == STU_HEARTBEAT)
        *host_type = host_device_type_ahub;

    bool offline = true;
    if (*host_type == host_device_type_ams)
        offline = (bambubus_stu == STU_ERROR);
    else if (*host_type == host_device_type_ahub)
        offline = (ahub_stu == STU_ERROR);
    return offline;
}

// main()'s conversion of a package type into a report.
static host_link_report_t report_of(stu_t stu)
{
    const host_link_report_t r = {stu == STU_HEARTBEAT, stu == STU_ERROR};
    return r;
}

static void reset_counts(void)
{
    motion_error = 0;
    nvm_runs = 0u;
}

// ---- host_link_offline() ----

static void test_same_host_type_and_offline_as_main_before_host_link_h(void)
{
    // Every host type main() can hold (and one it cannot), every pair of package types.
    const uint16_t types[] = {host_device_type_none, host_device_type_ahub, host_device_type_ams, 0x1234u};
    const stu_t stus[] = {STU_ERROR, STU_NONE, STU_HEARTBEAT, STU_PACKET};
    for (unsigned t = 0; t < 4u; t++)
        for (unsigned b = 0; b < 4u; b++)
            for (unsigned a = 0; a < 4u; a++)
            {
                uint16_t before = types[t];
                uint16_t now = types[t];
                const bool off_before = offline_before(&before, stus[b], stus[a]);
                TEST_ASSERT_EQUAL(off_before, host_link_offline(&now, report_of(stus[b]), report_of(stus[a])));
                TEST_ASSERT_EQUAL_HEX16(before, now);
            }
}

static void test_boot_before_any_heartbeat_stays_offline(void)
{
    // BambuBus reports error from boot until its first heartbeat, the AHUB nothing: neither decides.
    const host_link_report_t reports[] = {R_NONE, R_ERR};
    for (unsigned b = 0; b < 2u; b++)
        for (unsigned a = 0; a < 2u; a++)
        {
            uint16_t host = host_device_type_none;
            reset_counts();
            TEST_ASSERT_TRUE(main_pass(&host, reports[b], reports[a]));
            TEST_ASSERT_EQUAL_HEX16(host_device_type_none, host);
            TEST_ASSERT_EQUAL_INT(-1, motion_error);
            TEST_ASSERT_EQUAL_UINT(0u, nvm_runs);
        }
}

static void test_first_bambubus_heartbeat_sets_the_host_and_goes_online(void)
{
    uint16_t host = host_device_type_none;
    reset_counts();
    TEST_ASSERT_FALSE(main_pass(&host, R_HB, R_NONE));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);
    TEST_ASSERT_EQUAL_INT(0, motion_error);

    // Passes that report no heartbeat (packets, or nothing) stay online while the link is up.
    for (int i = 0; i < 100; i++)
        TEST_ASSERT_FALSE(main_pass(&host, R_NONE, R_NONE));
    TEST_ASSERT_EQUAL_UINT(101u, nvm_runs);
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);
}

static void test_bambubus_loss_with_the_ahub_never_seen_sets_offline(void)
{
    uint16_t host = host_device_type_none;
    reset_counts();
    TEST_ASSERT_FALSE(main_pass(&host, R_HB, R_NONE));

    // The AHUB link, never seen, reports nothing: it must not mask the lost BambuBus link.
    for (int i = 0; i < 100; i++)
    {
        TEST_ASSERT_TRUE(main_pass(&host, R_ERR, R_NONE));
        TEST_ASSERT_EQUAL_INT(-1, motion_error);
    }
    TEST_ASSERT_EQUAL_UINT(1u, nvm_runs);
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);

    // The next heartbeat brings it back.
    TEST_ASSERT_FALSE(main_pass(&host, R_HB, R_NONE));
    TEST_ASSERT_EQUAL_INT(0, motion_error);
}

static void test_ahub_link_does_not_decide_for_a_bambubus_host(void)
{
    // An AHUB link that was up once and is lost now (the host was AHUB, then BambuBus).
    uint16_t host = host_device_type_ams;
    TEST_ASSERT_FALSE(host_link_offline(&host, R_NONE, R_ERR));
    TEST_ASSERT_FALSE(host_link_offline(&host, R_HB, R_ERR));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);
}

static void test_ahub_heartbeat_flips_the_host_type(void)
{
    uint16_t host = host_device_type_ams;
    TEST_ASSERT_FALSE(host_link_offline(&host, R_NONE, R_HB));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ahub, host);

    // BambuBus goes lost (no BambuBus heartbeats any more): only the AHUB link decides now.
    TEST_ASSERT_FALSE(host_link_offline(&host, R_ERR, R_NONE));
    TEST_ASSERT_TRUE(host_link_offline(&host, R_NONE, R_ERR));
    TEST_ASSERT_TRUE(host_link_offline(&host, R_ERR, R_ERR));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ahub, host);

    // A BambuBus heartbeat flips it back, whatever the AHUB link reports.
    TEST_ASSERT_FALSE(host_link_offline(&host, R_HB, R_ERR));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);
}

static void test_first_ahub_heartbeat_goes_online_with_bambubus_never_seen(void)
{
    uint16_t host = host_device_type_none;
    TEST_ASSERT_FALSE(host_link_offline(&host, R_ERR, R_HB));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ahub, host);
    TEST_ASSERT_FALSE(host_link_offline(&host, R_ERR, R_NONE));
}

static void test_both_heartbeats_on_one_pass_latch_the_ahub(void)
{
    // main() set the BambuBus type first and the AHUB type after it.
    const uint16_t starts[] = {host_device_type_none, host_device_type_ams, host_device_type_ahub};
    for (unsigned i = 0; i < 3u; i++)
    {
        uint16_t host = starts[i];
        TEST_ASSERT_FALSE(host_link_offline(&host, R_HB, R_HB));
        TEST_ASSERT_EQUAL_HEX16(host_device_type_ahub, host);
    }
}

// ---- bambubus_run()'s heartbeat report (hb_unreported) ----

static void test_first_heartbeat_on_a_packet_pass_goes_online_on_the_next_pass_without_one(void)
{
    const uint32_t t0 = 1000u * TPMS;
    bambubus_model_t bb;
    bambubus_model_init(&bb);
    uint16_t host = host_device_type_none;
    reset_counts();

    // Boot: no heartbeat yet, a packet on the bus (a discovery the BMCU answers): offline.
    TEST_ASSERT_TRUE(main_pass(&host, bambubus_pass(&bb, true, false, t0), R_NONE));

    // The first heartbeat is flagged on a pass that also handled a packet: the link is alive, but
    // the heartbeat is not reported yet, so the host type is still unknown and the BMCU offline.
    const host_link_report_t r1 = bambubus_pass(&bb, true, true, t0 + 10u);
    TEST_ASSERT_FALSE(r1.heartbeat);
    TEST_ASSERT_FALSE(r1.error);
    TEST_ASSERT_TRUE(main_pass(&host, r1, R_NONE));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_none, host);

    // More packet passes: still waiting.
    TEST_ASSERT_TRUE(main_pass(&host, bambubus_pass(&bb, true, false, t0 + 20u), R_NONE));
    TEST_ASSERT_TRUE(main_pass(&host, bambubus_pass(&bb, true, false, t0 + 30u), R_NONE));
    TEST_ASSERT_EQUAL_UINT(0u, nvm_runs);

    // The first pass without a packet reports it: host BambuBus, online.
    const host_link_report_t r2 = bambubus_pass(&bb, false, false, t0 + 40u);
    TEST_ASSERT_TRUE(r2.heartbeat);
    TEST_ASSERT_FALSE(main_pass(&host, r2, R_NONE));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);
    TEST_ASSERT_EQUAL_INT(0, motion_error);

    // Reported once.
    TEST_ASSERT_FALSE(bambubus_pass(&bb, false, false, t0 + 50u).heartbeat);
}

static void test_heartbeat_on_a_packet_pass_keeps_a_known_host_online(void)
{
    const uint32_t t0 = 5000u;
    bambubus_model_t bb;
    bambubus_model_init(&bb);
    uint16_t host = host_device_type_none;
    reset_counts();

    TEST_ASSERT_FALSE(main_pass(&host, bambubus_pass(&bb, false, true, t0), R_NONE));

    // Every later heartbeat lands on a packet pass: the link stays alive, which is what counts.
    uint32_t now = t0;
    for (int i = 0; i < 20; i++)
    {
        now += 300u * TPMS;
        const host_link_report_t r = bambubus_pass(&bb, true, true, now);
        TEST_ASSERT_FALSE(r.heartbeat);
        TEST_ASSERT_FALSE(main_pass(&host, r, R_NONE));
    }
    TEST_ASSERT_EQUAL_UINT(21u, nvm_runs);
}

static void test_heartbeat_on_an_ahub_host_is_latched_on_the_pass_without_a_packet(void)
{
    const uint32_t t0 = 5000u;
    bambubus_model_t bb;
    bambubus_model_init(&bb);
    uint16_t host = host_device_type_ahub;

    TEST_ASSERT_FALSE(host_link_offline(&host, bambubus_pass(&bb, true, true, t0), R_NONE));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ahub, host);
    TEST_ASSERT_FALSE(host_link_offline(&host, bambubus_pass(&bb, false, false, t0 + 1u), R_NONE));
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);
}

// ---- main-loop models ----

// A BambuBus printer (AHUB never seen): heartbeats every 300 ms, a packet on three passes out of
// four, for two minutes across a tick wrap; then silence for three wraps (~12 min). Online from the
// first packet-free pass after the first heartbeat, offline from 1 s after the last heartbeat
// (the 119 s flip-flop of the old deadlines must not come back), NVM writes only while online.
static void test_bambubus_loop_goes_offline_once_and_stays_offline(void)
{
    const uint32_t step = 10u * TPMS;
    const uint32_t hb_period = 300u * TPMS;
    uint64_t now = WRAP - 60000ull * TPMS;
    const uint64_t hb_stop = now + 120000ull * TPMS;
    uint64_t next_hb = now + 5u * step; // a few passes of boot first
    uint64_t last_hb = 0u;
    bool hb_seen = false;
    bool reported = false;

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

        const host_link_report_t b = bambubus_pass(&bb, packet, hb, (uint32_t)now);
        if (b.heartbeat) reported = true;
        const bool offline = main_pass(&host, b, ahub_pass(&ahub, false, (uint32_t)now));

        if (!reported)
            TEST_ASSERT_TRUE(offline);
        else if (hb_seen && now - last_hb <= TIMEOUT)
            TEST_ASSERT_FALSE(offline);
        else
            TEST_ASSERT_TRUE(offline);

        TEST_ASSERT_EQUAL_INT(offline ? -1 : 0, motion_error);
        if (!offline) online_passes++;
        if (offline && !prev_offline) went_offline++;
        prev_offline = offline;
    }

    TEST_ASSERT_TRUE(reported);
    TEST_ASSERT_EQUAL_UINT(1u, went_offline);
    TEST_ASSERT_EQUAL_UINT(online_passes, nvm_runs);
    TEST_ASSERT_TRUE(online_passes > 11000u); // ~12000 passes of the two minutes
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ams, host);
}

// An AHUB host: its heartbeats every 300 ms for two minutes while BambuBus, never seen, reports
// error on every pass; then silence for a wrap. Online from the first AHUB heartbeat, offline from
// 1 s after the last one and for good.
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

    for (; now < hb_stop + WRAP; now += step)
    {
        const bool hb = (now < hb_stop) && (now >= next_hb);
        if (hb)
        {
            last_hb = now;
            hb_seen = true;
            next_hb += hb_period;
        }

        const host_link_report_t b = bambubus_pass(&bb, false, false, (uint32_t)now);
        TEST_ASSERT_TRUE(b.error);
        const bool offline = main_pass(&host, b, ahub_pass(&ahub, hb, (uint32_t)now));

        if (hb_seen && now - last_hb <= TIMEOUT)
            TEST_ASSERT_FALSE(offline);
        else
            TEST_ASSERT_TRUE(offline);

        if (offline && !prev_offline) went_offline++;
        prev_offline = offline;
    }

    TEST_ASSERT_EQUAL_UINT(1u, went_offline);
    TEST_ASSERT_EQUAL_HEX16(host_device_type_ahub, host);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_same_host_type_and_offline_as_main_before_host_link_h);
    RUN_TEST(test_boot_before_any_heartbeat_stays_offline);
    RUN_TEST(test_first_bambubus_heartbeat_sets_the_host_and_goes_online);
    RUN_TEST(test_bambubus_loss_with_the_ahub_never_seen_sets_offline);
    RUN_TEST(test_ahub_link_does_not_decide_for_a_bambubus_host);
    RUN_TEST(test_ahub_heartbeat_flips_the_host_type);
    RUN_TEST(test_first_ahub_heartbeat_goes_online_with_bambubus_never_seen);
    RUN_TEST(test_both_heartbeats_on_one_pass_latch_the_ahub);
    RUN_TEST(test_first_heartbeat_on_a_packet_pass_goes_online_on_the_next_pass_without_one);
    RUN_TEST(test_heartbeat_on_a_packet_pass_keeps_a_known_host_online);
    RUN_TEST(test_heartbeat_on_an_ahub_host_is_latched_on_the_pass_without_a_packet);
    RUN_TEST(test_bambubus_loop_goes_offline_once_and_stays_offline);
    RUN_TEST(test_ahub_loop_goes_offline_once_and_stays_offline);
    return UNITY_END();
}
