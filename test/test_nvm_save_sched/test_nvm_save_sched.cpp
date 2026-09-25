// Host tests for src/nvm_save_sched.h and the bus accessors it relies on (_bus_port_deal::tx_idle,
// rx_idle, last_rx_tick, last_tx_end_tick): main-loop NVM writes must never start while a reply
// is in flight or queued, or while a frame is arriving or waiting, should wait for 1 ms of silence
// in both directions, and filament-info bursts must collapse into one write.

#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "_bus_hardware.h"
#include "crc_bus.h"
#include "nvm_save_sched.h"

// One 9E1 byte at 1.25 Mbaud in 18 MHz SysTick ticks (158.4).
static const uint32_t BYTE_TICKS = 158u;
static const uint32_t US = 18u;
static const uint32_t MS = NVM_TICKS_PER_MS;

static _bus_port_deal port;
static uint32_t tick;
static uint32_t tx_end_at;

static nvm_job state_job;
static nvm_job fil_job[4];
static nvm_wait wait_st;

static int writes_state;
static int writes_fil[4];
static uint32_t last_write_tick;
static uint32_t last_write_rx_gap;
static uint32_t last_write_tx_gap;
static int last_write_job;
static bool fail_fil[4];

// What the simulated bambubus_run does with a received frame: a handler, and a reply length.
static void (*on_frame)(void);
static uint16_t reply_len;

void bambubus_heartbeat_seen_fast(void) {}

// Stand-in for bus_uart1_dma_send: DE on, TX ends len byte times later (then the TC ISR runs).
static void fake_dma_send(uint8_t *, uint16_t len)
{
    port.idle = false;
    tx_end_at = tick + (uint32_t)len * BYTE_TICKS;
}

void setUp(void)
{
    port.init(fake_dma_send);
    memset(&state_job, 0, sizeof(state_job));
    memset(fil_job, 0, sizeof(fil_job));
    memset(&wait_st, 0, sizeof(wait_st));
    memset(writes_fil, 0, sizeof(writes_fil));
    memset(fail_fil, 0, sizeof(fail_fil));
    writes_state = 0;
    last_write_tick = 0u;
    last_write_rx_gap = 0u;
    last_write_tx_gap = 0u;
    last_write_job = NVM_JOB_NONE;
    on_frame = nullptr;
    reply_len = 0;
    tick = 0x10000000u;
    tx_end_at = 0u;
    // Some earlier traffic, then the line has been quiet for a while.
    port.rx_byte(0xFF, tick, false);
    port.tx_done(tick);
    tick += 100u * MS;
}

void tearDown(void) {}

// The TC interrupt, if our TX has finished by `tick`.
static void tx_isr(void)
{
    if (!port.idle && (int32_t)(tick - tx_end_at) >= 0)
        port.tx_done(tx_end_at);
}

// ---- adapted from main.cpp: ams_nvm_save_run (one tick for the three samples; the writes only counted) ----
// ams_nvm_save_run (src/main.cpp): sample the bus, pick a job, run it.
static void nvm_pass(void)
{
    const bool rx_idle = port.rx_idle(tick);
    const bool tx_idle = port.tx_idle();
    const nvm_bus b = nvm_bus_from_samples(rx_idle, tx_idle, tick, port.last_rx_tick(),
                                           port.last_tx_end_tick());

    const int job = nvm_pick_job(&state_job, fil_job, &wait_st, &b, tick);
    if (job == NVM_JOB_NONE) return;

    // Never with a reply in flight or queued, a frame waiting, or a frame on the line.
    TEST_ASSERT_TRUE(port.idle);
    TEST_ASSERT_EQUAL_INT(0, port.send_data_len);
    TEST_ASSERT_EQUAL_INT(0, port.recv_data_len);
    TEST_ASSERT_TRUE(b.rx_gap > BUS_RX_RESYNC_GAP_TICKS);

    last_write_tick = tick;
    last_write_rx_gap = b.rx_gap;
    last_write_tx_gap = b.tx_gap;
    last_write_job = job;
    if (job == NVM_JOB_STATE)
    {
        writes_state++;
        nvm_job_result(&state_job, true, tick);
    }
    else
    {
        writes_fil[job]++;
        nvm_job_result(&fil_job[job], !fail_fil[job], tick);
    }
}

// ---- adapted from main.cpp: the main loop's order of bambubus_run, send_package and ams_nvm_save_run ----
// One main-loop pass: bambubus_run consumes a received frame and may queue a reply,
// send_package starts it, then the NVM step.
static void main_pass(void)
{
    tx_isr();
    if (port.recv_data_len != 0)
    {
        port.recv_data_len = 0;
        port.bus_package_type = _bus_data_type::none;
        if (on_frame) on_frame();
        if (reply_len) port.send_data_len = reply_len;
    }
    port.send_package();
    nvm_pass();
}

// Advance the clock by `ticks`, one main-loop pass every `pass_ticks`.
static void run_for(uint32_t ticks, uint32_t pass_ticks)
{
    for (uint32_t t = 0; t < ticks; t += pass_ticks)
    {
        tick += pass_ticks;
        main_pass();
    }
}

// Short-header frame: 3D, flags (bit 7 set), total length, CRC8, payload, CRC16 (little endian).
static int make_short(uint8_t *out, const uint8_t *payload, int n)
{
    const int len = 4 + n + 2;
    out[0] = 0x3D;
    out[1] = 0xC0;
    out[2] = (uint8_t)len;
    out[3] = bus_crc8(out, 3);
    memcpy(out + 4, payload, (size_t)n);
    const uint16_t crc = bus_crc16(out, (uint32_t)(len - 2));
    out[len - 2] = (uint8_t)(crc & 0xFFu);
    out[len - 1] = (uint8_t)(crc >> 8);
    return len;
}

// The printer sends a 12-byte frame, back to back, with a main-loop pass after every byte.
static void printer_frame(void)
{
    uint8_t f[16];
    const uint8_t payload[6] = {0x03, 0x00, 0x07, 0x00, 0x7F, 0x02};
    const int n = make_short(f, payload, (int)sizeof(payload));
    for (int i = 0; i < n; i++)
    {
        tick += BYTE_TICKS;
        port.rx_byte(f[i], tick, false);
        main_pass();
    }
}

static void mark_loaded(void) { nvm_job_changed(&state_job, tick); }
static void mark_slot1(void) { nvm_job_changed(&fil_job[1], tick); }

// ---- bus accessors ----

static void test_tx_idle_needs_no_reply_in_flight_or_queued(void)
{
    TEST_ASSERT_TRUE(port.tx_idle());
    port.send_data_len = 8;
    TEST_ASSERT_FALSE(port.tx_idle());
    port.send_package();
    TEST_ASSERT_EQUAL_INT(0, port.send_data_len);
    TEST_ASSERT_FALSE(port.tx_idle());
    tick = tx_end_at;
    tx_isr();
    TEST_ASSERT_TRUE(port.tx_idle());
    TEST_ASSERT_EQUAL_HEX32(tx_end_at, port.last_tx_end_tick());
}

static void test_rx_idle_tracks_frames_arriving_and_waiting(void)
{
    uint8_t f[16];
    const uint8_t payload[6] = {0x03, 0x00, 0x07, 0x00, 0x7F, 0x02};
    const int n = make_short(f, payload, (int)sizeof(payload));

    TEST_ASSERT_TRUE(port.rx_idle(tick));
    for (int i = 0; i < 5; i++)
    {
        tick += BYTE_TICKS;
        port.rx_byte(f[i], tick, false);
    }
    // Mid-frame: busy while the line is live, dead (idle) once quiet past the resync gap.
    TEST_ASSERT_FALSE(port.rx_idle(tick));
    TEST_ASSERT_FALSE(port.rx_idle(tick + BUS_RX_RESYNC_GAP_TICKS));
    TEST_ASSERT_TRUE(port.rx_idle(tick + BUS_RX_RESYNC_GAP_TICKS + 1u));

    // A complete frame waiting for the main loop is busy however long it waits.
    tick += 10u * MS;
    for (int i = 0; i < n; i++)
    {
        tick += BYTE_TICKS;
        port.rx_byte(f[i], tick, false);
    }
    TEST_ASSERT_EQUAL_INT(n, port.recv_data_len);
    TEST_ASSERT_FALSE(port.rx_idle(tick + 100u * MS));
    port.recv_data_len = 0;
    TEST_ASSERT_TRUE(port.rx_idle(tick));
}

static void test_rx_idle_is_false_during_a_heartbeat_skip(void)
{
    uint8_t hb[32];
    uint8_t payload[12];
    memset(payload, 0x55, sizeof(payload));
    payload[0] = 0x20;
    const int n = 4 + (int)sizeof(payload) + 2;
    hb[0] = 0x3D;
    hb[1] = 0xC5;
    hb[2] = (uint8_t)n;
    hb[3] = bus_crc8(hb, 3);
    memcpy(hb + 4, payload, sizeof(payload));
    for (int i = 0; i < 8; i++)
    {
        tick += BYTE_TICKS;
        port.rx_byte(hb[i], tick, false);
    }
    TEST_ASSERT_FALSE(port.rx_idle(tick));
    TEST_ASSERT_TRUE(port.rx_idle(tick + BUS_RX_RESYNC_GAP_TICKS + 1u));
}

// An RX byte or TX end the ISR stamps just after the main loop read `now` is happening right now: a
// zero gap, not a wrap to ~2^32 that would pass the quiet check. Genuinely old stamps stay long gaps.
static void test_stamp_just_after_now_is_a_zero_gap(void)
{
    const uint32_t now = 0xFFFFFFF0u; // next to the SysTick wrap as well
    TEST_ASSERT_EQUAL_UINT32(0u, nvm_gap(now, now));
    TEST_ASSERT_EQUAL_UINT32(0u, nvm_gap(now, now + 3u));
    TEST_ASSERT_EQUAL_UINT32(0u, nvm_gap(now, now + 40u)); // wraps past 0
    TEST_ASSERT_EQUAL_UINT32(0u, nvm_gap(now, now + NVM_BUS_QUIET_TICKS));
    TEST_ASSERT_EQUAL_UINT32(5u, nvm_gap(now, now - 5u));
    TEST_ASSERT_EQUAL_UINT32(NVM_BUS_QUIET_TICKS, nvm_gap(now, now - NVM_BUS_QUIET_TICKS));
    TEST_ASSERT_EQUAL_UINT32(0x90000000u, nvm_gap(now, now - 0x90000000u)); // older than 2^31

    const nvm_bus b = nvm_bus_from_samples(true, true, now, now + 2u, now - 5u * NVM_BUS_QUIET_TICKS);
    TEST_ASSERT_EQUAL_UINT32(0u, b.rx_gap);
    TEST_ASSERT_FALSE(nvm_bus_window(&b, false));
    TEST_ASSERT_FALSE(nvm_bus_window(&b, true));
}

// Same race inside rx_idle(): mid-frame, a byte stamped just after `now` means the frame is live.
static void test_rx_idle_with_a_byte_stamped_after_now(void)
{
    uint8_t f[16];
    const uint8_t payload[6] = {0x03, 0x00, 0x07, 0x00, 0x7F, 0x02};
    make_short(f, payload, (int)sizeof(payload));

    const uint32_t now = tick + 10u * MS;
    for (int i = 0; i < 4; i++)
        port.rx_byte(f[i], now + (uint32_t)i + 1u, false); // the ISR ran after `now` was read
    TEST_ASSERT_FALSE(port.rx_idle(now));
}

// ---- quiet window ----

static nvm_bus quiet_bus(void)
{
    nvm_bus b;
    b.tx_idle = true;
    b.rx_idle = true;
    b.rx_gap = NVM_BUS_QUIET_TICKS;
    b.tx_gap = NVM_BUS_QUIET_TICKS;
    return b;
}

static void test_window_needs_1ms_of_silence_both_ways(void)
{
    nvm_bus b = quiet_bus();
    TEST_ASSERT_TRUE(nvm_bus_window(&b, false));

    b.rx_gap = NVM_BUS_QUIET_TICKS - 1u;
    TEST_ASSERT_FALSE(nvm_bus_window(&b, false));
    b = quiet_bus();
    b.tx_gap = NVM_BUS_QUIET_TICKS - 1u; // our TX was not echoed to RX
    TEST_ASSERT_FALSE(nvm_bus_window(&b, false));

    b = quiet_bus();
    b.tx_idle = false;
    TEST_ASSERT_FALSE(nvm_bus_window(&b, false));
    TEST_ASSERT_FALSE(nvm_bus_window(&b, true));
    b = quiet_bus();
    b.rx_idle = false;
    TEST_ASSERT_FALSE(nvm_bus_window(&b, false));
    TEST_ASSERT_FALSE(nvm_bus_window(&b, true));
}

static void test_long_wait_settles_for_no_frame_on_the_line(void)
{
    nvm_bus b = quiet_bus();
    b.rx_gap = BUS_RX_RESYNC_GAP_TICKS + 1u;
    b.tx_gap = BUS_RX_RESYNC_GAP_TICKS + 1u;
    TEST_ASSERT_FALSE(nvm_bus_window(&b, false));
    TEST_ASSERT_TRUE(nvm_bus_window(&b, true));
    b.rx_gap = BUS_RX_RESYNC_GAP_TICKS;
    TEST_ASSERT_FALSE(nvm_bus_window(&b, true));
    b.rx_gap = BUS_RX_RESYNC_GAP_TICKS + 1u;
    b.tx_gap = BUS_RX_RESYNC_GAP_TICKS;
    TEST_ASSERT_FALSE(nvm_bus_window(&b, true));
}

// ---- job timing ----

static bool fil_due(const nvm_job *j, uint32_t now)
{
    return nvm_job_due(j, now, NVM_FIL_SETTLE_TICKS, NVM_FIL_MAX_DELAY_TICKS);
}

static void test_filament_settles_500ms_after_the_last_change(void)
{
    const uint32_t t0 = 0xFFFFFFFFu - 400u * MS; // burst before the SysTick wrap, due check after it
    nvm_job j;
    memset(&j, 0, sizeof(j));

    nvm_job_changed(&j, t0);
    nvm_job_changed(&j, t0 + 100u * MS);
    nvm_job_changed(&j, t0 + 350u * MS);
    // Before the wrap, with the settle deadline already past it: a compare like
    // now >= last + settle would call this due.
    TEST_ASSERT_FALSE(fil_due(&j, t0 + 360u * MS));
    TEST_ASSERT_FALSE(fil_due(&j, t0 + 400u * MS - 1u));
    TEST_ASSERT_FALSE(fil_due(&j, t0 + 850u * MS - 1u));
    TEST_ASSERT_TRUE(fil_due(&j, t0 + 850u * MS));

    nvm_job_result(&j, true, t0 + 850u * MS);
    TEST_ASSERT_FALSE(fil_due(&j, t0 + 10000u * MS));
}

static void test_filament_that_keeps_changing_is_written_after_5s(void)
{
    const uint32_t t0 = 0xFFFFFFFFu - 1000u * MS; // first change before the SysTick wrap
    nvm_job j;
    memset(&j, 0, sizeof(j));

    // A change every 400 ms never lets the slot settle for 500 ms.
    for (uint32_t k = 0; k * 400u < 5000u; k++)
    {
        const uint32_t t = t0 + k * 400u * MS;
        nvm_job_changed(&j, t);
        const uint32_t probe = (k * 400u + 399u < 5000u) ? t + 399u * MS : t0 + 5000u * MS - 1u;
        TEST_ASSERT_FALSE(fil_due(&j, probe));
    }
    TEST_ASSERT_TRUE(fil_due(&j, t0 + 5000u * MS));
}

static void test_failed_write_waits_before_retrying(void)
{
    const uint32_t t0 = 0xFFFFFFFFu - 100u * MS;
    nvm_job j;
    memset(&j, 0, sizeof(j));

    nvm_job_changed(&j, t0);
    TEST_ASSERT_TRUE(nvm_job_due(&j, t0, 0u, 0u));
    nvm_job_result(&j, false, t0);
    TEST_ASSERT_FALSE(nvm_job_due(&j, t0, 0u, 0u));
    TEST_ASSERT_FALSE(nvm_job_due(&j, t0 + NVM_RETRY_TICKS - 1u, 0u, 0u));
    TEST_ASSERT_TRUE(nvm_job_due(&j, t0 + NVM_RETRY_TICKS, 0u, 0u));
    nvm_job_result(&j, true, t0 + NVM_RETRY_TICKS);
    TEST_ASSERT_FALSE(nvm_job_due(&j, t0 + NVM_RETRY_TICKS, 0u, 0u));
}

// ---- main-loop scenarios ----

// The audit scenario: the printer's on_use sets the loaded channel, the BMCU starts its reply in
// the same pass. The old code wrote the STA journal right there, with the reply still on the wire.
static void test_state_write_waits_for_reply_end_plus_1ms(void)
{
    on_frame = mark_loaded;
    reply_len = 60;
    printer_frame();
    TEST_ASSERT_TRUE(state_job.pending);
    TEST_ASSERT_FALSE(port.idle); // reply in flight
    TEST_ASSERT_EQUAL_INT(0, writes_state);

    const uint32_t tx_end = tx_end_at;
    on_frame = nullptr;
    reply_len = 0;
    while (writes_state == 0 && (int32_t)(tick - tx_end) < (int32_t)(10u * MS))
        run_for(10u * US, 10u * US);

    TEST_ASSERT_EQUAL_INT(1, writes_state);
    TEST_ASSERT_TRUE(last_write_tick - tx_end >= NVM_BUS_QUIET_TICKS);
    TEST_ASSERT_TRUE(last_write_tick - tx_end < NVM_BUS_QUIET_TICKS + 10u * US);
}

// Three set_filament_info for slot 1 within 350 ms, each answered with the 8-byte ACK, while the
// printer keeps polling every 20 ms: one write, 500 ms after the last of them, in a quiet window.
static void test_filament_burst_ends_in_one_write_in_a_quiet_window(void)
{
    on_frame = mark_slot1;
    reply_len = 8;
    printer_frame();
    run_for(100u * MS, 100u * US);
    printer_frame();
    run_for(250u * MS, 100u * US);
    printer_frame();
    const uint32_t last_change = fil_job[1].last;
    on_frame = nullptr;
    reply_len = 0;

    // The printer keeps polling every 20 ms (frame + our 8-byte reply) while we wait.
    const uint32_t t_end = last_change + 2000u * MS;
    while ((int32_t)(tick - t_end) < 0)
    {
        reply_len = 8;
        printer_frame();
        reply_len = 0;
        run_for(20u * MS, 100u * US);
    }

    TEST_ASSERT_EQUAL_INT(1, writes_fil[1]);
    TEST_ASSERT_EQUAL_INT(0, writes_state);
    TEST_ASSERT_TRUE(last_write_tick - last_change >= NVM_FIL_SETTLE_TICKS);
    TEST_ASSERT_TRUE(last_write_tick - last_change < NVM_FIL_SETTLE_TICKS + 2u * MS);
    TEST_ASSERT_TRUE(last_write_rx_gap >= NVM_BUS_QUIET_TICKS);
    TEST_ASSERT_TRUE(last_write_tx_gap >= NVM_BUS_QUIET_TICKS);
}

// The printer polls with less than 1 ms of silence between exchanges: after 1 s the write goes
// into a gap that is only longer than the resync gap, still never during our TX or a frame.
static void test_busy_bus_falls_back_after_max_wait(void)
{
    on_frame = nullptr;
    reply_len = 0;
    nvm_job_changed(&state_job, tick);

    const uint32_t waited_from = tick;
    for (int cycle = 0; cycle < 3000 && writes_state == 0; cycle++)
    {
        // 12-byte request (~105 us), our 20-byte reply (~176 us), then 400 us of silence.
        reply_len = 20;
        printer_frame();
        reply_len = 0;
        run_for(20u * BYTE_TICKS + 400u * US, 50u * US);
    }

    TEST_ASSERT_EQUAL_INT(1, writes_state);
    TEST_ASSERT_TRUE(last_write_rx_gap < NVM_BUS_QUIET_TICKS);
    const uint32_t waited = last_write_tick - waited_from;
    TEST_ASSERT_TRUE(waited >= NVM_BUS_QUIET_MAX_WAIT_TICKS);
    TEST_ASSERT_TRUE(waited < NVM_BUS_QUIET_MAX_WAIT_TICKS + 10u * MS);
}

// Each job gets its own wait: with two jobs due on a busy bus, the second one still looks for a
// 1 ms window for up to NVM_BUS_QUIET_MAX_WAIT_TICKS after the first went into a short gap.
static void test_each_job_waits_for_its_own_window(void)
{
    nvm_job_changed(&state_job, tick);
    nvm_job_changed(&fil_job[0], tick - NVM_FIL_SETTLE_TICKS); // due at once as well
    uint32_t state_written = 0u;
    for (int cycle = 0; cycle < 6000 && writes_fil[0] == 0; cycle++)
    {
        reply_len = 20;
        printer_frame();
        reply_len = 0;
        run_for(20u * BYTE_TICKS + 400u * US, 50u * US);
        if (writes_state == 1 && state_written == 0u)
            state_written = last_write_tick;
    }

    TEST_ASSERT_EQUAL_INT(1, writes_state);
    TEST_ASSERT_EQUAL_INT(1, writes_fil[0]);
    TEST_ASSERT_TRUE(last_write_tick - state_written >= NVM_BUS_QUIET_MAX_WAIT_TICKS);
}

static void test_one_job_per_pass_state_first(void)
{
    nvm_job_changed(&fil_job[2], tick - NVM_FIL_SETTLE_TICKS);
    nvm_job_changed(&fil_job[0], tick - NVM_FIL_SETTLE_TICKS);
    nvm_job_changed(&state_job, tick);

    nvm_pass();
    TEST_ASSERT_EQUAL_INT(NVM_JOB_STATE, last_write_job);
    tick += 100u * US;
    nvm_pass();
    TEST_ASSERT_EQUAL_INT(0, last_write_job);
    tick += 100u * US;
    nvm_pass();
    TEST_ASSERT_EQUAL_INT(2, last_write_job);
    TEST_ASSERT_EQUAL_INT(1, writes_state);
    TEST_ASSERT_EQUAL_INT(1, writes_fil[0]);
    TEST_ASSERT_EQUAL_INT(1, writes_fil[2]);
}

// A slot whose write keeps failing is retried every NVM_RETRY_TICKS, not on every quiet pass, and
// does not hold back the slots after it.
static void test_failing_slot_backs_off_and_does_not_block_others(void)
{
    fail_fil[0] = true;
    nvm_job_changed(&fil_job[0], tick - NVM_FIL_SETTLE_TICKS);
    nvm_job_changed(&fil_job[3], tick - NVM_FIL_SETTLE_TICKS);

    run_for(NVM_RETRY_TICKS * 2u, 100u * US);
    TEST_ASSERT_EQUAL_INT(1, writes_fil[3]);
    TEST_ASSERT_EQUAL_INT(0, (int)fil_job[3].pending);
    TEST_ASSERT_TRUE(writes_fil[0] >= 2 && writes_fil[0] <= 3);
    TEST_ASSERT_EQUAL_INT(1, (int)fil_job[0].pending);
}

// A new change after a failed write is scheduled by the normal rule, not held back by the retry
// delay: a load/unload right after a failed STA write goes into the next quiet window.
static void test_new_change_after_a_failed_write_is_not_delayed(void)
{
    const uint32_t t0 = 0xFFFFFFFFu - 100u * MS;
    nvm_job j;
    memset(&j, 0, sizeof(j));

    nvm_job_changed(&j, t0);
    nvm_job_result(&j, false, t0);
    TEST_ASSERT_FALSE(nvm_job_due(&j, t0 + 1u * MS, 0u, 0u));
    nvm_job_changed(&j, t0 + 1u * MS);
    TEST_ASSERT_TRUE(nvm_job_due(&j, t0 + 1u * MS, 0u, 0u));
}

// A reply longer than the 1 ms quiet window (300 bytes, about 2.6 ms): the write must wait for the
// end of the reply plus the window, and never start while the reply is on the wire.
static void test_long_reply_is_never_overlapped(void)
{
    on_frame = mark_loaded;
    reply_len = 300;
    printer_frame();
    const uint32_t tx_end = tx_end_at;
    TEST_ASSERT_TRUE(tx_end - tick > NVM_BUS_QUIET_TICKS);
    on_frame = nullptr;
    reply_len = 0;

    while (writes_state == 0 && (int32_t)(tick - tx_end) < (int32_t)(10u * MS))
        run_for(10u * US, 10u * US); // nvm_pass() asserts port.idle on every write

    TEST_ASSERT_EQUAL_INT(1, writes_state);
    TEST_ASSERT_TRUE(last_write_tick - tx_end >= NVM_BUS_QUIET_TICKS);
}

// Same with the wait already expired (busy bus for over NVM_BUS_QUIET_MAX_WAIT_TICKS): the short
// window still starts only after the long reply ended.
static void test_long_reply_with_expired_wait_is_never_overlapped(void)
{
    nvm_job_changed(&state_job, tick);
    wait_st.active = 1u;
    wait_st.expired = 1u;
    wait_st.since = tick - NVM_BUS_QUIET_MAX_WAIT_TICKS;

    uint32_t tx_end = 0u;
    for (int cycle = 0; cycle < 100 && writes_state == 0; cycle++)
    {
        // 12-byte request, our 300-byte reply (about 2.6 ms), then 400 us of silence.
        reply_len = 300;
        printer_frame();
        reply_len = 0;
        tx_end = tx_end_at;
        run_for(300u * BYTE_TICKS + 400u * US, 10u * US);
    }

    TEST_ASSERT_EQUAL_INT(1, writes_state);
    TEST_ASSERT_TRUE(last_write_tx_gap >= NVM_BUS_QUIET_SHORT_TICKS);
    TEST_ASSERT_TRUE(last_write_tick - tx_end >= NVM_BUS_QUIET_SHORT_TICKS);
}

static void test_state_is_written_at_once_on_a_quiet_bus(void)
{
    nvm_job_changed(&state_job, tick);
    nvm_pass();
    TEST_ASSERT_EQUAL_INT(1, writes_state);
    TEST_ASSERT_EQUAL_HEX32(tick, last_write_tick);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_tx_idle_needs_no_reply_in_flight_or_queued);
    RUN_TEST(test_rx_idle_tracks_frames_arriving_and_waiting);
    RUN_TEST(test_rx_idle_is_false_during_a_heartbeat_skip);
    RUN_TEST(test_stamp_just_after_now_is_a_zero_gap);
    RUN_TEST(test_rx_idle_with_a_byte_stamped_after_now);
    RUN_TEST(test_window_needs_1ms_of_silence_both_ways);
    RUN_TEST(test_long_wait_settles_for_no_frame_on_the_line);
    RUN_TEST(test_filament_settles_500ms_after_the_last_change);
    RUN_TEST(test_filament_that_keeps_changing_is_written_after_5s);
    RUN_TEST(test_failed_write_waits_before_retrying);
    RUN_TEST(test_state_write_waits_for_reply_end_plus_1ms);
    RUN_TEST(test_filament_burst_ends_in_one_write_in_a_quiet_window);
    RUN_TEST(test_busy_bus_falls_back_after_max_wait);
    RUN_TEST(test_each_job_waits_for_its_own_window);
    RUN_TEST(test_one_job_per_pass_state_first);
    RUN_TEST(test_failing_slot_backs_off_and_does_not_block_others);
    RUN_TEST(test_new_change_after_a_failed_write_is_not_delayed);
    RUN_TEST(test_long_reply_is_never_overlapped);
    RUN_TEST(test_long_reply_with_expired_wait_is_never_overlapped);
    RUN_TEST(test_state_is_written_at_once_on_a_quiet_bus);
    return UNITY_END();
}
