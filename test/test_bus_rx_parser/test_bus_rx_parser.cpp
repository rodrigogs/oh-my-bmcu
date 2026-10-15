// Host tests for the BambuBus/AHUB RX frame parser in src/_bus_hardware.h (_bus_port_deal::irq) and
// the main-loop RX path around it (bus_rx_poll() in _bus_hardware.cpp, modelled in bus_rx_frames.h).
// The parser: back-to-back frames parse, the minimum lengths 7 (short header), 15 (long) and 12 (AHUB),
// the heartbeat skip, the queue of two frames (a third is dropped and counted), reset_rx() and the TX
// mark (rx_mark_tx / rx_reached). The poll: the heartbeat fast path, the stall reset after
// BUS_RX_RESYNC_GAP_TICKS of quiet line, and a frame our TX cut is dropped at the TX's ring position
// instead of combining with the bytes after it (audit #24), also when the heartbeat fast path would skip
// across that position.

#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "_bus_hardware.h"
#include "bus_rx_frames.h"
#include "crc_bus.h"

static _bus_port_deal port;
static bus_rx_model ring;
static uint32_t tick;
static int heartbeats;
static int sends;
static uint16_t sent_len;

void bambubus_heartbeat_seen_fast(void) { heartbeats++; }

// Stand-in for bus_uart1_dma_send: records the TX and starts it (the test sets idle back).
static bool fake_send(uint8_t *, uint16_t len)
{
    sends++;
    sent_len = len;
    return true;
}

void setUp(void)
{
    port.init(fake_send);
    port.rx_dropped = 0; // a counter init() leaves alone
    tick = 0x10000000u;
    bus_rx_model_init(&ring, &port, tick);
    heartbeats = 0;
    sends = 0;
    sent_len = 0;
}

void tearDown(void) {}

// No header byte after the first: a frame that the parser drops part-way leaves bytes it hunts through
// for a header, and these must not start one, so each test is deterministic.
static bool clean(const uint8_t *f, int len)
{
    for (int i = 1; i < len; i++)
        if (f[i] == 0x3D || f[i] == 0x33)
            return false;
    return true;
}

// A clean short-header frame with n payload bytes (6 + n bytes), different per seed.
static int make_frame(uint8_t *out, uint8_t seed, int n)
{
    uint8_t payload[64];
    for (uint8_t s = seed;; s++)
    {
        for (int i = 0; i < n; i++)
            payload[i] = (uint8_t)(0x40u + ((s + i * 7) & 0x3Fu));
        const int len = make_short(out, 0xC0, payload, n);
        if (clean(out, len))
            return len;
    }
}

// The same for a long-header frame (9 + n bytes).
static int make_long_frame(uint8_t *out, uint8_t seed, int n)
{
    uint8_t payload[64];
    for (uint8_t s = seed;; s++)
    {
        for (int i = 0; i < n; i++)
            payload[i] = (uint8_t)(0x40u + ((s + i * 5) & 0x3Fu));
        const int len = make_long(out, 0x05, payload, n);
        if (clean(out, len))
            return len;
    }
}

// A heartbeat: 3D C5, length, CRC8, 20, then 0x55 (len bytes in all).
static int make_heartbeat(uint8_t *out, int len)
{
    uint8_t payload[64];
    memset(payload, 0x55, sizeof(payload));
    payload[0] = 0x20;
    return make_short(out, 0xC5, payload, len - 6);
}

// Straight into the parser, as bus_rx_poll() does with each byte it takes from the ring.
static void feed(const uint8_t *d, int n)
{
    for (int i = 0; i < n; i++)
        port.irq(d[i]);
}

// The frame at the head of the queue must be exactly `want`; then the main loop releases it.
static void expect_frame(const uint8_t *want, int len)
{
    TEST_ASSERT_EQUAL_INT(len, port.recv_data_len);
    TEST_ASSERT_EQUAL_UINT8(want[0], (uint8_t)port.bus_package_type);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(want, port.bus_recv_data_ptr, len);
    port.release_packet();
}

static void expect_no_frame(void)
{
    TEST_ASSERT_EQUAL_INT(0, port.recv_data_len);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)_bus_data_type::none, (uint8_t)port.bus_package_type);
}

// ---- The parser ----

static void test_back_to_back_frames_all_parse(void)
{
    uint8_t a[32], b[32], c[32], d[32];
    const int la = make_frame(a, 1, 20), lb = make_frame(b, 2, 4), lc = make_frame(c, 3, 12);
    const int ld = make_long_frame(d, 4, 10);

    feed(a, la);
    expect_frame(a, la);
    feed(b, lb);
    expect_frame(b, lb);
    feed(c, lc);
    expect_frame(c, lc);
    feed(d, ld);
    expect_frame(d, ld);
    TEST_ASSERT_FALSE(port.receiving());
    TEST_ASSERT_EQUAL_UINT32(0, port.rx_dropped);
}

// A short header claiming 6 bytes (no payload) is dropped at its length byte; 7 is the shortest frame.
static void test_short_header_minimum_is_7_bytes(void)
{
    uint8_t f6[16], f7[16], b[32];
    const uint8_t p[] = {0x41};
    const int l6 = make_short(f6, 0xC0, p, 0), l7 = make_short(f7, 0xC0, p, 1);
    const int lb = make_frame(b, 5, 8);
    TEST_ASSERT_TRUE(clean(f6, l6) && clean(f7, l7));

    feed(f6, l6);
    expect_no_frame();
    TEST_ASSERT_FALSE(port.receiving());
    feed(b, lb);
    expect_frame(b, lb);

    feed(f7, l7);
    expect_frame(f7, l7);
}

// A long header claiming 14 bytes is dropped; 15 (the 13 header bytes up to the type, then the CRC16) is
// the shortest; more than the 1280-byte buffer is dropped too.
static void test_long_header_minimum_is_15_and_maximum_1280_bytes(void)
{
    uint8_t f14[32], f15[32], b[32];
    const int l14 = make_long_frame(f14, 6, 5), l15 = make_long_frame(f15, 7, 6);
    const int lb = make_frame(b, 8, 8);
    TEST_ASSERT_EQUAL_INT(14, l14);
    TEST_ASSERT_EQUAL_INT(15, l15);

    feed(f14, l14);
    expect_no_frame();
    feed(b, lb);
    expect_frame(b, lb);

    feed(f15, l15);
    expect_frame(f15, l15);

    uint8_t big[8] = {0x3D, 0x05, 0x2A, 0x00, (uint8_t)(1281u & 0xFFu), (uint8_t)(1281u >> 8), 0x00, 0x00};
    big[6] = bus_crc8(big, 6);
    feed(big, 6);
    TEST_ASSERT_FALSE(port.receiving()); // dropped at the length's high byte
    feed(b, lb);
    expect_frame(b, lb);
}

// An AHUB frame is length * 4 + 12 bytes, so 12 (no length words) is the shortest and is handed over.
static void test_ahub_frame_of_12_bytes_parses(void)
{
    uint8_t h[1100];
    const int l12 = make_ahub(h, 0x80, 0, 0);
    TEST_ASSERT_EQUAL_INT(12, l12);
    feed(h, l12);
    expect_frame(h, l12);

    const int lmax = make_ahub(h, 0x80, 255, 0);
    TEST_ASSERT_EQUAL_INT(1032, lmax);
    feed(h, lmax);
    expect_frame(h, lmax);
}

// A heartbeat is skipped by counting its bytes and reported once, never queued; the frame after it
// parses.
static void test_heartbeat_is_counted_not_queued(void)
{
    uint8_t hb[32], b[32];
    const int lh = make_heartbeat(hb, 18), lb = make_frame(b, 9, 8);

    feed(hb, lh - 1);
    TEST_ASSERT_EQUAL_INT(0, heartbeats);
    TEST_ASSERT_TRUE(port.receiving());
    feed(hb + lh - 1, 1);
    TEST_ASSERT_EQUAL_INT(1, heartbeats);
    TEST_ASSERT_FALSE(port.receiving());
    expect_no_frame();

    feed(b, lb);
    expect_frame(b, lb);
    TEST_ASSERT_EQUAL_INT(1, heartbeats);
}

// Two completed frames wait in the queue, the head first; a third that completes before the main loop
// released one is dropped and counted. Each release moves to the next; the queued bytes stay intact
// while later frames are parsed.
static void test_queue_holds_two_frames_and_drops_a_third(void)
{
    uint8_t a[32], b[32], c[32], d[32];
    const int la = make_frame(a, 10, 20), lb = make_frame(b, 11, 6), lc = make_frame(c, 12, 12);
    const int ld = make_frame(d, 13, 9);

    feed(a, la);
    feed(b, lb);
    feed(c, lc);
    TEST_ASSERT_EQUAL_UINT32(1, port.rx_dropped);

    expect_frame(a, la);
    feed(d, ld); // the slot c would have taken is free again
    expect_frame(b, lb);
    expect_frame(d, ld);
    expect_no_frame();

    // release_packet() on an empty queue changes nothing.
    port.release_packet();
    expect_no_frame();
    feed(a, la);
    expect_frame(a, la);
    TEST_ASSERT_EQUAL_UINT32(1, port.rx_dropped);
}

// reset_rx() (bus_rx_poll's stall reset and ring restart) drops the frame in progress, and the heartbeat
// skip without reporting it; the next frame parses. A queued frame is not touched.
static void test_reset_rx_drops_what_is_in_progress(void)
{
    uint8_t a[32], b[32], hb[32];
    const int la = make_frame(a, 14, 20), lb = make_frame(b, 15, 8), lh = make_heartbeat(hb, 20);

    feed(b, lb);
    feed(a, 10);
    TEST_ASSERT_TRUE(port.receiving());
    port.reset_rx();
    TEST_ASSERT_FALSE(port.receiving());
    feed(a + 10, la - 10);
    expect_frame(b, lb);
    expect_no_frame();

    TEST_ASSERT_TRUE(clean(hb + 7, lh - 7)); // the skipped tail starts no header once the skip is gone
    feed(hb, 8);
    port.reset_rx();
    feed(hb + 8, lh - 8);
    feed(b, lb);
    TEST_ASSERT_EQUAL_INT(0, heartbeats);
    expect_frame(b, lb);
}

// ---- The TX mark (rx_mark_tx / rx_reached) ----

// Our TX started while frame A was arriving (mark at the ring position of A's 10th byte); RX was off
// during it, so A lost its middle. Reaching the mark drops A's head (counted), and A's tail cannot
// combine with it or take the head of the next frame.
static void test_frame_in_progress_at_the_mark_is_dropped(void)
{
    uint8_t a[32], b[32];
    const int la = make_frame(a, 16, 20), lb = make_frame(b, 17, 8);

    feed(a, 10);
    port.rx_mark_tx(10u);
    port.rx_reached(9u); // the parser has not reached the mark yet
    TEST_ASSERT_TRUE(port.receiving());
    port.rx_reached(10u);
    TEST_ASSERT_FALSE(port.receiving());
    TEST_ASSERT_EQUAL_UINT32(1, port.rx_dropped);

    feed(a + 18, la - 18); // a[10..17] lost under our TX
    feed(b, lb);
    expect_frame(b, lb);
    expect_no_frame();
}

// A frame completed before the mark (the request our TX answers) stays queued; with the parser between
// frames reaching the mark changes nothing, and the mark is used once.
static void test_mark_keeps_a_completed_frame_and_is_used_once(void)
{
    uint8_t a[32], b[32];
    const int la = make_frame(a, 18, 20), lb = make_frame(b, 19, 8);

    feed(a, la);
    port.rx_mark_tx((uint32_t)la);
    port.rx_reached((uint32_t)la);
    TEST_ASSERT_EQUAL_UINT32(0, port.rx_dropped);

    feed(b, 5);
    port.rx_reached((uint32_t)la + 5u); // already used: b is not dropped
    feed(b + 5, lb - 5);
    expect_frame(a, la);
    expect_frame(b, lb);
    TEST_ASSERT_EQUAL_UINT32(0, port.rx_dropped);
}

// reset_rx() and init() clear a pending mark (the ring restarts from 0 after rx_fault, so an old mark
// would be compared with new positions); the mark compares positions across the 2^32 wrap.
static void test_mark_is_cleared_by_reset_and_init_and_survives_the_wrap(void)
{
    uint8_t a[32];
    const int la = make_frame(a, 20, 20);

    port.rx_mark_tx(100u);
    port.reset_rx();
    feed(a, 10);
    port.rx_reached(200u);
    TEST_ASSERT_TRUE(port.receiving());
    feed(a + 10, la - 10);
    expect_frame(a, la);

    port.rx_mark_tx(100u);
    port.init(fake_send);
    feed(a, 10);
    port.rx_reached(200u);
    TEST_ASSERT_TRUE(port.receiving());
    port.reset_rx();

    port.rx_mark_tx(0xFFFFFFF0u);
    feed(a, 10);
    port.rx_reached(0xFFFFFFEFu);
    TEST_ASSERT_TRUE(port.receiving());
    port.rx_reached(0x00000004u); // past the mark, after the wrap
    TEST_ASSERT_FALSE(port.receiving());
}

// send_package() starts the TX only when idle, and keeps the reply when the TX did not start.
static bool refusing_send(uint8_t *, uint16_t)
{
    sends++;
    return false;
}

static void test_send_package_keeps_a_reply_the_tx_refused(void)
{
    port.init(refusing_send);
    port.send_data_len = 8;
    port.send_package();
    TEST_ASSERT_EQUAL_INT(1, sends);
    TEST_ASSERT_EQUAL_INT(8, port.send_data_len);
    uint8_t *const build = port.tx_build_buf();

    port.init(fake_send);
    port.send_data_len = 8;
    port.idle = false;
    port.send_package();
    TEST_ASSERT_EQUAL_INT(1, sends); // not idle: not even tried
    port.idle = true;
    port.send_package();
    TEST_ASSERT_EQUAL_INT(2, sends);
    TEST_ASSERT_EQUAL_UINT16(8, sent_len);
    TEST_ASSERT_EQUAL_INT(0, port.send_data_len);
    TEST_ASSERT_TRUE(port.tx_build_buf() != build); // the next reply is built in the other buffer
}

// ---- The main-loop RX path (bus_rx_poll, bus_rx_frames.h) ----

// Bytes arrive one byte time apart; the main loop polls whenever the test says.
static void arrive(const uint8_t *d, int n)
{
    for (int i = 0; i < n; i++)
    {
        tick += BYTE_TICKS;
        bus_rx_model_arrive(&ring, d[i]);
    }
}

static void poll(void) { bus_rx_model_poll(&ring, tick); }

// A whole heartbeat found in the ring is skipped without the parser; one split across two polls goes
// through the parser's skip. Either way it is counted once and the frames around it parse.
static void test_poll_skips_heartbeats(void)
{
    uint8_t hb[32], a[32], b[32];
    const int lh = make_heartbeat(hb, 20), la = make_frame(a, 21, 8), lb = make_frame(b, 22, 8);

    arrive(a, la);
    arrive(hb, lh);
    arrive(b, lb);
    poll();
    TEST_ASSERT_EQUAL_INT(1, heartbeats);
    expect_frame(a, la);
    expect_frame(b, lb);

    arrive(hb, 3);
    poll();
    TEST_ASSERT_TRUE(port.receiving());
    arrive(hb + 3, lh - 3);
    poll();
    TEST_ASSERT_EQUAL_INT(2, heartbeats);
    TEST_ASSERT_FALSE(port.receiving());
}

// Stall reset: a frame still open once no byte has arrived for BUS_RX_RESYNC_GAP_TICKS (200 us) lost
// bytes, so it is dropped and counted, and the next frame parses. One tick less keeps it.
static void test_poll_drops_a_frame_after_the_quiet_gap(void)
{
    uint8_t a[32], b[32], c[32];
    const int la = make_frame(a, 23, 20), lc = make_frame(c, 25, 8);
    make_frame(b, 24, 20);

    arrive(a, 10);
    poll();
    tick += BUS_RX_RESYNC_GAP_TICKS - 1u;
    poll();
    TEST_ASSERT_TRUE(port.receiving());
    arrive(a + 10, la - 10);
    poll();
    expect_frame(a, la);

    arrive(b, 9);
    poll(); // b[9..] never arrive
    tick += BUS_RX_RESYNC_GAP_TICKS;
    poll();
    TEST_ASSERT_FALSE(port.receiving());
    TEST_ASSERT_EQUAL_UINT32(1, port.rx_dropped);
    arrive(c, lc);
    poll();
    expect_frame(c, lc);
}

// Starting mid-frame, a 0x3D in the payload can form a long header whose CRC8 happens to match; the
// 1200 bytes it claims are dropped once the line has been quiet for the gap.
static void test_poll_clears_a_false_long_header(void)
{
    uint8_t fake[16], b[32];
    fake[0] = 0x3D;
    fake[1] = 0x00; // long header
    fake[2] = 0x12;
    fake[3] = 0x34;
    fake[4] = (uint8_t)(1200u & 0xFFu);
    fake[5] = (uint8_t)(1200u >> 8);
    fake[6] = bus_crc8(fake, 6);
    for (int i = 7; i < 16; i++)
        fake[i] = 0x5A;
    const int lb = make_frame(b, 26, 8);

    arrive(fake, 16);
    poll();
    TEST_ASSERT_TRUE(port.receiving());
    tick += BUS_RX_RESYNC_GAP_TICKS;
    arrive(b, lb);
    poll(); // b arrived after the gap, but no poll saw the line quiet: b is taken by the false frame
    tick += BUS_RX_RESYNC_GAP_TICKS;
    poll();
    TEST_ASSERT_FALSE(port.receiving());
    arrive(b, lb);
    poll();
    expect_frame(b, lb);
}

// A late reply collides with the printer's frame A: the 8-byte set_filament ACK takes 70 us, under the
// quiet gap, and RX is off meanwhile. A's head, whether the main loop had parsed it before the TX or
// finds it in the ring after, is dropped at the TX's ring position; A's tail and frame B follow at once
// and B parses. The reply itself goes out.
static void test_frame_cut_by_our_tx_does_not_swallow_the_next_frame(void)
{
    uint8_t a[32], b[32];
    const int la = make_frame(a, 27, 20), lb = make_frame(b, 28, 8);

    for (int parsed_before_tx = 0; parsed_before_tx < 2; parsed_before_tx++)
    {
        setUp();
        arrive(a, 10);
        if (parsed_before_tx)
            poll();
        port.send_data_len = 8;
        port.send_package(); // bus_uart1_dma_send(): RX off, the mark at the ring position
        bus_rx_model_tx_start(&ring);
        TEST_ASSERT_EQUAL_INT(1, sends);
        tick += 8u * BYTE_TICKS; // a[10..17] lost while RX was off
        arrive(a + 18, la - 18);
        arrive(b, lb);
        poll();
        expect_frame(b, lb);
        expect_no_frame();
        TEST_ASSERT_EQUAL_UINT32(1, port.rx_dropped);
    }
}

// The first poll after our TX drops the cut frame even when nothing arrived after it yet.
static void test_frame_cut_by_our_tx_is_dropped_by_the_next_poll(void)
{
    uint8_t a[32];
    make_frame(a, 29, 20);

    arrive(a, 10);
    poll();
    bus_rx_model_tx_start(&ring);
    tick += 8u * BYTE_TICKS;
    poll();
    TEST_ASSERT_FALSE(port.receiving());
    TEST_ASSERT_EQUAL_UINT32(1, port.rx_dropped);
}

// The normal exchange is unchanged: the request that completed before our reply stays queued for the
// main loop, nothing is dropped at the mark, and the printer's next frame after our reply parses.
static void test_normal_exchange_keeps_the_request_and_parses_the_next_frame(void)
{
    uint8_t a[32], b[32];
    const int la = make_frame(a, 30, 20), lb = make_frame(b, 31, 4);

    arrive(a, la);
    poll();
    tick += 50u * 18u; // delay_us(50) before the reply
    port.send_data_len = 29;
    port.send_package();
    bus_rx_model_tx_start(&ring);
    tick += 29u * BYTE_TICKS;
    poll();
    tick += 100u * 18u;
    arrive(b, lb);
    poll();
    expect_frame(a, la);
    expect_frame(b, lb);
    TEST_ASSERT_EQUAL_UINT32(0, port.rx_dropped);
}

// Our TX cuts a printer heartbeat after its 5-byte head reached the ring, and no poll ran during the TX:
// the next poll finds the head plus the printer's next frame, more than the heartbeat's length in all.
// The fast path must not skip hb_len bytes across the TX mark (that would eat the frame's head and stamp
// a heartbeat that never completed): the parser takes the head and drops it at the mark, and the frame
// parses (bytes from the review, bus-protocol-1).
static void test_heartbeat_cut_by_our_tx_does_not_swallow_the_next_frame(void)
{
    static const uint8_t head[] = {0x3D, 0xC5, 0x09, 0x15, 0x20};
    static const uint8_t next[] = {0x3D, 0xC5, 0x0C, 0xC8, 0x03, 0x00, 0x07, 0x00, 0x7F, 0x02, 0x36, 0x54};
    TEST_ASSERT_EQUAL_HEX8(head[3], bus_crc8(head, 3));
    TEST_ASSERT_EQUAL_HEX8(next[3], bus_crc8(next, 3));

    arrive(head, (int)sizeof(head));
    bus_rx_model_tx_start(&ring);
    tick += 8u * BYTE_TICKS;
    arrive(next, (int)sizeof(next));
    poll();
    expect_frame(next, (int)sizeof(next));
    expect_no_frame();
    TEST_ASSERT_EQUAL_UINT32(1, port.rx_dropped);
    TEST_ASSERT_EQUAL_INT(0, heartbeats);
    TEST_ASSERT_FALSE(port.receiving());
}

// The same with our own heartbeat-length frames: the first 5 bytes of a heartbeat, the TX mark, then a
// whole frame. Without the mark the same bytes take the fast path (control).
static void test_heartbeat_fast_path_stops_at_a_pending_mark(void)
{
    uint8_t hb[32], b[32];
    const int lh = make_heartbeat(hb, 20), lb = make_frame(b, 32, 20);
    TEST_ASSERT_TRUE(lb >= lh - 5);

    arrive(hb, 5);
    bus_rx_model_tx_start(&ring);
    arrive(b, lb);
    poll();
    expect_frame(b, lb);
    TEST_ASSERT_EQUAL_UINT32(1, port.rx_dropped);
    TEST_ASSERT_EQUAL_INT(0, heartbeats);

    // A heartbeat that ended exactly at the mark (our TX after it) is still skipped in one step.
    setUp();
    arrive(hb, lh);
    bus_rx_model_tx_start(&ring);
    arrive(b, lb);
    poll();
    TEST_ASSERT_EQUAL_INT(1, heartbeats);
    expect_frame(b, lb);
    TEST_ASSERT_EQUAL_UINT32(0, port.rx_dropped);

    // Control: no TX, the head and the frame's bytes are skipped as one heartbeat and the frame is lost.
    setUp();
    arrive(hb, 5);
    arrive(b, lb);
    poll();
    TEST_ASSERT_EQUAL_INT(1, heartbeats);
    expect_no_frame();
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_back_to_back_frames_all_parse);
    RUN_TEST(test_short_header_minimum_is_7_bytes);
    RUN_TEST(test_long_header_minimum_is_15_and_maximum_1280_bytes);
    RUN_TEST(test_ahub_frame_of_12_bytes_parses);
    RUN_TEST(test_heartbeat_is_counted_not_queued);
    RUN_TEST(test_queue_holds_two_frames_and_drops_a_third);
    RUN_TEST(test_reset_rx_drops_what_is_in_progress);
    RUN_TEST(test_frame_in_progress_at_the_mark_is_dropped);
    RUN_TEST(test_mark_keeps_a_completed_frame_and_is_used_once);
    RUN_TEST(test_mark_is_cleared_by_reset_and_init_and_survives_the_wrap);
    RUN_TEST(test_send_package_keeps_a_reply_the_tx_refused);
    RUN_TEST(test_poll_skips_heartbeats);
    RUN_TEST(test_poll_drops_a_frame_after_the_quiet_gap);
    RUN_TEST(test_poll_clears_a_false_long_header);
    RUN_TEST(test_frame_cut_by_our_tx_does_not_swallow_the_next_frame);
    RUN_TEST(test_frame_cut_by_our_tx_is_dropped_by_the_next_poll);
    RUN_TEST(test_normal_exchange_keeps_the_request_and_parses_the_next_frame);
    RUN_TEST(test_heartbeat_cut_by_our_tx_does_not_swallow_the_next_frame);
    RUN_TEST(test_heartbeat_fast_path_stops_at_a_pending_mark);
    return UNITY_END();
}
