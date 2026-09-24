// Host tests for the BambuBus RX frame parser in src/_bus_hardware.h (_bus_port_deal::rx_byte):
// a frame that lost bytes must not swallow the next frame once the line went quiet or the USART
// flagged an overrun, and back-to-back frames must still parse. Also the parser resets around our own
// TX (send_package / tx_done): a frame cut by our reply must not combine with the bytes after it.

#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "_bus_hardware.h"
#include "crc_bus.h"

// One 9E1 byte at 1.25 Mbaud in 18 MHz SysTick ticks (158.4).
static const uint32_t BYTE_TICKS = 158u;
static const uint32_t FRAME_GAP_TICKS = 400u * 18u; // 400 us of silence between frames

static _bus_port_deal port;
static uint32_t tick;
static int heartbeats;
static int sends;
static uint16_t sent_len;

void bambubus_heartbeat_seen_fast(void) { heartbeats++; }

// Stand-in for bus_uart1_dma_send: records the TX; the test ends it with tx_done().
static void fake_send(uint8_t *, uint16_t len)
{
    sends++;
    sent_len = len;
}

void setUp(void)
{
    port.init(fake_send);
    tick = 0x10000000u;
    heartbeats = 0;
    sends = 0;
    sent_len = 0;
    // One quiet period after init, as on the real bus.
    port.rx_byte(0xFF, tick, false);
    tick += FRAME_GAP_TICKS;
}

void tearDown(void) {}

// Short-header frame: 3D, flags (bit 7 set), total length, CRC8, payload, CRC16 (little endian).
static int make_short(uint8_t *out, uint8_t flags, const uint8_t *payload, int n)
{
    const int len = 4 + n + 2;
    out[0] = 0x3D;
    out[1] = flags;
    out[2] = (uint8_t)len;
    out[3] = bus_crc8(out, 3);
    memcpy(out + 4, payload, (size_t)n);
    const uint16_t crc = bus_crc16(out, (uint32_t)(len - 2));
    out[len - 2] = (uint8_t)(crc & 0xFFu);
    out[len - 1] = (uint8_t)(crc >> 8);
    return len;
}

// Payload bytes chosen so that neither they nor the CRC16 look like a 0x3D/0x33 header byte, which
// keeps the tests deterministic when the parser hunts for a header inside a broken frame.
static int make_frame(uint8_t *out, uint8_t seed, int n)
{
    uint8_t payload[64];
    for (uint8_t s = seed;; s++)
    {
        for (int i = 0; i < n; i++)
            payload[i] = (uint8_t)(0x40u + ((s + i * 7) & 0x3Fu));
        const int len = make_short(out, 0xC0, payload, n);
        bool clean = true;
        for (int i = 1; i < len; i++)
            clean = clean && out[i] != 0x3D && out[i] != 0x33;
        if (clean)
            return len;
    }
}

static void feed(const uint8_t *d, int n, uint32_t spacing)
{
    for (int i = 0; i < n; i++)
    {
        tick += spacing;
        port.rx_byte(d[i], tick, false);
    }
}

static void quiet(void) { tick += FRAME_GAP_TICKS; }

// The frame handed to the main loop must be exactly `want`; then the main loop consumes it.
static void expect_frame(const uint8_t *want, int len)
{
    TEST_ASSERT_EQUAL_INT(len, port.recv_data_len);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)_bus_data_type::bambubus, (uint8_t)port.bus_package_type);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(want, port.bus_recv_data_ptr, len);
    port.recv_data_len = 0;
    port.bus_package_type = _bus_data_type::none;
}

static void expect_no_frame(void) { TEST_ASSERT_EQUAL_INT(0, port.recv_data_len); }

static void test_back_to_back_frames_all_parse(void)
{
    uint8_t a[32], b[32], c[32];
    const int la = make_frame(a, 1, 20), lb = make_frame(b, 2, 4), lc = make_frame(c, 3, 12);

    feed(a, la, BYTE_TICKS);
    expect_frame(a, la);
    feed(b, lb, BYTE_TICKS);
    expect_frame(b, lb);
    feed(c, lc, BYTE_TICKS);
    expect_frame(c, lc);
}

static void test_isr_jitter_up_to_the_threshold_keeps_the_frame(void)
{
    uint8_t a[32];
    const int la = make_frame(a, 4, 20);

    // Worst case inside a frame is 2 byte times between RX interrupts; also hit the threshold exactly.
    for (int i = 0; i < la; i++)
    {
        tick += (i == 7) ? BUS_RX_RESYNC_GAP_TICKS : ((i & 1) ? 1u : 2u * BYTE_TICKS);
        port.rx_byte(a[i], tick, false);
    }
    expect_frame(a, la);
}

static void test_gap_just_over_the_threshold_drops_the_frame(void)
{
    uint8_t a[32], b[32];
    const int la = make_frame(a, 5, 20), lb = make_frame(b, 6, 8);

    feed(a, 10, BYTE_TICKS);
    tick += BUS_RX_RESYNC_GAP_TICKS + 1u;
    port.rx_byte(a[10], tick, false);
    feed(a + 11, la - 11, BYTE_TICKS);
    expect_no_frame();

    quiet();
    feed(b, lb, BYTE_TICKS);
    expect_frame(b, lb);
}

// The audit scenario: an IRQ-off window loses one byte of frame A without leaving ORE behind (the
// ISR ran late but before the next byte), then the printer's next frame follows after a pause.
static void test_lost_byte_then_gap_next_frame_parses(void)
{
    uint8_t a[32], b[32];
    const int la = make_frame(a, 7, 20), lb = make_frame(b, 8, 8);

    feed(a, 9, BYTE_TICKS);
    feed(a + 10, la - 10, BYTE_TICKS); // a[9] lost
    expect_no_frame();

    quiet();
    feed(b, lb, BYTE_TICKS);
    expect_frame(b, lb);
}

static void test_lost_bytes_in_heartbeat_skip_then_gap_next_frame_parses(void)
{
    uint8_t hb[32], b[32];
    uint8_t hb_payload[12];
    memset(hb_payload, 0x55, sizeof(hb_payload));
    hb_payload[0] = 0x20;
    const int lh = make_short(hb, 0xC5, hb_payload, (int)sizeof(hb_payload));
    const int lb = make_frame(b, 9, 8);

    // A whole heartbeat is dropped by counting and then reported.
    feed(hb, lh, BYTE_TICKS);
    TEST_ASSERT_EQUAL_INT(1, heartbeats);
    expect_no_frame();

    // A heartbeat that lost two bytes is still reported (its header passed CRC8, as before) and no
    // longer eats the head of the next frame.
    quiet();
    feed(hb, 8, BYTE_TICKS);
    feed(hb + 10, lh - 10, BYTE_TICKS);
    quiet();
    feed(b, lb, BYTE_TICKS);
    TEST_ASSERT_EQUAL_INT(2, heartbeats);
    expect_frame(b, lb);
}

// Starting mid-frame (after a reset or an overrun), a 0x3D in the payload can form a long header
// whose CRC8 happens to match; the length it claims must not blind RX past the next quiet period.
static void test_false_long_header_is_cleared_by_the_next_gap(void)
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
    const int lb = make_frame(b, 10, 8);

    feed(fake, 16, BYTE_TICKS);
    quiet();
    feed(b, lb, BYTE_TICKS);
    expect_frame(b, lb);
}

// ORE: the byte read is good, a later one was lost. The next frame may follow at once.
static void test_overrun_mid_frame_back_to_back_next_frame_parses(void)
{
    uint8_t a[32], b[32];
    const int la = make_frame(a, 11, 20), lb = make_frame(b, 12, 8);

    feed(a, 9, BYTE_TICKS);
    tick += 2u * BYTE_TICKS;
    port.rx_byte(a[9], tick, true); // a[10] overran
    feed(a + 11, la - 11, BYTE_TICKS);
    expect_no_frame();

    feed(b, lb, BYTE_TICKS);
    expect_frame(b, lb);
}

static void test_overrun_on_the_last_byte_keeps_that_frame(void)
{
    uint8_t a[32], b[32], c[32];
    const int la = make_frame(a, 13, 20), lb = make_frame(b, 14, 8), lc = make_frame(c, 15, 8);

    feed(a, la - 1, BYTE_TICKS);
    tick += 2u * BYTE_TICKS;
    port.rx_byte(a[la - 1], tick, true); // b[0] overran
    expect_frame(a, la);

    feed(b + 1, lb - 1, BYTE_TICKS);
    expect_no_frame();
    quiet();
    feed(c, lc, BYTE_TICKS);
    expect_frame(c, lc);
}

// ORE with no byte pending (it was raised after the previous STATR read): the frame in progress lost a
// byte, so it must be dropped, and a frame that follows back to back must parse.
static void test_overrun_without_data_drops_frame_and_next_frame_parses(void)
{
    uint8_t a[32], b[32];
    const int la = make_frame(a, 16, 20), lb = make_frame(b, 17, 8);

    feed(a, 9, BYTE_TICKS);
    port.rx_overrun(); // a[9] lost
    feed(a + 10, la - 10, BYTE_TICKS);
    expect_no_frame();

    feed(b, lb, BYTE_TICKS);
    expect_frame(b, lb);
}

static void test_bytes_during_own_tx_are_ignored_but_timestamped(void)
{
    uint8_t a[32], echo[32];
    const int la = make_frame(a, 18, 8);
    const int le = make_frame(echo, 19, 8);

    port.idle = false;
    feed(echo, le, BYTE_TICKS);
    TEST_ASSERT_EQUAL_HEX32(tick, port.last_rx_tick());
    port.idle = true;
    expect_no_frame();

    quiet();
    feed(a, la, BYTE_TICKS);
    expect_frame(a, la);
    TEST_ASSERT_EQUAL_HEX32(tick, port.last_rx_tick());
}

// Jump the simulated clock to `t` after a quiet line, so a test can sit next to the 2^32 SysTick wrap.
static void restart_clock_quiet(uint32_t t)
{
    tick = t - FRAME_GAP_TICKS;
    port.rx_byte(0xFF, tick, false);
    quiet();
}

static void test_frame_across_the_systick_wrap_parses(void)
{
    uint8_t a[32];
    const int la = make_frame(a, 20, 20);

    restart_clock_quiet(0xFFFFFFFFu - 5u * BYTE_TICKS);
    feed(a, la, BYTE_TICKS);
    TEST_ASSERT_TRUE(tick < 0x10000u);
    expect_frame(a, la);
}

static void test_lost_byte_then_gap_across_the_systick_wrap(void)
{
    uint8_t a[32], b[32];
    const int la = make_frame(a, 21, 20), lb = make_frame(b, 22, 8);

    restart_clock_quiet(0xFFFFFFFFu - (uint32_t)la * BYTE_TICKS - FRAME_GAP_TICKS / 2u);
    feed(a, 9, BYTE_TICKS);
    feed(a + 10, la - 10, BYTE_TICKS); // a[9] lost
    TEST_ASSERT_TRUE(tick > 0xFFFF0000u);
    quiet();
    TEST_ASSERT_TRUE(tick < 0x10000u);
    feed(b, lb, BYTE_TICKS);
    expect_frame(b, lb);
}

// Our reply of n bytes, started now by the main loop.
static void start_reply(int n)
{
    port.send_data_len = n;
    port.send_package();
    TEST_ASSERT_FALSE(port.idle);
}

// TC ISR after n byte times of TX; `rx` (may be null) is what RX saw meanwhile (echo or collision).
static void end_reply(int n, const uint8_t *rx)
{
    for (int i = 0; i < n; i++)
    {
        tick += BYTE_TICKS;
        if (rx)
            port.rx_byte(rx[i], tick, false);
    }
    port.tx_done(tick);
    TEST_ASSERT_TRUE(port.idle);
}

// A late reply collides with the printer's next frame A. The 8-byte set_filament ACK takes 70 us,
// under the resync gap, and RX hears nothing while DE is on: A's head must not combine with its
// tail after our TX and then take the head of frame B. The reply itself still goes out.
static void test_frame_cut_by_a_short_reply_does_not_swallow_the_next_frame(void)
{
    uint8_t a[32], b[32];
    const int la = make_frame(a, 23, 20), lb = make_frame(b, 24, 8);

    feed(a, 10, BYTE_TICKS);
    tick += BYTE_TICKS;
    start_reply(8);
    TEST_ASSERT_EQUAL_INT(1, sends);
    TEST_ASSERT_EQUAL_UINT16(8, sent_len);
    const bool parser_idle_during_tx = port.rx_idle(tick);
    end_reply(8, nullptr); // a[10..17] lost in the collision
    feed(a + 18, la - 18, BYTE_TICKS);
    expect_no_frame();

    feed(b, lb, BYTE_TICKS);
    expect_frame(b, lb);
    TEST_ASSERT_TRUE(parser_idle_during_tx); // A's head was dropped when our TX started
}

// Same with a longer reply that RX echoes: our bytes are timestamped, so the line never looks quiet.
// The rest of A is lost under our TX; the printer's next frame follows 100 us after it.
static void test_frame_cut_by_an_echoed_reply_does_not_swallow_the_next_frame(void)
{
    uint8_t a[32], b[32], echo[32];
    make_frame(a, 25, 20);
    const int lb = make_frame(b, 26, 8);
    const int le = make_frame(echo, 27, 23); // 29 bytes, like the online_detect reply

    feed(a, 10, BYTE_TICKS);
    tick += BYTE_TICKS;
    start_reply(le);
    end_reply(le, echo);
    expect_no_frame();

    tick += 100u * 18u;
    feed(b, lb, BYTE_TICKS);
    expect_frame(b, lb);
}

// A heartbeat whose header passed CRC8 is reported when our TX cuts it, once, and its tail after our
// TX does not eat the head of the next frame.
static void test_heartbeat_cut_by_our_tx_is_reported_once(void)
{
    uint8_t hb[32], b[32];
    uint8_t hb_payload[12];
    memset(hb_payload, 0x55, sizeof(hb_payload));
    hb_payload[0] = 0x20;
    const int lh = make_short(hb, 0xC5, hb_payload, (int)sizeof(hb_payload));
    const int lb = make_frame(b, 28, 8);
    for (int i = 16; i < lh; i++)
        TEST_ASSERT_TRUE(hb[i] != 0x3D && hb[i] != 0x33); // the tail holds no header byte

    feed(hb, 8, BYTE_TICKS);
    TEST_ASSERT_EQUAL_INT(0, heartbeats);
    tick += BYTE_TICKS;
    start_reply(8);
    TEST_ASSERT_EQUAL_INT(1, heartbeats);
    end_reply(8, nullptr); // hb[8..15] lost in the collision
    feed(hb + 16, lh - 16, BYTE_TICKS);
    feed(b, lb, BYTE_TICKS);
    TEST_ASSERT_EQUAL_INT(1, heartbeats);
    expect_frame(b, lb);
}

// The TC ISR resets too: parser state from before a TX never meets the bytes after it, even if
// that TX did not start through send_package().
static void test_tx_done_resets_the_parser(void)
{
    uint8_t a[32], b[32];
    const int la = make_frame(a, 29, 20), lb = make_frame(b, 30, 8);

    feed(a, 10, BYTE_TICKS);
    port.idle = false;
    end_reply(8, nullptr);
    feed(a + 18, la - 18, BYTE_TICKS);
    expect_no_frame();

    feed(b, lb, BYTE_TICKS);
    expect_frame(b, lb);
}

// The normal exchange is unchanged: a request that completed before our reply stays queued for the
// main loop, and the printer's next frame after our reply parses.
static void test_normal_exchange_keeps_the_request_and_parses_the_next_frame(void)
{
    uint8_t a[32], b[32], c[32];
    const int la = make_frame(a, 31, 20), lb = make_frame(b, 32, 4), lc = make_frame(c, 33, 20);

    feed(a, la, BYTE_TICKS);
    tick += 50u * 18u; // delay_us(50) before the reply
    start_reply(29);
    TEST_ASSERT_EQUAL_INT(1, sends);
    TEST_ASSERT_EQUAL_UINT16(29, sent_len);
    TEST_ASSERT_EQUAL_INT(0, port.send_data_len);
    end_reply(29, nullptr);
    expect_frame(a, la);

    tick += 100u * 18u;
    feed(b, lb, BYTE_TICKS);
    expect_frame(b, lb);

    // The raw send_package(data, len) resets the same way.
    feed(c, 10, BYTE_TICKS);
    tick += BYTE_TICKS;
    port.send_package(a, 8);
    TEST_ASSERT_EQUAL_INT(2, sends);
    end_reply(8, nullptr);
    feed(c + 18, lc - 18, BYTE_TICKS);
    expect_no_frame();
    feed(b, lb, BYTE_TICKS);
    expect_frame(b, lb);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_back_to_back_frames_all_parse);
    RUN_TEST(test_isr_jitter_up_to_the_threshold_keeps_the_frame);
    RUN_TEST(test_gap_just_over_the_threshold_drops_the_frame);
    RUN_TEST(test_lost_byte_then_gap_next_frame_parses);
    RUN_TEST(test_lost_bytes_in_heartbeat_skip_then_gap_next_frame_parses);
    RUN_TEST(test_false_long_header_is_cleared_by_the_next_gap);
    RUN_TEST(test_overrun_mid_frame_back_to_back_next_frame_parses);
    RUN_TEST(test_overrun_on_the_last_byte_keeps_that_frame);
    RUN_TEST(test_overrun_without_data_drops_frame_and_next_frame_parses);
    RUN_TEST(test_bytes_during_own_tx_are_ignored_but_timestamped);
    RUN_TEST(test_frame_across_the_systick_wrap_parses);
    RUN_TEST(test_lost_byte_then_gap_across_the_systick_wrap);
    RUN_TEST(test_frame_cut_by_a_short_reply_does_not_swallow_the_next_frame);
    RUN_TEST(test_frame_cut_by_an_echoed_reply_does_not_swallow_the_next_frame);
    RUN_TEST(test_heartbeat_cut_by_our_tx_is_reported_once);
    RUN_TEST(test_tx_done_resets_the_parser);
    RUN_TEST(test_normal_exchange_keeps_the_request_and_parses_the_next_frame);
    return UNITY_END();
}
