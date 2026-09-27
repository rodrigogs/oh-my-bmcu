// Fuzz of the BambuBus/AHUB RX frame parser in src/_bus_hardware.h (_bus_port_deal::rx_byte): seeded
// xorshift streams into the real parser, of noise and of valid frames with faults injected (a lost
// byte, an extra byte, bit flips, a frame cut short, a header that claims another length, an ORE, a
// pause, our own TX cutting in), with ISR jitter, quiet gaps and SysTick wraps in between. After every
// event the parser's state stays in bounds, a frame handed to the main loop is the one its header
// describes, with a good CRC8 and made of the last bytes read, and it is left alone until the main
// loop takes it; a whole frame that starts with the parser at rest is always handed out (a heartbeat
// counted). The hand-written scenarios are in test_bus_rx_parser.

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "_bus_hardware.h"
#include "bus_rx_frames.h"
#include "crc_bus.h"

// Size of one of the parser's two receive buffers; the accessor below stops compiling if it changes.
static const int RX_BUF_SZ = 1280;
typedef uint8_t rx_bufs_t[2][RX_BUF_SZ];

// Read access to the parser's private state, for the invariants. Access checks do not apply to the
// names in an explicit instantiation ([temp.explicit]), so the firmware class needs no test hook.
template <typename Tag, typename Tag::type M>
struct private_member
{
    friend typename Tag::type member(Tag) { return M; }
};
struct index_of { typedef int _bus_port_deal::*type; };
struct drop_of { typedef int _bus_port_deal::*type; };
struct irq_buf_of { typedef uint8_t *_bus_port_deal::*type; };
struct rx_bufs_of { typedef rx_bufs_t _bus_port_deal::*type; };
index_of::type member(index_of);
drop_of::type member(drop_of);
irq_buf_of::type member(irq_buf_of);
rx_bufs_of::type member(rx_bufs_of);
template struct private_member<index_of, &_bus_port_deal::_index>;
template struct private_member<drop_of, &_bus_port_deal::drop_bytes>;
template struct private_member<irq_buf_of, &_bus_port_deal::bus_irq_data_ptr>;
template struct private_member<rx_bufs_of, &_bus_port_deal::recv_data_buf>;

static _bus_port_deal port;
static uint32_t tick;
static int heartbeats;

void bambubus_heartbeat_seen_fast(void) { heartbeats++; }
static void fake_send(uint8_t *, uint16_t) {}

static int parser_index(void) { return port.*member(index_of()); }
static int parser_drop(void) { return port.*member(drop_of()); }

// xorshift32: every run sees the same streams.
static uint32_t seed;
static uint32_t rng_state;
static uint32_t rng_next(void)
{
    uint32_t x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state = x;
    return x;
}
static uint32_t rng_in(uint32_t lo, uint32_t hi) { return lo + rng_next() % (hi - lo + 1u); }
static bool rng_pct(uint32_t pct) { return rng_next() % 100u < pct; }

static struct
{
    uint32_t events, bytes;
    uint32_t out, out_kind[4], out_full; // frames handed out: all, by header, of RX_BUF_SZ bytes
    int max_index, max_drop;
    uint32_t sent, faulted;
    uint32_t whole, whole_out, whole_busy, whole_hb; // intact from rest: handed out, no room, heartbeats
    uint32_t after_fault, after_fault_out;           // intact right after a faulted one: handed out
} st;

// Where a failure happened, to replay it: the stream's seed and the event count since the test began.
static void expect(bool ok, const char *what)
{
    if (ok)
        return;
    char msg[160];
    snprintf(msg, sizeof(msg), "%s (seed %u, event %u)", what, (unsigned)seed, (unsigned)st.events);
    TEST_FAIL_MESSAGE(msg);
}

// The bytes the parser read (RX outside our own TX), newest last.
static uint8_t seen[2048];
static uint32_t seen_n;

// The last frame handed out; the main loop takes it `hold` events later: at once mostly, sometimes
// only after more frames went by (a slow pass), which then find no room.
static uint8_t out_copy[RX_BUF_SZ];
static int out_len;
static bool out_pending;
static const uint8_t *out_ptr;
static uint32_t hold;

// Where the header's CRC8 sits and the total length it claims, as the parser reads them.
static int crc8_at(const uint8_t *f) { return (f[1] & 0x80u) ? 3 : 6; }
static int claimed_length(const uint8_t *f)
{
    const bool short_head = (f[1] & 0x80u) != 0;
    if (f[0] == 0x3D)
        return short_head ? f[2] : (f[4] | (f[5] << 8));
    return ((short_head ? f[2] : f[4]) << 2) + 12;
}

// Dropped by counting and reported through bambubus_heartbeat_seen_fast(), never handed out.
static bool is_heartbeat(const uint8_t *f, int len)
{
    return len >= 6 && f[0] == 0x3D && f[1] == 0xC5 && f[4] == 0x20;
}

static void new_frame(const uint8_t *f, int len)
{
    expect(f[0] == 0x3D || f[0] == 0x33, "frame without a header byte");
    expect((uint8_t)port.bus_package_type == f[0], "frame type differs from its header byte");
    expect(len == claimed_length(f), "frame length differs from its header");
    expect(len > crc8_at(f), "frame ends inside its header");
    expect(f[crc8_at(f)] == bus_crc8(f, (uint32_t)crc8_at(f)), "frame header CRC8 is bad");
    expect(!is_heartbeat(f, len), "heartbeat handed out");
    bool last_read = seen_n >= (uint32_t)len;
    for (int i = 0; i < len && last_read; i++)
        last_read = f[i] == seen[(seen_n - (uint32_t)len + (uint32_t)i) % sizeof(seen)];
    expect(last_read, "frame is not the last bytes read");

    memcpy(out_copy, f, (size_t)len);
    out_len = len;
    out_pending = true;
    out_ptr = f;
    hold = rng_pct(70) ? 0u : rng_pct(85) ? rng_in(1u, 40u) : rng_in(41u, 1500u);

    st.out++;
    st.out_kind[(f[0] == 0x3D ? 0 : 2) + ((f[1] & 0x80u) ? 0 : 1)]++;
    if (len == RX_BUF_SZ)
        st.out_full++;
}

static void check_state(void)
{
    st.events++;
    const int idx = parser_index();
    const int drop = parser_drop();
    const uint8_t *irq_buf = port.*member(irq_buf_of());
    rx_bufs_t &bufs = port.*member(rx_bufs_of());
    const uint8_t *out = port.bus_recv_data_ptr;
    const int len = port.recv_data_len;

    expect(idx >= 0 && idx < RX_BUF_SZ, "_index out of the buffer");
    expect(drop >= 0 && drop <= 255 - 5, "drop_bytes out of range"); // a short length is one byte
    expect(irq_buf == bufs[0] || irq_buf == bufs[1], "bus_irq_data_ptr off the buffers");
    expect(out == bufs[0] || out == bufs[1], "bus_recv_data_ptr off the buffers");
    expect(irq_buf != out, "the buffer handed out is the one being written");
    expect(len == 0 || (len >= 4 && len <= RX_BUF_SZ), "recv_data_len out of range");
    if (idx > st.max_index)
        st.max_index = idx;
    if (drop > st.max_drop)
        st.max_drop = drop;

    if (out_pending)
    {
        expect(len == out_len && out == out_ptr, "frame handed out changed before the main loop took it");
        return;
    }
    if (len != 0)
        new_frame(out, len);
}

static void main_loop(void)
{
    if (!out_pending)
        return;
    if (hold)
    {
        hold--;
        return;
    }
    expect(memcmp(out_ptr, out_copy, (size_t)out_len) == 0, "frame overwritten before the main loop took it");
    port.recv_data_len = 0;
    port.bus_package_type = _bus_data_type::none;
    out_pending = false;
}

// The parser is between frames: a resync leaves it here.
static bool parser_at_rest(void) { return parser_index() == 0 && parser_drop() == 0; }

static void rx(uint8_t b, uint32_t gap, bool overrun)
{
    tick += gap;
    const bool quiet = (uint32_t)(tick - port.last_rx_tick()) > BUS_RX_RESYNC_GAP_TICKS;
    const bool idle = port.idle;
    const int idx0 = parser_index(), drop0 = parser_drop();
    if (idle)
        seen[seen_n++ % sizeof(seen)] = b;
    port.rx_byte(b, tick, overrun);
    st.bytes++;
    if (!idle)
        expect(parser_index() == idx0 && parser_drop() == drop0, "parser moved during our TX");
    else if (overrun)
        expect(parser_at_rest(), "parser not reset by an ORE");
    else if (quiet)
        expect(parser_drop() == 0 && parser_index() == ((b == 0x3D || b == 0x33) ? 1 : 0),
               "parser not reset by a quiet line");
    check_state();
    main_loop();
}

static void rx_overrun(void)
{
    port.rx_overrun();
    expect(parser_at_rest(), "parser not reset by an ORE without data");
    check_state();
    main_loop();
}

// Our reply of n bytes; RX hears its echo or nothing. The parser resets at both ends; a quarter of
// the TXs do not start through send_package(), so only the TC ISR's reset covers them.
static void own_tx(int n, bool echo)
{
    if (rng_pct(25))
        port.idle = false;
    else
    {
        port.send_data_len = n;
        port.send_package();
        expect(!port.idle && parser_at_rest(), "parser not reset when our TX started");
    }
    check_state();
    for (int i = 0; i < n; i++)
    {
        if (echo)
            rx((uint8_t)rng_next(), BYTE_TICKS, false);
        else
            tick += BYTE_TICKS;
    }
    port.tx_done(tick);
    expect(port.idle && parser_at_rest(), "parser not reset when our TX ended");
    check_state();
    main_loop();
}

// From one RX interrupt to the next inside a frame: up to two byte times (ISR latency), now and then
// longer, up to the resync gap itself, which must not split the frame.
static uint32_t in_frame_gap(void)
{
    const uint32_t r = rng_next() % 1000u;
    if (r < 950u)
        return rng_in(1u, 2u * BYTE_TICKS);
    if (r < 995u)
        return rng_in(2u * BYTE_TICKS, BUS_RX_RESYNC_GAP_TICKS);
    return BUS_RX_RESYNC_GAP_TICKS;
}

// Before a frame: mostly a quiet line, else back to back, exactly the resync gap (no resync) or any
// SysTick delta.
static uint32_t between_frames_gap(void)
{
    const uint32_t r = rng_next() % 100u;
    if (r < 75u)
        return rng_in(BUS_RX_RESYNC_GAP_TICKS + 1u, 20u * BUS_RX_RESYNC_GAP_TICKS);
    if (r < 90u)
        return in_frame_gap();
    if (r < 95u)
        return BUS_RX_RESYNC_GAP_TICKS;
    return rng_next();
}

// A byte of noise: header, flag and heartbeat bytes are frequent, so false headers form.
static uint8_t noise_byte(void)
{
    static const uint8_t hot[] = {0x3D, 0x33, 0x80, 0xC0, 0xC5, 0x00, 0x20, 0x05, 0xFF};
    return rng_pct(30) ? hot[rng_next() % sizeof(hot)] : (uint8_t)rng_next();
}

enum frame_kind { SHORT, HEARTBEAT, LONG, AHUB_SHORT, AHUB_LONG, KINDS };

// Sets the header's length field to claim `len` bytes and its CRC8 to match.
static void set_length(uint8_t *f, int len)
{
    const bool short_head = (f[1] & 0x80u) != 0;
    if (f[0] == 0x3D && short_head)
        f[2] = (uint8_t)len;
    else if (f[0] == 0x3D)
    {
        f[4] = (uint8_t)(len & 0xFF);
        f[5] = (uint8_t)(len >> 8);
    }
    else
        f[short_head ? 2 : 4] = (uint8_t)((len - 12) >> 2);
    f[crc8_at(f)] = bus_crc8(f, (uint32_t)crc8_at(f));
}

// A header whose CRC8 is good but whose length is any the field can hold, around the buffer size
// and the header end most of all.
static void set_any_length(uint8_t *f)
{
    const bool short_head = (f[1] & 0x80u) != 0;
    if (f[0] == 0x3D && !short_head)
    {
        static const int edge[] = {0, 5, 6, 7, RX_BUF_SZ - 1, RX_BUF_SZ, RX_BUF_SZ + 1, 0xFFFF};
        const int len = rng_pct(30)   ? edge[rng_next() % 8u]
                        : rng_pct(50) ? (int)rng_in(0u, 1535u)
                                      : (int)(rng_next() & 0xFFFFu);
        set_length(f, len);
        return;
    }
    f[short_head ? 2 : 4] = (uint8_t)rng_next();
    f[crc8_at(f)] = bus_crc8(f, (uint32_t)crc8_at(f));
}

// A valid frame of this kind; the parser checks only the header CRC8, the rest is random.
static int build_frame(uint8_t *f, frame_kind kind)
{
    uint8_t payload[256];
    for (int i = 0; i < (int)sizeof(payload); i++)
        payload[i] = noise_byte();
    switch (kind)
    {
    case SHORT:
    {
        uint8_t flags = (uint8_t)(0x80u | rng_next());
        if (flags == 0xC5)
            flags = 0xC0;
        return make_short(f, flags, payload, (int)(rng_pct(90) ? rng_in(0u, 40u) : rng_in(41u, 249u)));
    }
    case HEARTBEAT:
        payload[0] = 0x20;
        return make_short(f, 0xC5, payload, (int)rng_in(1u, 40u));
    case LONG:
    {
        const int r = (int)(rng_next() % 100u);
        const int len = r < 80   ? (int)rng_in(7u, 120u)
                        : r < 95 ? (int)rng_in(121u, (uint32_t)RX_BUF_SZ)
                                 : RX_BUF_SZ;
        for (int i = 1; i < len; i++)
            f[i] = noise_byte();
        f[0] = 0x3D;
        f[1] &= 0x7Fu;
        set_length(f, len);
        return len;
    }
    default:
    {
        const int len = 4 * (int)(rng_pct(85) ? rng_in(0u, 30u) : rng_in(31u, 255u)) + 12;
        for (int i = 1; i < len; i++)
            f[i] = noise_byte();
        f[0] = 0x33;
        f[1] = (uint8_t)(kind == AHUB_SHORT ? (f[1] | 0x80u) : (f[1] & 0x7Fu));
        set_length(f, len);
        return len;
    }
    }
}

static void begin_stream(uint32_t s)
{
    seed = s;
    rng_state = 0x2545F491u ^ (0x9E3779B9u * (s + 1u));
    tick = 0u - rng_in(0u, 10000000u); // crosses the SysTick wrap early on
}

// ---- Noise ----

// Noise, with a header of good CRC8 and any length now and then. breaks: per 100000 events, how often
// each of a quiet line, an ORE, an ORE without data and our own TX cuts in; the quiet streams let
// false frames grow to the buffer size.
static void run_noise(uint32_t s, uint32_t events, uint32_t breaks)
{
    begin_stream(s);
    uint8_t f[RX_BUF_SZ + 8];
    const uint32_t end = st.events + events;
    while (st.events < end)
    {
        const uint32_t r = rng_next() % 100000u;
        if (r < 3000u)
        {
            build_frame(f, (frame_kind)(rng_next() % KINDS));
            set_any_length(f);
            for (int i = 0; i <= crc8_at(f); i++)
                rx(f[i], in_frame_gap(), false);
        }
        else if (r < 3000u + breaks)
            rx(noise_byte(), between_frames_gap(), false);
        else if (r < 3000u + 2u * breaks)
            rx(noise_byte(), in_frame_gap(), true);
        else if (r < 3000u + 3u * breaks)
            rx_overrun();
        else if (r < 3000u + 4u * breaks)
            own_tx((int)rng_in(1u, 64u), rng_pct(50));
        else
            rx(noise_byte(), in_frame_gap(), false);
    }
}

// ---- Valid frames with faults ----

enum fault_kind { NONE, LOST, EXTRA, FLIP, CUT, LENGTH, ORE, ORE_NO_DATA, PAUSE, OWN_TX, FAULTS };

// Sends frame f (len bytes) with a fault and checks what a whole frame must give; returns whether
// the frame went out faulted. after_fault: the frame before this one did.
static bool send_frame(const uint8_t *f, int len, fault_kind fault, bool after_fault)
{
    static uint8_t wire[RX_BUF_SZ + 8];
    memcpy(wire, f, (size_t)len);
    int n = len;
    switch (fault)
    {
    case LOST:
    {
        const int p = (int)rng_in(0u, (uint32_t)(n - 1));
        memmove(wire + p, wire + p + 1, (size_t)(n - p - 1));
        n--;
        break;
    }
    case EXTRA:
    {
        const int p = (int)rng_in(0u, (uint32_t)n);
        memmove(wire + p + 1, wire + p, (size_t)(n - p));
        wire[p] = noise_byte();
        n++;
        break;
    }
    case FLIP:
        for (int k = (int)rng_in(1u, 3u); k > 0; k--)
            wire[rng_in(0u, (uint32_t)(n - 1))] ^= (uint8_t)(1u << (rng_next() % 8u));
        break;
    case CUT:
        n = (int)rng_in(1u, (uint32_t)(n - 1));
        break;
    case LENGTH:
        set_any_length(wire);
        break;
    default:
        break;
    }
    const bool intact = fault < ORE && n == len && memcmp(wire, f, (size_t)len) == 0;
    // Where an ORE (the next byte lost), an ORE without data (this byte lost), a pause or our TX hits.
    const int p = fault == ORE  ? (int)rng_in(0u, (uint32_t)(n - 2))
                  : fault > ORE ? (int)rng_in(1u, (uint32_t)(n - 1))
                                : -1;

    const uint32_t gap0 = between_frames_gap();
    const bool at_rest = parser_at_rest() ||
                         (uint32_t)(tick + gap0 - port.last_rx_tick()) > BUS_RX_RESYNC_GAP_TICKS;
    const bool whole = intact && at_rest;
    uint32_t out0 = 0, out_last = 0;
    int hb0 = 0, hb_last = 0;
    bool room = false;
    for (int i = 0; i < n; i++)
    {
        uint32_t gap = i == 0 ? gap0 : in_frame_gap();
        if (i == p && fault == ORE_NO_DATA)
        {
            rx_overrun();
            continue;
        }
        if (i == p && fault == OWN_TX)
        {
            const int k = (int)rng_in(1u, 40u);
            own_tx(k, rng_pct(50));
            i += k - 1;
            continue;
        }
        if (i == p && fault == PAUSE)
            gap = rng_in(BUS_RX_RESYNC_GAP_TICKS + 1u, 10u * BUS_RX_RESYNC_GAP_TICKS);
        if (i == n - 1)
        {
            if (whole)
                expect(st.out == out0 && heartbeats == hb0, "whole frame ended early");
            out_last = st.out;
            hb_last = heartbeats;
            room = port.recv_data_len == 0;
        }
        rx(wire[i], gap, fault == ORE && i == p);
        if (i == 0)
        {
            out0 = st.out;
            hb0 = heartbeats;
        }
        if (fault == ORE && i == p)
            i++; // the byte after it overran
    }

    const bool heartbeat = is_heartbeat(f, len);
    const bool counted = heartbeats > hb_last;
    const bool out = st.out == out_last + 1u && out_len == len && memcmp(out_copy, f, (size_t)len) == 0;
    if (whole)
    {
        st.whole++;
        if (heartbeat)
            expect(heartbeats == hb_last + 1 && st.out == out0, "whole heartbeat not counted once");
        else if (room)
            expect(out, "whole frame not handed out");
        else
            expect(st.out == out_last, "frame handed out over one the main loop had not taken");
        st.whole_hb += heartbeat ? 1u : 0u;
        st.whole_out += (!heartbeat && room) ? 1u : 0u;
        st.whole_busy += (!heartbeat && !room) ? 1u : 0u;
    }
    if (intact && after_fault && (heartbeat || room))
    {
        st.after_fault++;
        st.after_fault_out += (heartbeat ? counted : out) ? 1u : 0u;
    }
    st.sent++;
    st.faulted += intact ? 0u : 1u;
    return !intact;
}

// Frames of every kind, a third of them faulted, with our reply after some and line glitches.
static void run_frames(uint32_t s, uint32_t frames)
{
    begin_stream(s);
    static uint8_t f[RX_BUF_SZ + 8];
    bool faulted = false;
    for (uint32_t k = 0; k < frames; k++)
    {
        const int len = build_frame(f, (frame_kind)(rng_next() % KINDS));
        const fault_kind fault = rng_pct(65) ? NONE : (fault_kind)rng_in(LOST, FAULTS - 1);
        faulted = send_frame(f, len, fault, faulted);
        if (rng_pct(20))
        {
            tick += 50u * 18u; // delay_us(50) before the reply
            own_tx((int)rng_in(8u, 64u), rng_pct(50));
        }
        if (rng_pct(3))
            for (int i = (int)rng_in(1u, 8u); i > 0; i--)
                rx(noise_byte(), in_frame_gap(), false);
    }
}

static void report(const char *name)
{
    char msg[512];
    snprintf(msg, sizeof(msg),
             "%s: %u bytes, %u frames sent (%u faulted); %u handed out (short %u, long %u, AHUB short %u, "
             "AHUB long %u, %u of %d bytes); whole from rest: %u of %u handed out, %u heartbeats counted, "
             "%u found no room; after a fault: %u of %u recovered; max _index %d, max drop_bytes %d",
             name, (unsigned)st.bytes, (unsigned)st.sent, (unsigned)st.faulted, (unsigned)st.out,
             (unsigned)st.out_kind[0], (unsigned)st.out_kind[1], (unsigned)st.out_kind[2],
             (unsigned)st.out_kind[3], (unsigned)st.out_full, RX_BUF_SZ, (unsigned)st.whole_out,
             (unsigned)st.whole, (unsigned)st.whole_hb, (unsigned)st.whole_busy, (unsigned)st.after_fault_out,
             (unsigned)st.after_fault, st.max_index, st.max_drop);
    TEST_MESSAGE(msg);
}

void setUp(void)
{
    port.init(fake_send);
    heartbeats = 0;
    seen_n = 0;
    out_pending = false;
    memset(&st, 0, sizeof(st));
}

void tearDown(void) {}

static void test_noise_keeps_the_parser_in_bounds(void)
{
    run_noise(1, 250000u, 200u);
    run_noise(2, 250000u, 200u);
    run_noise(3, 250000u, 10u);
    run_noise(4, 250000u, 10u);
    report("noise");
    TEST_ASSERT_TRUE(st.out >= 100u);
    TEST_ASSERT_TRUE(st.out_kind[0] && st.out_kind[1] && st.out_kind[2] && st.out_kind[3]);
    TEST_ASSERT_TRUE(st.out_full >= 1u);
    TEST_ASSERT_EQUAL_INT(RX_BUF_SZ - 1, st.max_index);
}

static void test_faulted_frames_keep_the_parser_in_bounds_and_whole_frames_parse(void)
{
    for (uint32_t s = 101; s <= 104; s++)
        run_frames(s, 8000u);
    report("frames");
    TEST_ASSERT_TRUE(st.whole_out >= 10000u);
    TEST_ASSERT_TRUE(st.whole_hb >= 1000u);
    TEST_ASSERT_TRUE(st.whole_busy >= 100u);
    TEST_ASSERT_TRUE(st.after_fault_out >= 1000u);
    TEST_ASSERT_TRUE(st.out_kind[0] && st.out_kind[1] && st.out_kind[2] && st.out_kind[3]);
    TEST_ASSERT_TRUE(st.out_full >= 1u);
    TEST_ASSERT_EQUAL_INT(RX_BUF_SZ - 1, st.max_index);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_noise_keeps_the_parser_in_bounds);
    RUN_TEST(test_faulted_frames_keep_the_parser_in_bounds_and_whole_frames_parse);
    return UNITY_END();
}
