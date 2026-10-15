// Fuzz of the BambuBus/AHUB RX frame parser in src/_bus_hardware.h (_bus_port_deal::irq) with the
// main-loop RX path around it (bus_rx_poll(), modelled in bus_rx_frames.h): seeded xorshift streams
// into the real parser, of noise and of valid frames with faults injected (a lost byte, an extra byte,
// bit flips, a frame cut short, a header that claims another length, a pause on the line, our own TX
// cutting in, a ring restart), with quiet gaps and SysTick wraps in between. The main loop polls once
// before every byte arrives and once after (so the poll takes one byte at a time and the heartbeat
// fast path, which needs 5 bytes in the ring, stays out; test_bus_rx_parser covers it). After every
// event the parser's state stays in bounds, the queue holds at most two frames, each one the frame its
// header describes (7-1280 bytes short, 15-1280 long, 12-1032 AHUB, a good CRC8, not a heartbeat) and
// made of the last bytes read, left alone until the main loop releases it; a whole frame that starts
// with the parser at rest (or after a quiet gap) is always queued when there is room, dropped and
// counted when there is not (a heartbeat counted once); the parser is at rest once the poll after our
// TX has reached the TX mark. The hand-written scenarios are in test_bus_rx_parser.

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "_bus_hardware.h"
#include "bus_rx_frames.h"
#include "crc_bus.h"

// Size of one of the parser's three receive buffers; the accessor below stops compiling if it changes.
static const int RX_BUF_SZ = 1280;
typedef uint8_t rx_bufs_t[3][RX_BUF_SZ];

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
struct rx_read_of { typedef uint8_t _bus_port_deal::*type; };
struct rx_write_of { typedef uint8_t _bus_port_deal::*type; };
struct rx_count_of { typedef uint8_t _bus_port_deal::*type; };
struct rx_length_of { typedef uint16_t (_bus_port_deal::*type)[3]; };
struct rx_type_of { typedef _bus_data_type (_bus_port_deal::*type)[3]; };
index_of::type member(index_of);
drop_of::type member(drop_of);
irq_buf_of::type member(irq_buf_of);
rx_bufs_of::type member(rx_bufs_of);
rx_read_of::type member(rx_read_of);
rx_write_of::type member(rx_write_of);
rx_count_of::type member(rx_count_of);
rx_length_of::type member(rx_length_of);
rx_type_of::type member(rx_type_of);
template struct private_member<index_of, &_bus_port_deal::_index>;
template struct private_member<drop_of, &_bus_port_deal::heartbeat_drop_bytes>;
template struct private_member<irq_buf_of, &_bus_port_deal::bus_irq_data_ptr>;
template struct private_member<rx_bufs_of, &_bus_port_deal::recv_data_buf>;
template struct private_member<rx_read_of, &_bus_port_deal::rx_read>;
template struct private_member<rx_write_of, &_bus_port_deal::rx_write>;
template struct private_member<rx_count_of, &_bus_port_deal::rx_count>;
template struct private_member<rx_length_of, &_bus_port_deal::rx_length>;
template struct private_member<rx_type_of, &_bus_port_deal::rx_type>;

static _bus_port_deal port;
static bus_rx_model ring;
static uint32_t tick;
static int heartbeats;

void bambubus_heartbeat_seen_fast(void) { heartbeats++; }
static bool fake_send(uint8_t *, uint16_t) { return true; }

static int parser_index(void) { return port.*member(index_of()); }
static int parser_drop(void) { return port.*member(drop_of()); }
static int queued(void) { return port.*member(rx_count_of()); }

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
    uint32_t out, out_kind[4], out_full; // frames queued: all, by header, of RX_BUF_SZ bytes
    uint32_t full_drops;                 // completed frames dropped because two were queued
    int max_index, max_drop, max_queued;
    uint32_t sent, faulted, tx_cuts;
    uint32_t whole, whole_out, whole_busy, whole_hb; // intact from rest: queued, no room, heartbeats
    uint32_t after_fault, after_fault_out;           // intact right after a faulted one: queued/counted
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

// The bytes the parser read, newest last.
static uint8_t seen[2048];
static uint32_t seen_n;

// The frames queued, as they were when queued, oldest first; the main loop releases the head `hold`
// events after it reached the head: at once mostly, sometimes only after more frames went by (a slow
// pass), which then find the queue full.
static uint8_t q_copy[2][RX_BUF_SZ];
static int q_len[2];
static int q_n;
static uint32_t hold;

// The last frame queued, kept after the main loop released it.
static uint8_t last_copy[RX_BUF_SZ];
static int last_len;

// Where the header's CRC8 sits and the total length it claims, as the parser reads them.
static int crc8_at(const uint8_t *f) { return (f[1] & 0x80u) ? 3 : 6; }
static int claimed_length(const uint8_t *f)
{
    const bool short_head = (f[1] & 0x80u) != 0;
    if (f[0] == 0x3D)
        return short_head ? f[2] : (f[4] | (f[5] << 8));
    return ((short_head ? f[2] : f[4]) << 2) + 12;
}
static int minimum_length(const uint8_t *f)
{
    return f[0] == 0x33 ? 12 : ((f[1] & 0x80u) ? 7 : 15);
}

// Dropped by counting and reported through bambubus_heartbeat_seen_fast(), never queued.
static bool is_heartbeat(const uint8_t *f, int len)
{
    return len >= 7 && f[0] == 0x3D && f[1] == 0xC5 && f[4] == 0x20;
}

static void new_hold(void) { hold = rng_pct(70) ? 0u : rng_pct(85) ? rng_in(1u, 40u) : rng_in(41u, 1500u); }

// The parser just queued frame f of len bytes in the slot after the q_n already queued.
static void new_frame(const uint8_t *f, int len, uint8_t type)
{
    expect(f[0] == 0x3D || f[0] == 0x33, "frame without a header byte");
    expect(type == f[0], "frame type differs from its header byte");
    expect(len == claimed_length(f), "frame length differs from its header");
    expect(len >= minimum_length(f) && len <= RX_BUF_SZ, "frame shorter than its minimum or over the buffer");
    expect(f[crc8_at(f)] == bus_crc8(f, (uint32_t)crc8_at(f)), "frame header CRC8 is bad");
    expect(!is_heartbeat(f, len), "heartbeat queued");
    bool last_read = seen_n >= (uint32_t)len;
    for (int i = 0; i < len && last_read; i++)
        last_read = f[i] == seen[(seen_n - (uint32_t)len + (uint32_t)i) % sizeof(seen)];
    expect(last_read, "frame is not the last bytes read");
    expect(q_n < 2, "a third frame queued");

    memcpy(q_copy[q_n], f, (size_t)len);
    q_len[q_n] = len;
    memcpy(last_copy, f, (size_t)len);
    last_len = len;
    if (q_n == 0)
        new_hold();
    q_n++;

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
    const int n = queued();
    const int rd = port.*member(rx_read_of());
    const int wr = port.*member(rx_write_of());
    const uint8_t *irq_buf = port.*member(irq_buf_of());
    rx_bufs_t &bufs = port.*member(rx_bufs_of());
    const uint16_t(&lengths)[3] = port.*member(rx_length_of());

    expect(idx >= 0 && idx < RX_BUF_SZ, "_index out of the buffer");
    expect(drop >= 0 && drop <= 255 - 5, "heartbeat_drop_bytes out of range"); // a short length is one byte
    expect(n == q_n, "queue length differs from the frames queued");
    expect(rd < 3 && wr < 3 && wr == (rd + n) % 3, "queue indices inconsistent");
    expect(irq_buf == bufs[wr], "bus_irq_data_ptr is not the free slot");
    expect(port.bus_recv_data_ptr == bufs[rd], "bus_recv_data_ptr is not the queue head");
    expect(port.recv_data_len == (n ? lengths[rd] : 0), "recv_data_len is not the head's length");
    for (int k = 0; k < n; k++)
    {
        const int slot = (rd + k) % 3;
        expect(lengths[slot] == q_len[k] && memcmp(bufs[slot], q_copy[k], (size_t)q_len[k]) == 0,
               "queued frame changed before the main loop released it");
    }
    if (idx > st.max_index)
        st.max_index = idx;
    if (drop > st.max_drop)
        st.max_drop = drop;
    if (n > st.max_queued)
        st.max_queued = n;
}

static void main_loop(void)
{
    if (q_n == 0)
        return;
    if (hold)
    {
        hold--;
        return;
    }
    port.release_packet();
    q_n--;
    memmove(q_copy[0], q_copy[1], (size_t)q_len[1]);
    q_len[0] = q_len[1];
    if (q_n)
        new_hold();
    check_state();
}

// The parser is between frames: a reset leaves it here.
static bool parser_at_rest(void) { return parser_index() == 0 && parser_drop() == 0; }

// The main loop polls with nothing new in the ring (the line quiet since the last byte). A gap of
// BUS_RX_RESYNC_GAP_TICKS or more there drops the frame in progress.
static void idle_poll(void)
{
    const uint32_t quiet = (uint32_t)(tick - ring.last_progress);
    bus_rx_model_poll(&ring, tick);
    if (quiet >= BUS_RX_RESYNC_GAP_TICKS)
        expect(parser_at_rest(), "parser not reset by a quiet line");
}

// One byte `gap` ticks after the last: the main loop polls just before it arrives, then takes it.
static void rx(uint8_t b, uint32_t gap)
{
    tick += gap;
    idle_poll();
    const int n0 = queued();
    const int wr = port.*member(rx_write_of());
    const uint32_t dropped0 = port.rx_dropped;
    seen[seen_n++ % sizeof(seen)] = b;
    bus_rx_model_arrive(&ring, b);
    bus_rx_model_poll(&ring, tick);
    st.bytes++;
    if (queued() == n0 + 1)
        new_frame((port.*member(rx_bufs_of()))[wr], (port.*member(rx_length_of()))[wr],
                  (uint8_t)(port.*member(rx_type_of()))[wr]);
    else
        expect(queued() == n0, "queue changed without a frame");
    if (port.rx_dropped != dropped0)
    {
        // Neither a stall nor the TX mark can fire in the poll that took a new byte (the idle poll
        // before it reached the mark and stamped the progress), so only a full queue drops here.
        expect(n0 == 2 && port.rx_dropped == dropped0 + 1u, "frame dropped with room in the queue");
        st.full_drops++;
    }
    check_state();
    main_loop();
}

// Our reply of n bytes. bus_uart1_dma_send() turns RX off and marks the ring position; what the printer
// sent meanwhile is lost. The main loop polls during and after the TX: the parser is at rest then.
static void own_tx(int n)
{
    const bool was_receiving = port.receiving();
    port.send_data_len = n;
    port.send_package();
    expect(port.send_data_len == 0, "reply not started");
    bus_rx_model_tx_start(&ring);
    tick += (uint32_t)n * BYTE_TICKS;
    bus_rx_model_poll(&ring, tick);
    expect(parser_at_rest(), "parser not at rest after our TX's mark");
    st.tx_cuts += was_receiving ? 1u : 0u;
    check_state();
    main_loop();
}

// rx_fault (ring overflow, DMA error, TX abort): bus_rx_poll() restarts the ring and resets the parser.
static void ring_restart(void)
{
    port.reset_rx();
    ++port.rx_dropped;
    bus_rx_model_init(&ring, &port, tick);
    expect(parser_at_rest(), "parser not reset by a ring restart");
    check_state();
}

// From one byte to the next inside a frame: mostly back to back, now and then a pause shorter than
// the quiet gap, which must not split the frame.
static uint32_t in_frame_gap(void)
{
    const uint32_t r = rng_next() % 1000u;
    if (r < 950u)
        return rng_in(BYTE_TICKS, 2u * BYTE_TICKS);
    if (r < 995u)
        return rng_in(2u * BYTE_TICKS, BUS_RX_RESYNC_GAP_TICKS - 1u);
    return BUS_RX_RESYNC_GAP_TICKS - 1u;
}

// Before a frame: mostly a quiet line, else back to back, one tick short of the quiet gap or any
// SysTick delta.
static uint32_t between_frames_gap(void)
{
    const uint32_t r = rng_next() % 100u;
    if (r < 75u)
        return rng_in(BUS_RX_RESYNC_GAP_TICKS, 20u * BUS_RX_RESYNC_GAP_TICKS);
    if (r < 90u)
        return in_frame_gap();
    if (r < 95u)
        return BUS_RX_RESYNC_GAP_TICKS - 1u;
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

// A header whose CRC8 is good but whose length is any the field can hold, around the minima, the
// buffer size and the header end most of all.
static void set_any_length(uint8_t *f)
{
    const bool short_head = (f[1] & 0x80u) != 0;
    if (f[0] == 0x3D && !short_head)
    {
        static const int edge[] = {0, 5, 6, 7, 14, 15, RX_BUF_SZ - 1, RX_BUF_SZ, RX_BUF_SZ + 1, 0xFFFF};
        const int len = rng_pct(30)   ? edge[rng_next() % 10u]
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
        return make_short(f, flags, payload, (int)(rng_pct(90) ? rng_in(1u, 40u) : rng_in(41u, 249u)));
    }
    case HEARTBEAT:
        payload[0] = 0x20;
        return make_short(f, 0xC5, payload, (int)rng_in(1u, 40u));
    case LONG:
    {
        const int r = (int)(rng_next() % 100u);
        const int len = r < 80   ? (int)rng_in(15u, 120u)
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
    bus_rx_model_init(&ring, &port, tick);
}

// ---- Noise ----

// Noise, with a header of good CRC8 and any length now and then. breaks: per 100000 events, how often
// each of a quiet line, our own TX and a ring restart cuts in; the streams with few breaks let false
// frames grow to the buffer size.
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
                rx(f[i], in_frame_gap());
        }
        else if (r < 3000u + breaks)
            rx(noise_byte(), between_frames_gap());
        else if (r < 3000u + 2u * breaks)
            own_tx((int)rng_in(1u, 64u));
        else if (r < 3000u + 2u * breaks + breaks / 10u)
            ring_restart();
        else
            rx(noise_byte(), in_frame_gap());
    }
}

// ---- Valid frames with faults ----

enum fault_kind { NONE, LOST, EXTRA, FLIP, CUT, LENGTH, PAUSE, OWN_TX, RESTART, FAULTS };

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
    const bool intact = fault < PAUSE && n == len && memcmp(wire, f, (size_t)len) == 0;
    // Where a pause, our TX or a ring restart hits.
    const int p = fault >= PAUSE ? (int)rng_in(1u, (uint32_t)(n - 1)) : -1;

    const uint32_t gap0 = between_frames_gap();
    const bool at_rest = parser_at_rest() ||
                         (uint32_t)(tick + gap0 - ring.last_progress) >= BUS_RX_RESYNC_GAP_TICKS;
    const bool whole = intact && at_rest;
    uint32_t out0 = 0, out_last = 0, drops_last = 0;
    int hb0 = 0, hb_last = 0;
    bool room = false;
    for (int i = 0; i < n; i++)
    {
        uint32_t gap = i == 0 ? gap0 : in_frame_gap();
        if (i == p && fault == OWN_TX)
        {
            const int k = (int)rng_in(1u, 40u);
            own_tx(k);
            i += k - 1;
            continue;
        }
        if (i == p && fault == RESTART)
            ring_restart();
        if (i == p && fault == PAUSE)
            gap = rng_in(BUS_RX_RESYNC_GAP_TICKS, 10u * BUS_RX_RESYNC_GAP_TICKS);
        if (i == n - 1)
        {
            if (whole)
                expect(st.out == out0 && heartbeats == hb0, "whole frame ended early");
            out_last = st.out;
            drops_last = st.full_drops;
            hb_last = heartbeats;
            room = queued() < 2;
        }
        rx(wire[i], gap);
        if (i == 0)
        {
            out0 = st.out;
            hb0 = heartbeats;
        }
    }

    const bool heartbeat = is_heartbeat(f, len);
    const bool counted = heartbeats > hb_last;
    const bool out = st.out == out_last + 1u && last_len == len && memcmp(last_copy, f, (size_t)len) == 0;
    if (whole)
    {
        st.whole++;
        if (heartbeat)
            expect(heartbeats == hb_last + 1 && st.out == out0, "whole heartbeat not counted once");
        else if (room)
            expect(out, "whole frame not queued");
        else
            expect(st.out == out_last && st.full_drops == drops_last + 1u,
                   "frame over a full queue not dropped and counted");
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

// Frames of every kind, 35% of them faulted (rng_pct(65) keeps 65% intact), with our reply after
// some and line glitches.
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
            own_tx((int)rng_in(8u, 64u));
        }
        if (rng_pct(3))
            for (int i = (int)rng_in(1u, 8u); i > 0; i--)
                rx(noise_byte(), in_frame_gap());
    }
}

static void report(const char *name)
{
    char msg[600];
    snprintf(msg, sizeof(msg),
             "%s: %u bytes, %u frames sent (%u faulted); %u queued (short %u, long %u, AHUB short %u, "
             "AHUB long %u, %u of %d bytes), %u dropped on a full queue; whole from rest: %u of %u queued, "
             "%u heartbeats counted, %u found no room; after a fault: %u of %u recovered; %u frames cut by "
             "our TX; max _index %d, max heartbeat_drop_bytes %d, max queued %d",
             name, (unsigned)st.bytes, (unsigned)st.sent, (unsigned)st.faulted, (unsigned)st.out,
             (unsigned)st.out_kind[0], (unsigned)st.out_kind[1], (unsigned)st.out_kind[2],
             (unsigned)st.out_kind[3], (unsigned)st.out_full, RX_BUF_SZ, (unsigned)st.full_drops,
             (unsigned)st.whole_out, (unsigned)st.whole, (unsigned)st.whole_hb, (unsigned)st.whole_busy,
             (unsigned)st.after_fault_out, (unsigned)st.after_fault, (unsigned)st.tx_cuts, st.max_index,
             st.max_drop, st.max_queued);
    TEST_MESSAGE(msg);
}

void setUp(void)
{
    port.init(fake_send);
    port.rx_dropped = 0;
    heartbeats = 0;
    seen_n = 0;
    q_n = 0;
    hold = 0;
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
    TEST_ASSERT_TRUE(st.tx_cuts >= 10u);
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
    TEST_ASSERT_TRUE(st.tx_cuts >= 100u);
    TEST_ASSERT_EQUAL_INT(2, st.max_queued);
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
