#pragma once
// When the main loop may write the filament-info and loaded-channel journals (Flash_saves.cpp).
// Hardware-free, so the decisions are tested on the host (test/test_nvm_save_sched).
//
// Every flash erase/program runs with IRQs off. While they are off the USART1 TC interrupt cannot
// release DE after our reply (we keep driving the bus) and RX bytes overrun the single-byte DATAR.
// So NVM work never runs in the loop pass that starts a reply: it waits for a quiet bus window.
#include <stdint.h>
#include "_bus_hardware.h"

// SysTick ticks (HCLK/8 = 18 MHz), as for BUS_RX_RESYNC_GAP_TICKS.
#define NVM_TICKS_PER_MS 18000u

// Quiet window: nothing on the line in either direction for 1 ms (about 114 byte times). That is
// far longer than any gap inside a frame (at most 2 byte times) or the RX resync gap
// (BUS_RX_RESYNC_GAP_TICKS, 200 us), and longer than a device's turnaround (this BMCU answers within
// one main-loop pass, under 1 ms, plus 50 us). So no request/reply exchange is in progress and none has just ended: a request the
// printer sends right after our reply (next poll, read-back of a slot) finds the bus free instead
// of landing inside a flash operation.
#define NVM_BUS_QUIET_TICKS (1u * NVM_TICKS_PER_MS)

// The printer's poll timing is not measured. If it never leaves a 1 ms gap, a job that has waited
// this long settles for "no frame on the line": quiet for more than the RX resync gap. That still
// never overlaps our TX or a frame being received, and the write is never held back for good.
#define NVM_BUS_QUIET_MAX_WAIT_TICKS (1000u * NVM_TICKS_PER_MS)
#define NVM_BUS_QUIET_SHORT_TICKS (BUS_RX_RESYNC_GAP_TICKS + 1u)

// Filament info is written once its slot has not changed for 500 ms, so a burst of
// set_filament_info for one slot costs one journal record (the page is erased every 6 records).
// A slot that keeps changing is still written 5 s after its first unsaved change.
#define NVM_FIL_SETTLE_TICKS (500u * NVM_TICKS_PER_MS)
#define NVM_FIL_MAX_DELAY_TICKS (5000u * NVM_TICKS_PER_MS)

// A write that failed is retried after this long, not in every quiet pass.
#define NVM_RETRY_TICKS (500u * NVM_TICKS_PER_MS)

// nvm_pick_job() result: a filament slot 0..3, the loaded-channel state, or nothing.
#define NVM_JOB_NONE (-1)
#define NVM_JOB_STATE 4

// One journal whose RAM copy may differ from flash. Ticks are compared as unsigned differences, so
// they survive the 2^32 SysTick wrap.
struct nvm_job
{
    uint8_t pending; // RAM changed since the last successful write
    uint8_t failed;  // the last write failed: wait NVM_RETRY_TICKS
    uint32_t first;  // tick of the first change since the last successful write
    uint32_t last;   // tick of the latest change or failed write
};

// How long the due job has waited for a quiet window.
struct nvm_wait
{
    uint8_t active;
    uint8_t expired; // latched, so waits past the 2^32 wrap stay expired
    uint32_t since;
};

// Bus state sampled by the main loop, built with nvm_bus_from_samples().
struct nvm_bus
{
    bool tx_idle;    // _bus_port_deal::tx_idle()
    bool rx_idle;    // _bus_port_deal::rx_idle()
    uint32_t rx_gap; // ticks since the last RX byte
    uint32_t tx_gap; // ticks since our last TX ended
};

// Ticks from `stamp` to `now`. The ISRs write the stamps, so one can land just after `now` was
// read: that byte (or TX end) is happening right now, a gap of 0, not a wrap to about 2^32 that
// would pass the quiet check. Only stamps up to one quiet window ahead count as "now", so a stamp
// older than 2^31 ticks still reads as a long gap.
static inline uint32_t nvm_gap(uint32_t now, uint32_t stamp)
{
    return ((uint32_t)(stamp - now) <= NVM_BUS_QUIET_TICKS) ? 0u : (uint32_t)(now - stamp);
}

// Sample rx_idle and tx_idle first, then read `now`, then the stamps: a byte that arrives after the
// idle checks then shows up as a zero or short gap.
static inline nvm_bus nvm_bus_from_samples(bool rx_idle, bool tx_idle, uint32_t now,
                                           uint32_t last_rx_tick, uint32_t last_tx_end_tick)
{
    nvm_bus b;
    b.tx_idle = tx_idle;
    b.rx_idle = rx_idle;
    b.rx_gap = nvm_gap(now, last_rx_tick);
    b.tx_gap = nvm_gap(now, last_tx_end_tick);
    return b;
}

static inline void nvm_job_changed(nvm_job *j, uint32_t now)
{
    if (!j->pending) j->first = now;
    j->pending = 1u;
    j->failed = 0u; // new data: scheduled by the normal rule, not held back by a retry delay
    j->last = now;
}

static inline bool nvm_job_due(const nvm_job *j, uint32_t now, uint32_t settle, uint32_t max_delay)
{
    if (!j->pending) return false;
    if (j->failed) return (uint32_t)(now - j->last) >= NVM_RETRY_TICKS;
    return (uint32_t)(now - j->last) >= settle || (uint32_t)(now - j->first) >= max_delay;
}

static inline void nvm_job_result(nvm_job *j, bool ok, uint32_t now)
{
    if (ok)
    {
        j->pending = 0u;
        j->failed = 0u;
        return;
    }
    j->failed = 1u;
    j->last = now;
}

static inline bool nvm_bus_window(const nvm_bus *b, bool waited_long)
{
    if (!b->tx_idle || !b->rx_idle) return false;
    const uint32_t need = waited_long ? NVM_BUS_QUIET_SHORT_TICKS : NVM_BUS_QUIET_TICKS;
    return b->rx_gap >= need && b->tx_gap >= need;
}

// The job to run now, at most one per main-loop pass so each starts in its own window. The
// loaded-channel state (power-loss resume) goes first and is not debounced; then the lowest
// filament slot that is due, so a slot waiting to retry does not hold back the others.
static inline int nvm_pick_job(const nvm_job *state, const nvm_job fil[4], nvm_wait *w,
                               const nvm_bus *b, uint32_t now)
{
    int job = NVM_JOB_NONE;
    if (nvm_job_due(state, now, 0u, 0u))
    {
        job = NVM_JOB_STATE;
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            if (nvm_job_due(&fil[i], now, NVM_FIL_SETTLE_TICKS, NVM_FIL_MAX_DELAY_TICKS))
            {
                job = i;
                break;
            }
        }
    }

    if (job == NVM_JOB_NONE)
    {
        w->active = 0u;
        return NVM_JOB_NONE;
    }

    if (!w->active)
    {
        w->active = 1u;
        w->expired = 0u;
        w->since = now;
    }
    if ((uint32_t)(now - w->since) >= NVM_BUS_QUIET_MAX_WAIT_TICKS)
        w->expired = 1u;

    if (!nvm_bus_window(b, w->expired != 0u))
        return NVM_JOB_NONE;

    w->active = 0u;
    return job;
}
