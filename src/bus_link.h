#pragma once
#include <stdbool.h>
#include <stdint.h>

// Host-link liveness of one bus protocol, driven by its heartbeats (hardware-free, host-tested).
//
// A 32-bit tick deadline checked with a signed difference goes back to "not expired" 2^31 ticks
// (119 s at 18 MHz) after the last heartbeat and keeps flipping every 119 s. So the timeout is
// latched instead: once the silence exceeds it, the link stays lost until the next heartbeat, no
// matter how long the silence lasts. bus_link_poll() has to run at least once every 2^31 ticks for
// the latch to engage; the main loop polls on every pass. timeout_ticks must be below 2^31.

typedef enum
{
    BUS_LINK_NOT_SEEN = 0, // no heartbeat since boot
    BUS_LINK_ALIVE,
    BUS_LINK_LOST,         // seen, then silent for longer than the timeout
} bus_link_state_t;

typedef struct
{
    uint32_t last_hb_ticks;
    bool     seen;
    bool     lost;
} bus_link_t;

static inline void bus_link_init(bus_link_t *l)
{
    l->last_hb_ticks = 0u;
    l->seen = false;
    l->lost = false;
}

static inline void bus_link_heartbeat(bus_link_t *l, uint32_t hb_ticks)
{
    l->last_hb_ticks = hb_ticks;
    l->seen = true;
    l->lost = false;
}

static inline bus_link_state_t bus_link_poll(bus_link_t *l, uint32_t now_ticks, uint32_t timeout_ticks)
{
    if (!l->seen) return BUS_LINK_NOT_SEEN;
    if (l->lost) return BUS_LINK_LOST;

    // Signed, so a heartbeat the RX IRQ stamped just after now_ticks was sampled still counts as fresh.
    if ((int32_t)(now_ticks - l->last_hb_ticks) > (int32_t)timeout_ticks)
    {
        l->lost = true;
        return BUS_LINK_LOST;
    }

    return BUS_LINK_ALIVE;
}
