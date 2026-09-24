#pragma once
// Hardware-free frame bookkeeping for one WS2812 strip (host-testable, no ch32v20x includes).
//
// A strip needs a redraw only when what the LEDs show would change: some requested colour
// differs from the one last sent, or the LED contents are unknown (after init/clear).
// Comparing against the last *requested* colour is not enough: Motion_control writes a
// baseline colour and then the state colour for the active channel on every loop pass,
// which kept that strip dirty forever and redrew it (IRQs off ~59 us) at the throttle rate.

#include <stdbool.h>
#include <stdint.h>

#define WS2812_FRAME_MAX_LEDS 4u

typedef struct
{
    uint32_t want[WS2812_FRAME_MAX_LEDS];   // last requested, GRB packed: [23:16]=G, [15:8]=R, [7:0]=B
    uint32_t shown[WS2812_FRAME_MAX_LEDS];  // last sent to the LEDs
    uint8_t  num;
    uint8_t  shown_valid;                   // 0 = LED contents unknown, next update must send
} ws2812_frame_t;

static inline void ws2812_frame_clear(ws2812_frame_t *f)
{
    for (uint32_t i = 0; i < WS2812_FRAME_MAX_LEDS; i++)
    {
        f->want[i]  = 0u;
        f->shown[i] = 0u;
    }
    f->shown_valid = 0u;
}

static inline void ws2812_frame_init(ws2812_frame_t *f, uint8_t num)
{
    if (num > WS2812_FRAME_MAX_LEDS) num = (uint8_t)WS2812_FRAME_MAX_LEDS;
    f->num = num;
    ws2812_frame_clear(f);
}

static inline void ws2812_frame_set(ws2812_frame_t *f, uint8_t index, uint32_t grb)
{
    if (index >= f->num) return;
    f->want[index] = grb;
}

static inline bool ws2812_frame_dirty(const ws2812_frame_t *f)
{
    if (!f->shown_valid) return true;
    for (uint32_t i = 0; i < (uint32_t)f->num; i++)
        if (f->want[i] != f->shown[i]) return true;
    return false;
}

// Call after the whole strip has been sent.
static inline void ws2812_frame_mark_shown(ws2812_frame_t *f)
{
    for (uint32_t i = 0; i < (uint32_t)f->num; i++) f->shown[i] = f->want[i];
    f->shown_valid = 1u;
}
