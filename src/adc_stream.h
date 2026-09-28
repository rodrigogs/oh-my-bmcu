#pragma once
// Freshness of the ADC stream that Motion_control.cpp reads the buffers and filament switches from
// (ADC_DMA.cpp). Hardware-free, so the decision is tested on the host (test/test_adc_stream).
//
// DMA1 channel 1 copies the ADC1/ADC2 pairs (8 channels, 71.5 + 12.5 ADCCLK cycles per conversion
// at 18 MHz) into a circular buffer of two halves of 32 scans each: a half is complete every
// 32 x 8 x 84 = 21504 cycles, 1.19 ms. ADC_DMA_poll(), which ADC_DMA_get_value() runs on every
// main-loop pass, processes each half whose flag is set and stamps it with time_ticks64(). The
// ADCs and the DMA run on their own, so a pass that comes late (a flash erase, 16 ms; the worst NVM
// job, about 36 ms; any stall up to the watchdog) finds the flags set, and the stamp is fresh again
// when Motion_control_run reads the age right after the poll. Only a stream that stopped (the
// conversions, or the DMA channel after a transfer error) leaves the stamp behind, and then
// ADC_DMA_get_value() keeps returning the last readings.
#include <stdbool.h>
#include <stdint.h>

// About 83 halves. Past it, Motion_control_run treats the readings as stale: no channel drives.
#define ADC_STREAM_STALE_MS 100u

// Ticks from the last stamp to now. A stamp taken after now was read counts as age 0. 64-bit: a
// 32-bit tick difference wraps after 238.6 s at 18 MHz, and a dead stream would then read fresh.
static inline uint64_t adc_stream_age_ticks(uint64_t now_ticks, uint64_t last_ticks)
{
    return (now_ticks > last_ticks) ? (now_ticks - last_ticks) : 0u;
}

// True when the readings are older than ADC_STREAM_STALE_MS (ticks_per_ms: time_hw_tpms).
static inline bool adc_stream_stale(uint64_t age_ticks, uint32_t ticks_per_ms)
{
    return age_ticks > (uint64_t)ADC_STREAM_STALE_MS * ticks_per_ms;
}
