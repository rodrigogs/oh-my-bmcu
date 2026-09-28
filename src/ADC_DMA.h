#pragma once

#include <stdbool.h>
#include <stdint.h>

void  ADC_DMA_init(void);
bool  ADC_DMA_is_inited(void);

void  ADC_DMA_gpio_analog(void);

void  ADC_DMA_poll(void);
const float* ADC_DMA_get_value(void);

void  ADC_DMA_filter_reset(void);
bool  ADC_DMA_ready(void);
void  ADC_DMA_wait_full(void);

// time_ticks64() ticks since ADC_DMA_poll last processed a half-buffer (since boot before the
// first one): adc_stream.h.
uint64_t ADC_DMA_age_ticks(void);

// Restarts the ADCs and the DMA when the stream is due (adc_stream_restart_due): no half for
// ADC_STREAM_RESTART_MS, and none of its restarts in that time. Nothing if ADC_DMA_init found no data.
void  ADC_DMA_restart_if_stale(void);
