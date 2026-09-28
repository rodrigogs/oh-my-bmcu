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

// True once an ADC calibration wait in ADC_DMA_init gave up.
bool  ADC_DMA_cal_timed_out(void);
