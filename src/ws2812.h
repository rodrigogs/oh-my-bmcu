#pragma once
#include <stdint.h>
#include "ch32v20x.h"
#include "ws2812_frame.h"

#ifndef BMCU_ONLINE_LED_FILAMENT_RGB
#define BMCU_ONLINE_LED_FILAMENT_RGB 0
#endif

class WS2812_class
{
public:
    static constexpr uint8_t MAX_NUM = WS2812_FRAME_MAX_LEDS;

    void init(uint8_t num, GPIO_TypeDef* port, uint16_t pin);

    void clear(void);
    void updata(void);

    void set_RGB(uint8_t R, uint8_t G, uint8_t B, uint8_t index);

    void set_RGB_online(uint8_t R, uint8_t G, uint8_t B, uint8_t index, bool filament = false);

    // True only when the LEDs would show something different (see ws2812_frame.h).
    inline bool is_dirty() const { return ws2812_frame_dirty(&frame); }

private:
    GPIO_TypeDef* port = nullptr;
    uint16_t      pin  = 0;

    // requested vs. shown GRB per LED, and the LED count
    ws2812_frame_t frame = {};

    // cache tylko pod ONLINE/filament (porównujemy surowe RGB)
    uint32_t last_online_raw_rgb[MAX_NUM]   = {0u, 0u, 0u, 0u}; // RGB packed
    uint8_t  last_online_is_filament[MAX_NUM] = {0u, 0u, 0u, 0u};
};
