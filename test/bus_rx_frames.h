// Frames for the host tests of the BambuBus RX parser in src/_bus_hardware.h (test_bus_rx_parser,
// test_bus_rx_fuzz). PlatformIO puts test/ on the include path of every suite.
#pragma once

#include <stdint.h>
#include <string.h>

#include "crc_bus.h"

// One 9E1 byte at 1.25 Mbaud in 18 MHz SysTick ticks (158.4).
static const uint32_t BYTE_TICKS = 158u;

// Short-header frame: 3D, flags (bit 7 set), total length, CRC8, payload, CRC16 (little endian).
static inline int make_short(uint8_t *out, uint8_t flags, const uint8_t *payload, int n)
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
