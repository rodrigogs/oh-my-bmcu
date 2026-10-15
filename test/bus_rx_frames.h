// Frames and an RX ring model for the host tests of the BambuBus/AHUB RX parser in src/_bus_hardware.h
// (test_bus_rx_parser, test_bus_rx_fuzz). PlatformIO puts test/ on the include path of every suite.
#pragma once

#include <stdint.h>
#include <string.h>

#include "_bus_hardware.h"
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

// Long-header BambuBus frame: 3D, flags (bit 7 clear), package number (2 bytes), total length (2, little
// endian), CRC8 of bytes 0-5, payload, CRC16.
static inline int make_long(uint8_t *out, uint8_t flags, const uint8_t *payload, int n)
{
    const int len = 7 + n + 2;
    out[0] = 0x3D;
    out[1] = (uint8_t)(flags & 0x7Fu);
    out[2] = 0x2A;
    out[3] = 0x00;
    out[4] = (uint8_t)(len & 0xFF);
    out[5] = (uint8_t)(len >> 8);
    out[6] = bus_crc8(out, 6);
    memcpy(out + 7, payload, (size_t)n);
    const uint16_t crc = bus_crc16(out, (uint32_t)(len - 2));
    out[len - 2] = (uint8_t)(crc & 0xFFu);
    out[len - 1] = (uint8_t)(crc >> 8);
    return len;
}

// An AHUB frame of header length `length`: length * 4 + 12 bytes. Short header (flag bit 7 set): 33,
// flag, length, CRC8. Long header (flag bit 7 clear), as the RX parser reads it: length at byte 4, CRC8
// of bytes 0-5 at byte 6, byte 2 free (b2). The rest is filler of 0x40-0x7F (never a header byte); the
// parser does not check the CRC32 word (ahub_bus.cpp does).
static inline int make_ahub(uint8_t *out, uint8_t flag, int length, uint8_t b2)
{
    const int len = length * 4 + 12;
    for (int i = 0; i < len; i++)
        out[i] = (uint8_t)(0x40u + ((i * 7) & 0x3Fu));
    out[0] = 0x33;
    out[1] = flag;
    if (flag & 0x80u)
    {
        out[2] = (uint8_t)length;
        out[3] = bus_crc8(out, 3);
    }
    else
    {
        out[2] = b2;
        out[4] = (uint8_t)length;
        out[6] = bus_crc8(out, 6);
    }
    return len;
}

// The RX side of _bus_hardware.cpp around the parser: USART1 RX goes by DMA into a ring, and the main
// loop's bus_rx_poll() feeds the new bytes to bus_port_to_host.irq(). Here the ring is an array the test
// fills (bus_rx_model_arrive) and the poll takes the test's clock.
#define BUS_RX_MODEL_RING_SIZE 2048u // RX_SIZE in _bus_hardware.cpp

struct bus_rx_model
{
    _bus_port_deal *port;
    uint8_t ring[BUS_RX_MODEL_RING_SIZE];
    uint32_t produced;      // rx_produced(): bytes the DMA wrote, as an absolute count
    uint32_t consumed;      // rx_consumed
    uint32_t last_progress; // rx_last_progress
};

// bus_uart1_init(): an empty ring, the stall clock started.
static inline void bus_rx_model_init(bus_rx_model *m, _bus_port_deal *port, uint32_t now)
{
    m->port = port;
    m->produced = m->consumed = 0u;
    m->last_progress = now;
}

// A byte from the line lands in the ring. The tests poll before it could overflow (the real ring then
// sets rx_fault and restarts).
static inline void bus_rx_model_arrive(bus_rx_model *m, uint8_t b)
{
    m->ring[m->produced & (BUS_RX_MODEL_RING_SIZE - 1u)] = b;
    ++m->produced;
}

static inline uint8_t bus_rx_model_peek(const bus_rx_model *m, uint32_t absolute_index)
{
    return m->ring[absolute_index & (BUS_RX_MODEL_RING_SIZE - 1u)];
}

// ---- adapted from _bus_hardware.cpp: bus_uart1_dma_send()'s RX-off and TX mark (RX hears nothing until the TX ends) ----
// ---- anchor: bus_uart1_dma_send from /USART_Mode_Rx/ to /rx_mark_tx/ ----
static inline void bus_rx_model_tx_start(bus_rx_model *m)
{
    m->port->rx_mark_tx(m->produced);
}

// ---- adapted from _bus_hardware.cpp: bus_rx_poll()'s ring consumer, heartbeat skip, TX mark and stall reset (no TX state machine, rx_fault or overflow check; RXNE never set) ----
// ---- anchor: bus_rx_poll from /if \(produced != rx_consumed\)/ to /\+\+bus_port_to_host.rx_dropped;/ ----
static inline void bus_rx_model_poll(bus_rx_model *m, uint32_t now)
{
    _bus_port_deal &port = *m->port;
    const uint32_t produced = m->produced;
    if (produced != m->consumed) m->last_progress = now;
    while (m->consumed != produced)
    {
        port.rx_reached(m->consumed);
        // skip heartbeat
        const uint32_t available = (uint32_t)(produced - m->consumed);
        if (available >= 5u &&
            bus_rx_model_peek(m, m->consumed + 0u) == 0x3Du &&
            bus_rx_model_peek(m, m->consumed + 1u) == 0xC5u)
        {
            const uint32_t hb_len = bus_rx_model_peek(m, m->consumed + 2u);
            if (hb_len >= 6u && hb_len <= 255u &&
                bus_rx_model_peek(m, m->consumed + 4u) == 0x20u &&
                available >= hb_len &&
                !port.rx_mark_inside(m->consumed, hb_len))
            {
                m->consumed += hb_len;
                bambubus_heartbeat_seen_fast();
                continue;
            }
        }
        const uint8_t data = bus_rx_model_peek(m, m->consumed);
        ++m->consumed;
        port.irq(data);
    }
    port.rx_reached(m->consumed);
    if (port.receiving() &&
        (uint32_t)(now - m->last_progress) >= BUS_RX_RESYNC_GAP_TICKS &&
        m->consumed == m->produced)
    {
        port.reset_rx();
        ++port.rx_dropped;
    }
}
