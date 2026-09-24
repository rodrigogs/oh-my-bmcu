#pragma once
#include "crc_bus.h"
#include <string.h>

enum class _bus_data_type : uint8_t
{
    bambubus = 0x3D,
    ahub_bus = 0x33,
    none = 0x00
};

void bambubus_heartbeat_seen_fast(void);

// RX parser resync gap in SysTick ticks (HCLK/8 = 18 MHz). One 9E1 byte at 1.25 Mbaud is 11 bits =
// 8.8 us = 158.4 ticks. Inside a frame the BMCU sees RX interrupts at most 2 byte times apart (a
// longer ISR delay overruns the single-byte DATAR and is flagged by ORE). Between frames the bus
// stays quiet much longer: a device must receive and check the whole request before replying (this
// BMCU then waits another 50 us), and a broken request gets no reply, so the next frame waits for the
// printer's timeout. The printer's own inter-byte timing has not been measured, so the threshold is
// 200 us (about 23 byte times): generous for pauses on the sending side, still well below the reply
// timeout that follows a lost frame.
#define BUS_RX_RESYNC_GAP_TICKS 3600u

class _bus_port_deal // 中断数据处理
{
public:
    uint8_t send_data_buf[1280] __attribute__((aligned(4)));

private:
    uint8_t tx_dma_buf[1280] __attribute__((aligned(4)));
    uint8_t recv_data_buf[2][1280] __attribute__((aligned(4)));
    uint8_t tx_build_sel = 0;
    int _index = 0;
    int length = 999;
    uint8_t data_length_index = 0;
    uint8_t data_CRC8_index = 0;
    _bus_data_type irq_package_type = _bus_data_type::none;
    uint8_t *bus_irq_data_ptr = recv_data_buf[0];
    int drop_bytes = 0;
    volatile uint32_t rx_last_tick = 0;
    volatile uint32_t tx_end_tick = 0;
    void (*port_send_datas)(uint8_t *data, uint16_t len); // starts the TX; only tx_start() calls it

    // Drop the frame (or heartbeat skip) in progress. Also runs in the main loop at TX start, so the
    // ISR-owned fields go through volatile: kept after the `idle = false` store in tx_start().
    void rx_resync()
    {
        volatile int &drop = *(volatile int *)&drop_bytes;
        // A heartbeat whose header passed CRC8 counts even if its tail was cut, as it did before.
        if (drop > 0)
            bambubus_heartbeat_seen_fast();
        *(volatile int *)&_index = 0;
        drop = 0;
    }

    // Start of our TX (main loop). The printer waits for our reply, so normally no frame is arriving
    // and the reset does nothing. If one did start first (we answered late), our TX garbles it and RX
    // ignores its bytes meanwhile, so its head must not combine with the bytes after our TX. The gap
    // resync misses that when RX echoes our own bytes (they are timestamped) or the reply is shorter
    // than BUS_RX_RESYNC_GAP_TICKS (the 8-byte set_filament ACK takes 70 us). idle goes false first:
    // from then on the RX ISR leaves the parser alone, so the reset cannot race with it (at worst
    // rx_overrun() reports the same cut heartbeat again, which only refreshes its stamp).
    void tx_start(uint8_t *data, uint16_t len)
    {
        idle = false;
        rx_resync();
        port_send_datas(data, len);
    }

public:
    uint8_t * volatile bus_recv_data_ptr = recv_data_buf[0];
    volatile int recv_data_len = 0;
    volatile int send_data_len = 0;
    volatile _bus_data_type bus_package_type = _bus_data_type::none;
    volatile bool idle = true;

    inline __attribute__((always_inline)) uint8_t* tx_build_buf()
    {
        return tx_build_sel ? tx_dma_buf : send_data_buf;
    }

    void init(void (*_port_send_datas)(uint8_t *data, uint16_t len))
    {
        _index = 0;
        length = 999;
        data_length_index = 0;
        data_CRC8_index = 0;
        irq_package_type = _bus_data_type::none;
        bus_irq_data_ptr = recv_data_buf[0];
        drop_bytes = 0;
        rx_last_tick = 0;
        tx_end_tick = 0;
        bus_recv_data_ptr = recv_data_buf[1];
        idle = true;
        send_data_len = 0;
        recv_data_len = 0;
        tx_build_sel  = 0;
        port_send_datas = _port_send_datas;
    }

    // RX ISR entry, once per byte read from DATAR. now: STK_CNTL at ISR entry. overrun: ORE was set,
    // so this byte is good but at least one byte after it was lost. PE/FE/NE are not acted on: the
    // configured framing was never checked against the printers, and CRC16 rejects bad frames anyway.
    void rx_byte(uint8_t data, uint32_t now, bool overrun)
    {
        const uint32_t gap = now - rx_last_tick;
        rx_last_tick = now;
        if (!idle) return; // our own TX, never host data

        // Drop the frame (or heartbeat skip) in progress once the line went quiet; otherwise a frame
        // that lost bytes swallows the head of the next one.
        if (gap > BUS_RX_RESYNC_GAP_TICKS)
            rx_resync();

        irq(data);
        if (overrun)
            rx_resync();
    }

    // ORE without a pending byte: a byte was lost after the last one read.
    void rx_overrun()
    {
        rx_resync();
    }

    // STK_CNTL of the last byte received, also during our own TX. A single aligned 32-bit load, so
    // main code can read it without masking IRQs.
    inline __attribute__((always_inline)) uint32_t last_rx_tick() const
    {
        return rx_last_tick;
    }

    // TC ISR: the last byte of our reply has left the shifter and DE is released. now: STK_CNTL.
    // RX bytes were not parsed during the TX; reset again so nothing from before it meets the
    // bytes after it.
    inline __attribute__((always_inline)) void tx_done(uint32_t now)
    {
        tx_end_tick = now;
        rx_resync();
        idle = true;
    }

    // STK_CNTL at the end of our last TX. Tracked apart from RX because our own bytes are not
    // necessarily echoed to RX.
    inline __attribute__((always_inline)) uint32_t last_tx_end_tick() const
    {
        return tx_end_tick;
    }

    // No reply in flight and none built but not yet started.
    inline __attribute__((always_inline)) bool tx_idle() const
    {
        return idle && send_data_len == 0;
    }

    // No received frame waiting for the main loop, and the parser is not inside a frame that is
    // still arriving. A frame cut short leaves _index/drop_bytes set until the next byte resyncs
    // the parser; once the line has been quiet past the resync gap that frame is dead, so it
    // counts as idle. _index and drop_bytes are ISR-owned, hence the volatile reads.
    inline __attribute__((always_inline)) bool rx_idle(uint32_t now) const
    {
        if (recv_data_len != 0) return false;
        if (*(const volatile int *)&_index == 0 && *(const volatile int *)&drop_bytes == 0)
            return true;
        // A byte the ISR stamped just after `now` was read is arriving right now, not 2^32 old.
        const uint32_t last = rx_last_tick;
        if ((uint32_t)(last - now) <= BUS_RX_RESYNC_GAP_TICKS) return false;
        return (uint32_t)(now - last) > BUS_RX_RESYNC_GAP_TICKS;
    }

    void irq(uint8_t data)
    {
        if (drop_bytes > 0)
        {
            if (--drop_bytes == 0)
                bambubus_heartbeat_seen_fast();
            return;
        }

        const int BUF_SZ = (int)sizeof(recv_data_buf[0]);
        int idx = _index;

        if (idx == 0)
        {
            if (data == 0x3D || data == 0x33)
            {
                bus_irq_data_ptr[0] = data;
                data_length_index = 4;
                length = data_CRC8_index = 6;
                _index = 1;
                irq_package_type = (_bus_data_type)data;
            }
            return;
        }

        if (idx < 0 || idx >= BUF_SZ)
        {
            _index = 0;
            return;
        }

        uint8_t *buf = bus_irq_data_ptr;
        buf[idx] = data;

        if (idx == 1)
        {
            if (data & 0x80)
            {
                data_length_index = 2;
                data_CRC8_index = 3;
            }
            else
            {
                data_CRC8_index = 6;
                data_length_index = (irq_package_type == _bus_data_type::bambubus) ? 5 : 4;
            }
        }

        if (idx == data_length_index)
        {
            if (irq_package_type == _bus_data_type::bambubus)
            {
                if (data_length_index == 2)
                    length = data;
                else
                    length = (int)buf[4] | ((int)data << 8);
            }
            else if (irq_package_type == _bus_data_type::ahub_bus)
            {
                length = (((int)data) << 2) + 12;
            }

            if (length <= (int)data_CRC8_index || length > BUF_SZ)
            {
                _index = 0;
                return;
            }
        }

        if (idx == data_CRC8_index)
        {
            if (data != bus_crc8(buf, (uint32_t)data_CRC8_index))
            {
                _index = 0;
                return;
            }
        }

        if (irq_package_type == _bus_data_type::bambubus &&
            idx == 4 &&
            data_length_index == 2 &&
            length >= 6 &&
            buf[1] == 0xC5 &&
            data == 0x20)
        {
            const int remain = length - 5;
            _index = 0;

            if (remain > 0)
            {
                drop_bytes = remain;
            }
            else
            {
                bambubus_heartbeat_seen_fast();
            }
            return;
        }

        ++idx;

        if (idx >= length)
        {
            _index = 0;

            if (recv_data_len == 0)
            {
                uint8_t *tmp = bus_recv_data_ptr;
                bus_recv_data_ptr = bus_irq_data_ptr;
                bus_irq_data_ptr = tmp;
                bus_package_type = irq_package_type;
                recv_data_len = length;
            }
            return;
        }

        _index = idx;
    }

    void send_package()
    {
        const int len = send_data_len;
        if (len > 0 && len <= 1280)
        {
            if (!idle) return;

            uint8_t *tx = tx_build_buf();
            tx_build_sel ^= 1;

            tx_start(tx, (uint16_t)len);
            send_data_len = 0;
        }
    }

    void send_package(uint8_t *data, uint16_t len)
    {
        if (len > 0 && len <= 1280)
        {
            if (!idle) return;
            tx_start(data, len);
        }
    }
} __attribute__((aligned(4)));

extern _bus_port_deal bus_port_to_host;
extern void bus_init();

#define host_device_type_none 0x0000
#define host_device_type_ahub 0x0001
#define host_device_type_ams 0x0700
extern uint16_t bus_host_device_type;