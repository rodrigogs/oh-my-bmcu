#pragma once
#include "crc_bus.h"
#include "host_link.h"
#include <string.h>

enum class _bus_data_type : uint8_t
{
    bambubus = 0x3D,
    ahub_bus = 0x33,
    none = 0x00
};

void bambubus_heartbeat_seen_fast(void);

// RX resync gap in SysTick ticks (HCLK/8 = 18 MHz; time_ticks32() keeps that rate while
// time_hw_flash_clock() slows the core). One 9E1 byte at 1.25 Mbaud is 11 bits = 8.8 us = 158.4 ticks.
// bus_rx_poll() drops a frame (or heartbeat skip) still in progress once it has seen no new bytes for at
// least this long, measured between its own polls (the main loop's, plus those bus_background_ready()
// runs): the stamp is the poll that first saw the last byte, not the byte's arrival. In practice that is
// 200 us plus up to about two main-loop passes after the last byte, so a frame that lost bytes or began on
// a false header is dropped before it swallows the printer's next frame only if that frame starts later
// than this; one that starts sooner can still be lost (one request, which the printer retries). The fork's
// RX ISR timed each byte and reset at exactly 200 us. The printer's own inter-byte timing has not been
// measured; 200 us (about 23 byte times) is the value the fork ran on the A1 (hardware test Runs 1 and 2)
// and is still far below the printer's reply timeout.
#define BUS_RX_RESYNC_GAP_TICKS 3600u

class _bus_port_deal // RX frame parser: bus_rx_poll() (main loop) feeds it byte by byte from the RX DMA ring
{
public:
    uint8_t send_data_buf[1280] __attribute__((aligned(4)));

private:
    // The only instance (bus_port_to_host) is a global, so the buffers start zeroed.
    // cppcheck-suppress uninitMemberVarPrivate
    uint8_t tx_dma_buf[1280] __attribute__((aligned(4)));
    // cppcheck-suppress uninitMemberVarPrivate
    uint8_t recv_data_buf[3][1280] __attribute__((aligned(4)));
    uint8_t tx_build_sel = 0;
    uint8_t rx_read = 0, rx_write = 0, rx_count = 0;
    uint16_t rx_length[3] = {};
    _bus_data_type rx_type[3] = {};
    int _index = 0;
    int length = 999;
    uint8_t data_length_index = 0;
    uint8_t data_CRC8_index = 0;
    _bus_data_type irq_package_type = _bus_data_type::none;
    uint8_t *bus_irq_data_ptr = recv_data_buf[0];
    int heartbeat_drop_bytes = 0;
    // RX ring position (bus_rx_poll's absolute byte count) at the start of our last TX; see rx_mark_tx().
    uint32_t rx_tx_mark = 0;
    bool rx_tx_mark_pending = false;
    // cppcheck-suppress uninitMemberVarPrivate ; set by init() before any send_package()
    bool (*port_send_datas)(uint8_t *data, uint16_t len);

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

    void init(bool (*_port_send_datas)(uint8_t *data, uint16_t len))
    {
        _index = 0;
        length = 999;
        data_length_index = 0;
        data_CRC8_index = 0;
        irq_package_type = _bus_data_type::none;
        bus_irq_data_ptr = recv_data_buf[0];
        heartbeat_drop_bytes = 0;
        bus_recv_data_ptr = recv_data_buf[0];
        rx_read = rx_write = rx_count = 0;
        rx_tx_mark_pending = false;
        idle = true;
        send_data_len = 0;
        recv_data_len = 0;
        tx_build_sel  = 0;
        port_send_datas = _port_send_datas;
    }

    uint32_t rx_dropped = 0;
    volatile uint32_t tx_errors = 0;

    void reset_rx()
    {
        // a reset parser holds nothing our TX could have cut (also clears the mark when rx_fault restarts the ring)
        rx_tx_mark_pending = false;
        _index = 0;
        heartbeat_drop_bytes = 0;
        irq_package_type = _bus_data_type::none;
    }

    bool receiving() const { return _index != 0 || heartbeat_drop_bytes != 0; }

    void release_packet()
    {
        if (rx_count) {
            rx_read = (uint8_t)((rx_read + 1u) % 3u);
            --rx_count;
        }
        bus_recv_data_ptr = recv_data_buf[rx_read];
        bus_package_type = rx_count ? rx_type[rx_read] : _bus_data_type::none;
        recv_data_len = rx_count ? rx_length[rx_read] : 0;
    }

    // Our TX starts (bus_uart1_dma_send(); RX stays off until it ends). ring_pos: the RX ring's byte
    // count at that moment. A frame the parser is still inside once it has consumed those bytes was cut
    // by our TX (we answered while the printer was already sending), so its head must not combine with
    // the bytes received after the TX (audit #24, tx-no-collision-guard). Main loop only.
    void rx_mark_tx(uint32_t ring_pos)
    {
        rx_tx_mark = ring_pos;
        rx_tx_mark_pending = true;
    }

    // bus_rx_poll(): the parser has consumed the RX ring up to ring_pos.
    void rx_reached(uint32_t ring_pos)
    {
        if (!rx_tx_mark_pending || (int32_t)(ring_pos - rx_tx_mark) < 0) return;
        rx_tx_mark_pending = false;
        if (receiving())
        {
            reset_rx();
            ++rx_dropped;
        }
    }

    // bus_rx_poll()'s heartbeat fast path: true when a pending TX mark falls strictly inside the n bytes
    // starting at ring position `from`. Skipping them in one step would carry a heartbeat our TX cut
    // across the mark into the printer's next frame, so the bytes go to irq() and rx_reached() drops the
    // cut heartbeat at the mark. A mark at `from` was already handled by rx_reached(); one at from + n
    // means the heartbeat ended before our TX.
    bool rx_mark_inside(uint32_t from, uint32_t n) const
    {
        const uint32_t d = rx_tx_mark - from;
        return rx_tx_mark_pending && d != 0u && d < n;
    }

    void irq(uint8_t data)
    {
        // Heartbeat
        if (__builtin_expect(heartbeat_drop_bytes > 0, 0))
        {
            if (--heartbeat_drop_bytes == 0)
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

            const int minimum = irq_package_type == _bus_data_type::ahub_bus ? 12 :
                (data_length_index == 2 ? 7 : 15);
            if (length < minimum || length > BUF_SZ)
            {
                _index = 0;
                return;
            }
        }

        if (irq_package_type == _bus_data_type::bambubus &&
            idx == 4 && data_length_index == 2 && length >= 6 &&
            buf[1] == 0xC5u && data == 0x20u)
        {
            const int remain = length - 5;
            _index = 0;
            irq_package_type = _bus_data_type::none;
            if (remain > 0)
                heartbeat_drop_bytes = remain;
            else
                bambubus_heartbeat_seen_fast();
            return;
        }

        if (idx == data_CRC8_index)
        {
            if (data != bus_crc8(buf, (uint32_t)data_CRC8_index))
            {
                _index = 0;
                return;
            }
        }

        ++idx;

        if (idx >= length)
        {
            _index = 0;

            if (rx_count < 2u)
            {
                rx_length[rx_write] = (uint16_t)length;
                rx_type[rx_write] = irq_package_type;
                if (rx_count == 0u) {
                    bus_recv_data_ptr = bus_irq_data_ptr;
                    bus_package_type = irq_package_type;
                    recv_data_len = length;
                }
                ++rx_count;
                rx_write = (uint8_t)((rx_write + 1u) % 3u);
                bus_irq_data_ptr = recv_data_buf[rx_write];
            }
            else ++rx_dropped;
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
            if (port_send_datas(tx, (uint16_t)len)) {
                tx_build_sel ^= 1;
                send_data_len = 0;
            }
        }
    }

    void send_package(uint8_t *data, uint16_t len)
    {
        if (len > 0 && len <= 1280)
        {
            if (!idle) return;
            port_send_datas(data, len);
        }
    }
} __attribute__((aligned(4)));

extern _bus_port_deal bus_port_to_host;
extern void bus_init();
void bus_rx_poll();
bool bus_background_ready();
void bus_shutdown();

extern uint16_t bus_host_device_type; // host_device_type_* values: host_link.h