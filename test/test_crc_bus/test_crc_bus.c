// Host tests for src/crc_bus.c: the lookup tables must match a bitwise reference of the BambuBus
// CRCs, and the CRCs must match a reply frame that printers accept.

#include <stdint.h>
#include <unity.h>

#include "crc_bus.h"

// BambuBus header CRC: CRC-8, poly 0x39, init 0x66, MSB first, no reflection, no final xor.
static uint8_t ref_crc8(const uint8_t *data, uint32_t len)
{
    uint8_t r = 0x66u;
    for (uint32_t i = 0; i < len; i++)
    {
        r ^= data[i];
        for (int b = 0; b < 8; b++)
        {
            const unsigned msb = r & 0x80u;
            r = (uint8_t)(r << 1);
            if (msb) r ^= 0x39u;
        }
    }
    return r;
}

// BambuBus frame CRC: CRC-16/CCITT poly 0x1021, init 0x913D, MSB first, stored little-endian.
static uint16_t ref_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t r = 0x913Du;
    for (uint32_t i = 0; i < len; i++)
    {
        r ^= (uint16_t)(data[i] << 8);
        for (int b = 0; b < 8; b++)
        {
            const unsigned msb = r & 0x8000u;
            r = (uint16_t)(r << 1);
            if (msb) r ^= 0x1021u;
        }
    }
    return r;
}

// Short-header ACK the BMCU sends for set_filament_info (src/bambu_bus_ams.cpp, set_filament_res).
static const uint8_t k_set_filament_res[] = {0x3D, 0xC0, 0x08, 0xB2, 0x08, 0x60, 0xB4, 0x04};

void setUp(void) {}
void tearDown(void) {}

static void test_crc8_table_matches_bitwise_reference_for_every_byte(void)
{
    for (unsigned v = 0; v < 256; v++)
    {
        const uint8_t b = (uint8_t)v;
        TEST_ASSERT_EQUAL_HEX8(ref_crc8(&b, 1), bus_crc8(&b, 1));
    }
}

static void test_crc16_table_matches_bitwise_reference_for_every_byte(void)
{
    for (unsigned v = 0; v < 256; v++)
    {
        const uint8_t b = (uint8_t)v;
        TEST_ASSERT_EQUAL_HEX16(ref_crc16(&b, 1), bus_crc16(&b, 1));
    }
}

static void test_crcs_match_reference_on_a_long_buffer(void)
{
    uint8_t buf[300];
    for (unsigned i = 0; i < sizeof(buf); i++)
        buf[i] = (uint8_t)(i * 37u + 11u);
    TEST_ASSERT_EQUAL_HEX8(ref_crc8(buf, sizeof(buf)), bus_crc8(buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_HEX16(ref_crc16(buf, sizeof(buf)), bus_crc16(buf, sizeof(buf)));
}

static void test_set_filament_ack_header_crc8(void)
{
    TEST_ASSERT_EQUAL_HEX8(k_set_filament_res[3], bus_crc8(k_set_filament_res, 3));
}

static void test_set_filament_ack_frame_crc16_little_endian(void)
{
    const uint16_t crc = bus_crc16(k_set_filament_res, 6);
    TEST_ASSERT_EQUAL_HEX8(k_set_filament_res[6], (uint8_t)(crc & 0xFFu));
    TEST_ASSERT_EQUAL_HEX8(k_set_filament_res[7], (uint8_t)(crc >> 8));
}

static void test_empty_input_returns_init_values(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x66, bus_crc8(k_set_filament_res, 0));
    TEST_ASSERT_EQUAL_HEX16(0x913D, bus_crc16(k_set_filament_res, 0));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_crc8_table_matches_bitwise_reference_for_every_byte);
    RUN_TEST(test_crc16_table_matches_bitwise_reference_for_every_byte);
    RUN_TEST(test_crcs_match_reference_on_a_long_buffer);
    RUN_TEST(test_set_filament_ack_header_crc8);
    RUN_TEST(test_set_filament_ack_frame_crc16_little_endian);
    RUN_TEST(test_empty_input_returns_init_values);
    return UNITY_END();
}
