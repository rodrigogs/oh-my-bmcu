// Golden-frame tests for src/bambu_bus_ams.cpp, the BambuBus replies the printer reads. The real
// ams.cpp and bambu_bus_ams.cpp are compiled into this file, with the real RX parser
// (_bus_hardware.h) and CRCs (crc_bus.c): each test feeds printer frames byte by byte into
// bus_port_to_host, runs bambubus_run() as the main loop does, and compares the bytes handed to the
// UART with a frame recorded from this code at 4933421 (image 1c42d010..., the one tested on the
// A1). A change to any reply byte fails here, so one that is meant has to update the frame on purpose.
//
// Host stand-ins: ch32v20x.h (next to this file) for ws2812.h; the IRQ mask, the SysTick counter and
// the chip UID (src/hal/irq_wch.h, src/hal/time_hw.h and bambu_bus_ams.cpp, #if !defined(__riscv));
// the loaded-channel hooks of app_api.h (main.cpp), delay_us and the bus_port_to_host instance
// (_bus_hardware.cpp).

#include <stdint.h>
#include <string.h>
#include <unity.h>

#define BAMBU_BUS_AMS_NUM 0

#include "ams.cpp"
#include "bambu_bus_ams.cpp"

// ---- Host stand-ins ----
volatile uint32_t time_hw_host_stk_cntl = 0u;
uint32_t time_hw_tpus = 18u;     // SysTick ticks per us (144 MHz / 8)
uint32_t time_hw_tpms = 18000u;  // per ms
void delay_us(uint32_t us) { time_hw_host_stk_cntl += us * time_hw_tpus; }

// Any 12 bytes: the serial number and the online-detect ID are hashed from them.
const uint8_t bambubus_host_uid[12] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x0F, 0x1E, 0x2D, 0x3C};

_bus_port_deal bus_port_to_host;

// ---- adapted from main.cpp: the loaded-channel state, without the NVM and boot-restore parts ----
static uint8_t g_loaded_ch = 0xFF;

void ams_datas_set_need_to_save_filament(uint8_t) {}

void ams_state_set_loaded(uint8_t filament_ch)
{
    if (filament_ch >= 4u) return;
    if (g_loaded_ch != 0xFFu) return;
    g_loaded_ch = filament_ch;
}

void ams_state_set_unloaded(uint8_t filament_ch)
{
    if (g_loaded_ch == 0xFFu) return;
    if (filament_ch < 4u && g_loaded_ch != filament_ch) return;
    g_loaded_ch = 0xFFu;
}

uint8_t ams_state_get_loaded(void) { return g_loaded_ch; }
bool ams_state_boot_restore_deferred(void) { return false; }
void ams_state_printer_command(uint8_t, uint8_t, uint8_t) {}

// ---- Frames quoted in the source ----
// ---- bambu_bus_ams.cpp at this commit: a printer 0x03 request, verbatim ----
// 3D C5 0C C8 03 00 07 00 7F 02 36 54
// ---- end of the bambu_bus_ams.cpp copy ----
// on_use, channel 0.
static const uint8_t kPrinterOnUse[] = {0x3D, 0xC5, 0x0C, 0xC8, 0x03, 0x00, 0x07, 0x00, 0x7F, 0x02, 0x36, 0x54};

// ---- bambu_bus_ams.cpp at this commit: two AMS 0x03 replies, verbatim ----
// 3D F0 2C C1 03 00 00 00 FF 00 00 00 00 6F F0 FB FF 36 00 00 00 F8 FF F7 FF 00 00 27 00 55 F8 EE F9 F0 B7 BA B9 B2 00 00 00 00 88 E6
// 3D D0 2C D1 03 03 00 02 00 00 00 80 3F FF FF FF FF 36 00 00 00 00 00 00 00 00 00 27 00 55 FF FF FF FF 01 01 01 01 00 00 00 00 15 95
// ---- end of the bambu_bus_ams.cpp copy ----
static const uint8_t kAmsIdle[44] = {
    0x3D, 0xF0, 0x2C, 0xC1, 0x03, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x6F, 0xF0, 0xFB,
    0xFF, 0x36, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xF7, 0xFF, 0x00, 0x00, 0x27, 0x00, 0x55, 0xF8, 0xEE,
    0xF9, 0xF0, 0xB7, 0xBA, 0xB9, 0xB2, 0x00, 0x00, 0x00, 0x00, 0x88, 0xE6};
static const uint8_t kAmsSendOut[44] = {
    0x3D, 0xD0, 0x2C, 0xD1, 0x03, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x80, 0x3F, 0xFF, 0xFF, 0xFF,
    0xFF, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x00, 0x55, 0xFF, 0xFF,
    0xFF, 0xFF, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x15, 0x95};

// ---- bambu_bus_ams.cpp at this commit: a printer 0x04 request, verbatim ----
// 3D C5 0D F1 04 00 01 00 03 FF 00 B2 C4
// ---- end of the bambu_bus_ams.cpp copy ----
// The printer's poll (0xFF, 0x01): it changes nothing, the reply reports the current state.
static const uint8_t kPrinterPoll[] = {0x3D, 0xC5, 0x0D, 0xF1, 0x04, 0x00, 0x01, 0x00, 0x03, 0xFF, 0x00, 0xB2, 0xC4};

// ---- Golden frames: what this code sends at 4933421 ----
// 0x03 replies (44 bytes): 3D, C0 | package number << 3, 2C, CRC8, 03, unit, 00, use flag, channel,
// meters (float), pressure, unknow2, unknow3[12], state flags (bit 2ch present, 2ch+1 moving), last1,
// last2, channel again, 00, 00 00, CRC16. The package number counts 0..7 over the 0x03 and 0x04
// replies together.

// Nothing in use: channel FF, pressure 0xFF74, unknow2 0x0008.
static const uint8_t kReplyIdle[] = {
    0x3D, 0xC0, 0x2C, 0xD9, 0x03, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x74, 0xFF, 0x08,
    0x00, 0x36, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xF7, 0xFF, 0x00, 0x00, 0x27, 0x00, 0x55, 0xFB, 0xF4,
    0xF2, 0xF6, 0xB5, 0xB7, 0xB4, 0xB1, 0xFF, 0x00, 0x00, 0x00, 0x6E, 0xF1};
// Loading channel 0: send_out (use 0x02, pressure 0x4700), before_on_use 0xA5 after it (use 0x04,
// still 0x4700), on_use (0x2B00); channel 0 present and moving (state 0x57), 1.0 m.
static const uint8_t kReplySendOut[] = {
    0x3D, 0xC0, 0x2C, 0xD9, 0x03, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x47, 0xFF,
    0xFF, 0x36, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xF7, 0xFF, 0x00, 0x00, 0x27, 0x00, 0x57, 0xFB, 0xF4,
    0xF2, 0xF6, 0xB5, 0xB7, 0xB4, 0xB1, 0x00, 0x00, 0x00, 0x00, 0x25, 0x58};
static const uint8_t kReplyBeforeOnUse[] = {
    0x3D, 0xC8, 0x2C, 0xDD, 0x03, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x47, 0xFF,
    0xFF, 0x36, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xF7, 0xFF, 0x00, 0x00, 0x27, 0x00, 0x57, 0xFB, 0xF4,
    0xF2, 0xF6, 0xB5, 0xB7, 0xB4, 0xB1, 0x00, 0x00, 0x00, 0x00, 0xB7, 0x17};
static const uint8_t kReplyOnUse[] = {
    0x3D, 0xD0, 0x2C, 0xD1, 0x03, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x2B, 0xFF,
    0xFF, 0x36, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xF7, 0xFF, 0x00, 0x00, 0x27, 0x00, 0x57, 0xFB, 0xF4,
    0xF2, 0xF6, 0xB5, 0xB7, 0xB4, 0xB1, 0x00, 0x00, 0x00, 0x00, 0x5C, 0xC0};
// The 0x04 reply (60 bytes) to the printer's poll with channel 0 on_use: temperature 22.0 C (DC 00),
// 20% humidity, channels 0-3 present (0F 0F 0F), the channel of the request (FF), then the 0x03
// fields from the unit on, with their own unknow3 and last3/last4 00000000/FFFFFFFF.
static const uint8_t kReplyPollOnUse[] = {
    0x3D, 0xD8, 0x3C, 0x0E, 0x04, 0x00, 0xDC, 0x00, 0x14, 0x0F, 0x0F, 0x0F, 0xFF, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x2B, 0xFF, 0xFF, 0x36, 0x00, 0x00,
    0x00, 0xF9, 0xFF, 0xF8, 0xFF, 0x00, 0x00, 0x27, 0x00, 0x57, 0xFB, 0xF4, 0xF2, 0xF6, 0xB5, 0xB7,
    0xB4, 0xB1, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xE6, 0x6E};
// A jam latched on channel 0 (Motion_control sets pressure 0xF06F): the on_use reply and the poll
// report 0xF06F with unknow2 0x1CE7, on which the printer pauses.
static const uint8_t kReplyOnUseJam[] = {
    0x3D, 0xD8, 0x2C, 0xD5, 0x03, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x6F, 0xF0, 0xE7,
    0x1C, 0x36, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xF7, 0xFF, 0x00, 0x00, 0x27, 0x00, 0x57, 0xFB, 0xF4,
    0xF2, 0xF6, 0xB5, 0xB7, 0xB4, 0xB1, 0x00, 0x00, 0x00, 0x00, 0xCE, 0x55};
static const uint8_t kReplyPollJam[] = {
    0x3D, 0xE0, 0x3C, 0x12, 0x04, 0x00, 0xDC, 0x00, 0x14, 0x0F, 0x0F, 0x0F, 0xFF, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x6F, 0xF0, 0xE7, 0x1C, 0x36, 0x00, 0x00,
    0x00, 0xF9, 0xFF, 0xF8, 0xFF, 0x00, 0x00, 0x27, 0x00, 0x57, 0xFB, 0xF4, 0xF2, 0xF6, 0xB5, 0xB7,
    0xB4, 0xB1, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xDA, 0xA9};
// 0xF06F set while channel 0 is in send_out: set_motion's send_out sets 0x4700 before the reply is
// built, so the jam is not reported there (the same bytes as kReplySendOut, one package
// number later).
static const uint8_t kReplySendOutJam[] = {
    0x3D, 0xC8, 0x2C, 0xDD, 0x03, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x47, 0xFF,
    0xFF, 0x36, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xF7, 0xFF, 0x00, 0x00, 0x27, 0x00, 0x57, 0xFB, 0xF4,
    0xF2, 0xF6, 0xB5, 0xB7, 0xB4, 0xB1, 0x00, 0x00, 0x00, 0x00, 0x28, 0x55};
// before_on_use 0x7F: the first reply replays row 0 of before_on_use_sniff_7f_rows (frame 55848),
// from pressure to last2; the rows start at the state flags byte, so it reads 0x57 (channel 0 present
// and moving) whatever the channel, and the last byte of last2 keeps its default.
static const uint8_t kReplyReplayCh0[] = {
    0x3D, 0xC0, 0x2C, 0xD9, 0x03, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x3D, 0x40, 0xF8,
    0xFF, 0x36, 0x00, 0x00, 0x00, 0xF7, 0xFF, 0xF6, 0xFF, 0x00, 0x00, 0xD9, 0xFF, 0x57, 0xFA, 0xF4,
    0xF3, 0xF7, 0xF0, 0xB5, 0xB4, 0xB1, 0x00, 0x00, 0x00, 0x00, 0x56, 0xCA};
static const uint8_t kReplyReplayCh1[] = {
    0x3D, 0xC0, 0x2C, 0xD9, 0x03, 0x00, 0x00, 0x04, 0x01, 0x00, 0x00, 0x80, 0x3F, 0x3D, 0x40, 0xF8,
    0xFF, 0x36, 0x00, 0x00, 0x00, 0xF7, 0xFF, 0xF6, 0xFF, 0x00, 0x00, 0xD9, 0xFF, 0x57, 0xFA, 0xF4,
    0xF3, 0xF7, 0xF0, 0xB5, 0xB4, 0xB1, 0x01, 0x00, 0x00, 0x00, 0xC7, 0x20};

// Online detect (29 bytes): 3D C0 1D B4 05, subtype, unit, prefix, the 16 ID bytes
// (long_packge_version_serial_number from 33 on: 0E, A0 + unit, 10 bytes of the UID hash, FF FF FF FF),
// 00 00 00, CRC16. The first probe after boot or a link loss gets prefix 0x0C, the next ones 0x0A.
static const uint8_t kOnlineDetectFirst[] = {
    0x3D, 0xC0, 0x1D, 0xB4, 0x05, 0x00, 0x00, 0x0C, 0x0E, 0xA0, 0x99, 0xC1, 0x40, 0x76, 0x56, 0x45,
    0x56, 0x45, 0xAC, 0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x6C, 0xF3};
static const uint8_t kOnlineDetectNext[] = {
    0x3D, 0xC0, 0x1D, 0xB4, 0x05, 0x00, 0x00, 0x0A, 0x0E, 0xA0, 0x99, 0xC1, 0x40, 0x76, 0x56, 0x45,
    0x56, 0x45, 0xAC, 0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xC6, 0x4E};
// The reply to the printer's confirm of that ID: subtype 01, prefix 0x0A.
static const uint8_t kOnlineDetectConfirm[] = {
    0x3D, 0xC0, 0x1D, 0xB4, 0x05, 0x01, 0x00, 0x0A, 0x0E, 0xA0, 0x99, 0xC1, 0x40, 0x76, 0x56, 0x45,
    0x56, 0x45, 0xAC, 0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x1E, 0x23};

// Long replies: 3D 00, the request's package number, total length, CRC8, the request's source as
// target, 0x0700 as source, the request's type, data, CRC16.
// 0x103: version 00 00 32 0A (10.50.00.00) and "AMS08", the unit in the last byte.
static const uint8_t kReplyVersion[] = {
    0x3D, 0x00, 0x2A, 0x00, 0x24, 0x00, 0xE6, 0x00, 0x06, 0x00, 0x07, 0x03, 0x01, 0x00, 0x00, 0x32,
    0x0A, 0x41, 0x4D, 0x53, 0x30, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x86, 0xFC};
// 0x21A: the unit, then five zeros.
static const uint8_t kReplyMcOnline[] = {
    0x3D, 0x00, 0x2A, 0x00, 0x15, 0x00, 0x62, 0x00, 0x06, 0x00, 0x07, 0x1A, 0x02, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x93, 0x4C};
// 0x402: 15, "0EA0" with the unit in the last digit, 11 hex digits of the UID hash, zeros, the 16 ID
// bytes of the online detect, 9 more FF, 00, FF 00 FF 00 FF 00, and the unit.
static const uint8_t kReplySerial[] = {
    0x3D, 0x00, 0x2A, 0x00, 0x51, 0x00, 0x40, 0x00, 0x06, 0x00, 0x07, 0x02, 0x04, 0x0F, 0x30, 0x45,
    0x41, 0x30, 0x39, 0x39, 0x43, 0x31, 0x34, 0x30, 0x37, 0x36, 0x35, 0x36, 0x34, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0xA0,
    0x99, 0xC1, 0x40, 0x76, 0x56, 0x45, 0x56, 0x45, 0xAC, 0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x7E,
    0x8B};
// 0x211 for channel 1 after boot (ams_init defaults): unit, channel, "GFG00", "PETG", colour FFFFFFFF,
// 240 and 220 C.
static const uint8_t kReplyFilament[] = {
    0x3D, 0x00, 0x2A, 0x00, 0x92, 0x00, 0xBD, 0x00, 0x06, 0x00, 0x07, 0x11, 0x02, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x47, 0x46, 0x47, 0x30, 0x30, 0x00, 0x00, 0x00, 0x50, 0x45, 0x54, 0x47, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x00, 0xDC, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x5E, 0x29};

// ---- Test harness ----
// One 9E1 byte at 1.25 Mbaud in 18 MHz SysTick ticks (158.4).
static const uint32_t BYTE_TICKS = 158u;
static const uint32_t FRAME_GAP_TICKS = 300u * 18u; // 300 us of silence between frames

static uint8_t g_tx[1280];
static int g_tx_len;

// Stand-in for bus_uart1_dma_send: keeps the bytes that would go on the wire.
static void fake_send(uint8_t *data, uint16_t len)
{
    memcpy(g_tx, data, len);
    g_tx_len = len;
}

void setUp(void)
{
    // The BMCU at boot: zeroed statics, ams_init(), Motion_control_init() sets the unit online, then
    // bus_init() and bambubus_init().
    package_num = 0u;
    last_before_on_use_motion_flag = 0x00u;
    count_on_use = 0u;
    memset(time_sendout_onuse_ticks, 0, sizeof(time_sendout_onuse_ticks));
    before_on_use_sniff_7f_active = 0u;
    before_on_use_sniff_7f_index = 0u;
    before_on_use_sniff_7f_channel = 0xFFu;
    g_loaded_ch = 0xFFu;

    ams_init();
    ams[BAMBU_BUS_AMS_NUM].online = true;
    time_hw_host_stk_cntl = 0x10000000u;
    bus_port_to_host.init(fake_send);
    bambubus_init();
}

void tearDown(void) {}

// Short-header frame from the printer: 3D C5, total length, CRC8, payload, CRC16.
static int short_frame(uint8_t *out, const uint8_t *payload, int n)
{
    const int len = 4 + n + 2;
    out[0] = 0x3D;
    out[1] = 0xC5;
    out[2] = (uint8_t)len;
    memcpy(out + 4, payload, (size_t)n);
    package_add_crc(out, len);
    return len;
}

// Long-header frame from the printer to the AMS (0x0700): 3D 05, package number, total length,
// CRC8, target, source, type (all little endian), data, CRC16.
static const uint16_t PRINTER_ADDRESS = 0x0600u; // any source: the reply only sends it back
static int long_frame(uint8_t *out, uint16_t type, const uint8_t *data, int n)
{
    const int len = 13 + n + 2;
    const uint8_t head[13] = {0x3D, 0x05, 0x2A, 0x00, (uint8_t)len, (uint8_t)(len >> 8), 0x00,
                              0x00, 0x07, (uint8_t)PRINTER_ADDRESS, (uint8_t)(PRINTER_ADDRESS >> 8),
                              (uint8_t)type, (uint8_t)(type >> 8)};
    memcpy(out, head, sizeof(head));
    memcpy(out + 13, data, (size_t)n);
    package_add_crc(out, len);
    return len;
}

static void rx(const uint8_t *frame, int n)
{
    time_hw_host_stk_cntl += FRAME_GAP_TICKS;
    for (int i = 0; i < n; i++)
    {
        bus_port_to_host.rx_byte(frame[i], time_hw_host_stk_cntl, false);
        time_hw_host_stk_cntl += BYTE_TICKS;
    }
}

// One main-loop pass: bambubus_run(), then the TX of whatever it built. Returns the reply length,
// 0 without a reply.
static int loop_pass(void)
{
    g_tx_len = 0;
    bambubus_run();
    bus_port_to_host.send_package();
    if (g_tx_len != 0)
        bus_port_to_host.tx_done(time_hw_host_stk_cntl);
    return g_tx_len;
}

// The frame arrives on the RX line and the main loop answers it.
static int exchange(const uint8_t *frame, int n)
{
    rx(frame, n);
    TEST_ASSERT_EQUAL_INT_MESSAGE(n, bus_port_to_host.recv_data_len, "the RX parser did not deliver the frame");
    return loop_pass();
}

static int motion(uint8_t ch, uint8_t statu, uint8_t flag)
{
    const uint8_t p[] = {0x03, BAMBU_BUS_AMS_NUM, statu, ch, flag, 0x02};
    uint8_t f[16];
    return exchange(f, short_frame(f, p, (int)sizeof(p)));
}

static int on_use(void) { return exchange(kPrinterOnUse, (int)sizeof(kPrinterOnUse)); }
static int printer_poll(void) { return exchange(kPrinterPoll, (int)sizeof(kPrinterPoll)); }

static int probe(void)
{
    const uint8_t p[] = {0x05, 0x00};
    uint8_t f[16];
    return exchange(f, short_frame(f, p, (int)sizeof(p)));
}

// The printer's confirm: subtype 0x01, our unit, then the prefix and the 16 ID bytes it read in
// our last reply.
static int confirm(const uint8_t *prefix_and_id)
{
    uint8_t p[23] = {0x05, 0x01, BAMBU_BUS_AMS_NUM};
    memcpy(p + 3, prefix_and_id, 17);
    uint8_t f[32];
    return exchange(f, short_frame(f, p, (int)sizeof(p)));
}

static void heartbeat(void)
{
    const uint8_t p[] = {0x20, 0x00, 0x00};
    uint8_t f[16];
    rx(f, short_frame(f, p, (int)sizeof(p)));
    TEST_ASSERT_EQUAL_INT(0, loop_pass());
}

static int request(uint16_t type, const uint8_t *data, int n)
{
    uint8_t f[64];
    return exchange(f, long_frame(f, type, data, n));
}

static void assert_reply(const uint8_t *golden, int len)
{
    TEST_ASSERT_EQUAL_INT(len, g_tx_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(golden, g_tx, len);
}
#define ASSERT_REPLY(golden) assert_reply(golden, (int)sizeof(golden))

// The first two probes and the confirm, from boot.
static void register_unit(void)
{
    TEST_ASSERT_EQUAL_INT(29, probe());
    TEST_ASSERT_EQUAL_INT(29, probe());
    uint8_t id[17];
    memcpy(id, g_tx + 7, sizeof(id));
    TEST_ASSERT_EQUAL_INT(29, confirm(id));
}

// ---- The CRCs against the captured frames ----

// package_add_crc with the real crc_bus.c gives back the CRC8 and CRC16 of the four frames the
// source quotes (two printer requests, two AMS replies).
static void test_package_add_crc_reproduces_the_captured_frames(void)
{
    const uint8_t *frames[] = {kPrinterOnUse, kPrinterPoll, kAmsIdle, kAmsSendOut};
    const int lens[] = {(int)sizeof(kPrinterOnUse), (int)sizeof(kPrinterPoll), (int)sizeof(kAmsIdle),
                        (int)sizeof(kAmsSendOut)};
    for (int i = 0; i < 4; i++)
    {
        uint8_t f[64];
        memcpy(f, frames[i], (size_t)lens[i]);
        f[3] = 0x00;
        f[lens[i] - 2] = 0x00;
        f[lens[i] - 1] = 0x00;
        package_add_crc(f, lens[i]);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(frames[i], f, lens[i]);
    }
}

// ---- Motion replies (0x03 and 0x04) ----

static void test_idle_reply(void)
{
    TEST_ASSERT_EQUAL_INT(44, motion(0xFF, 0x00, 0x00));
    ASSERT_REPLY(kReplyIdle);
}

// The idle reply against the captured AMS one it was modelled on: the same header, command, unit,
// use flag, channel FF, meters, unknow3 and state flags (all four present, none moving). It differs
// in the package number, the pressure and unknow2 (0xFF74 / 0x0008, the capture 0xF06F / 0xFFFB),
// last1/last2 and the second channel byte (FF, the capture 00).
static void test_idle_reply_keeps_the_captured_idle_layout(void)
{
    motion(0xFF, 0x00, 0x00);
    TEST_ASSERT_EQUAL_HEX8(kAmsIdle[0], g_tx[0]);
    TEST_ASSERT_EQUAL_HEX8(kAmsIdle[2], g_tx[2]);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(kAmsIdle + 4, g_tx + 4, 9);    // command .. meters
    TEST_ASSERT_EQUAL_HEX8_ARRAY(kAmsIdle + 17, g_tx + 17, 13); // unknow3, state flags
    TEST_ASSERT_EQUAL_HEX8_ARRAY(kAmsIdle + 39, g_tx + 39, 3);  // 00, 00 00
}

static void test_replies_through_a_load(void)
{
    TEST_ASSERT_EQUAL_INT(44, motion(0x00, 0x03, 0x00));
    ASSERT_REPLY(kReplySendOut);
    TEST_ASSERT_EQUAL_INT(44, motion(0x00, 0x09, 0xA5));
    ASSERT_REPLY(kReplyBeforeOnUse);
    TEST_ASSERT_EQUAL_INT(44, on_use());
    ASSERT_REPLY(kReplyOnUse);
    TEST_ASSERT_EQUAL_INT(60, printer_poll());
    ASSERT_REPLY(kReplyPollOnUse);
}

static void test_a_jam_on_use_reports_f06f(void)
{
    motion(0x00, 0x03, 0x00);
    motion(0x00, 0x09, 0xA5);
    on_use();
    ams[BAMBU_BUS_AMS_NUM].pressure = 0xF06Fu;
    TEST_ASSERT_EQUAL_INT(44, on_use());
    ASSERT_REPLY(kReplyOnUseJam);
    TEST_ASSERT_EQUAL_INT(60, printer_poll());
    ASSERT_REPLY(kReplyPollJam);
}

static void test_a_jam_in_send_out_is_not_reported(void)
{
    motion(0x00, 0x03, 0x00);
    ams[BAMBU_BUS_AMS_NUM].pressure = 0xF06Fu;
    TEST_ASSERT_EQUAL_INT(44, motion(0x00, 0x03, 0x00));
    ASSERT_REPLY(kReplySendOutJam);
}

static void test_before_on_use_7f_replays_the_capture(void)
{
    TEST_ASSERT_EQUAL_INT(44, motion(0x00, 0x09, 0x7F));
    ASSERT_REPLY(kReplyReplayCh0);

    setUp();
    TEST_ASSERT_EQUAL_INT(44, motion(0x01, 0x09, 0x7F));
    ASSERT_REPLY(kReplyReplayCh1);
}

// ---- Online detect (0x05) ----

static void test_online_detect_prefix_phases(void)
{
    TEST_ASSERT_EQUAL_INT(29, probe());
    ASSERT_REPLY(kOnlineDetectFirst);
    TEST_ASSERT_EQUAL_INT(29, probe());
    ASSERT_REPLY(kOnlineDetectNext);
    TEST_ASSERT_EQUAL_INT(29, probe());
    ASSERT_REPLY(kOnlineDetectNext);

    uint8_t id[17];
    memcpy(id, g_tx + 7, sizeof(id));
    TEST_ASSERT_EQUAL_INT(29, confirm(id));
    ASSERT_REPLY(kOnlineDetectConfirm);

    // Registered: probes get no reply while heartbeats flow (none have yet).
    TEST_ASSERT_EQUAL_INT(0, probe());
}

static void test_online_detect_confirm_of_another_id_gets_no_reply(void)
{
    TEST_ASSERT_EQUAL_INT(29, probe());
    uint8_t id[17];
    memcpy(id, g_tx + 7, sizeof(id));
    id[16] ^= 0x01u;
    TEST_ASSERT_EQUAL_INT(0, confirm(id));

    // Not registered: the next probe is answered, with the later prefix.
    TEST_ASSERT_EQUAL_INT(29, probe());
    ASSERT_REPLY(kOnlineDetectNext);
}

static void test_online_detect_is_answered_again_after_a_link_loss(void)
{
    heartbeat();
    register_unit();
    heartbeat();
    TEST_ASSERT_EQUAL_INT(0, probe());

    // 1.1 s without a heartbeat: the pass that sees the link lost re-arms the handshake.
    time_hw_host_stk_cntl += 1100u * time_hw_tpms;
    TEST_ASSERT_EQUAL_INT(0, loop_pass());
    TEST_ASSERT_EQUAL_INT(29, probe());
    ASSERT_REPLY(kOnlineDetectFirst);
}

// ---- Long replies ----

static void test_version_reply(void)
{
    const uint8_t unit[] = {BAMBU_BUS_AMS_NUM};
    TEST_ASSERT_EQUAL_INT(36, request(0x103, unit, 1));
    ASSERT_REPLY(kReplyVersion);

    // What the A1 reported for this image over its LAN MQTT on 2026-09-25 (hardware-test-log.md,
    // run 1, section 1): module ams/0, version 10.50.00.00, AMS08. The version is the first data word, read
    // high byte first.
    TEST_ASSERT_EQUAL_UINT8(10, g_tx[13 + 3]);
    TEST_ASSERT_EQUAL_UINT8(50, g_tx[13 + 2]);
    TEST_ASSERT_EQUAL_UINT8(0, g_tx[13 + 1]);
    TEST_ASSERT_EQUAL_UINT8(0, g_tx[13 + 0]);
    TEST_ASSERT_EQUAL_MEMORY("AMS08", g_tx + 13 + 4, 6);
}

static void test_mc_online_reply(void)
{
    const uint8_t unit[] = {BAMBU_BUS_AMS_NUM};
    TEST_ASSERT_EQUAL_INT(21, request(0x21A, unit, 1));
    ASSERT_REPLY(kReplyMcOnline);
}

static void test_serial_number_reply(void)
{
    uint8_t data[34] = {0};
    data[33] = BAMBU_BUS_AMS_NUM;
    TEST_ASSERT_EQUAL_INT(81, request(0x402, data, (int)sizeof(data)));
    ASSERT_REPLY(kReplySerial);
}

static void test_filament_info_reply(void)
{
    const uint8_t unit_and_channel[] = {BAMBU_BUS_AMS_NUM, 0x01};
    TEST_ASSERT_EQUAL_INT(146, request(0x211, unit_and_channel, 2));
    ASSERT_REPLY(kReplyFilament);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_package_add_crc_reproduces_the_captured_frames);
    RUN_TEST(test_idle_reply);
    RUN_TEST(test_idle_reply_keeps_the_captured_idle_layout);
    RUN_TEST(test_replies_through_a_load);
    RUN_TEST(test_a_jam_on_use_reports_f06f);
    RUN_TEST(test_a_jam_in_send_out_is_not_reported);
    RUN_TEST(test_before_on_use_7f_replays_the_capture);
    RUN_TEST(test_online_detect_prefix_phases);
    RUN_TEST(test_online_detect_confirm_of_another_id_gets_no_reply);
    RUN_TEST(test_online_detect_is_answered_again_after_a_link_loss);
    RUN_TEST(test_version_reply);
    RUN_TEST(test_mc_online_reply);
    RUN_TEST(test_serial_number_reply);
    RUN_TEST(test_filament_info_reply);
    return UNITY_END();
}
