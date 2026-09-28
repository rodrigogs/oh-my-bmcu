// Host tests for src/nvm_records.h: the filament-info and loaded-channel journal records of
// Flash_saves.cpp, and what their scans at boot give back (the filament info and loaded channel the
// printer reads from the BMCU after a reset, and the slot the next record goes into), with valid,
// torn, blank and older-layout slots.

#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "Flash_saves.h"
#include "nvm_journal.h"
#include "nvm_records.h"

static const uint32_t PAGE_WORDS = FLASH_NVM256_PAGE_SIZE / 4u;
static const uint32_t FIL_WORDS = NVM_FIL_SLOT_WORDS;
static const uint32_t FIL_SLOTS = NVM_FIL_SLOTS_PER_PAGE;
static const uint32_t STA_WORDS = NVM_STA_SLOT_WORDS;
static const uint32_t STA_SLOTS = NVM_STA_SLOTS_PER_PAGE;
static const uint32_t STA_LOG_PAGES = NVM_STA_LOG_PAGES;
static const uint32_t STA_LOG_SLOTS = STA_LOG_PAGES * STA_SLOTS;

// ---- adapted from Flash_saves.cpp: crc32_hw_words, the CH32 CRC unit in software ----
// ---- anchor: crc32_hw_words ----
// CRC-32 polynomial 0x04C11DB7, register reset to 0xFFFFFFFF, one 32-bit word at a time MSB first,
// no reflection or final XOR (STM32-compatible unit). test_crc32_model_matches_the_wch_example pins
// it to WCH's documented result; the tests below only need pack and validate to use the same CRC.
static uint32_t crc32_words(const void *data, uint32_t bytes)
{
    const uint32_t *p = (const uint32_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0u; i < (bytes >> 2); i++)
    {
        crc ^= p[i];
        for (int b = 0; b < 32; b++) crc = (crc & 0x80000000u) ? (crc << 1) ^ 0x04C11DB7u : (crc << 1);
    }
    return crc;
}

static uint32_t crc_other(const void *data, uint32_t bytes)
{
    return crc32_words(data, bytes) ^ 1u;
}

// One filament page followed by an erased guard page (a scan that ran past its 6 slots would find
// the guard's words), and the 10-page loaded-channel log followed by a guard page.
static uint32_t fil_mem[2u * PAGE_WORDS];
static uint32_t *const fil_page = fil_mem;
static uint32_t sta_mem[(STA_LOG_PAGES + 1u) * PAGE_WORDS];
static uint32_t *const sta_log = sta_mem;

void setUp(void)
{
    for (uint32_t i = 0u; i < 2u * PAGE_WORDS; i++) fil_mem[i] = NVM_ERASED_WORD;
    for (uint32_t i = 0u; i < (STA_LOG_PAGES + 1u) * PAGE_WORDS; i++) sta_mem[i] = NVM_ERASED_WORD;
}

void tearDown(void) {}

static Flash_FilamentInfo info_for(uint8_t tag)
{
    Flash_FilamentInfo f;
    memset(&f, 0, sizeof(f));
    for (uint32_t i = 0u; i < sizeof(f.bambubus_filament_id); i++)
        f.bambubus_filament_id[i] = (uint8_t)('A' + tag + i);
    f.color_R = tag;
    f.color_G = (uint8_t)(0xFFu - tag);
    f.color_B = 0x80u;
    f.color_A = 0xFFu;
    f.temperature_min = (uint16_t)(190u + tag);
    f.temperature_max = (uint16_t)(230u + tag);
    for (uint32_t i = 0u; i < sizeof(f.name) - 1u; i++) f.name[i] = (char)('a' + (tag + i) % 26u);
    return f;
}

static void fil_record(uint32_t slot, uint8_t tag)
{
    const Flash_FilamentInfo f = info_for(tag);
    nvm_fil_pack(fil_page + slot * FIL_WORDS, &f, crc32_words);
}

static void sta_record(uint32_t slot, uint16_t seq, uint8_t ch)
{
    nvm_sta_pack(sta_log + slot * STA_WORDS, seq, ch);
}

static bool fil_load(Flash_FilamentInfo *last, uint32_t *next)
{
    memset(last, 0xA5, sizeof(*last));  // must be overwritten either way
    *next = 0xDEADu;
    return nvm_fil_load(fil_page, last, next, crc32_words);
}

static bool sta_scan(uint8_t *ch, uint16_t *next_seq, uint32_t *next_slot)
{
    *ch = 0xEEu;
    *next_seq = 0xDEADu;
    *next_slot = 0xDEADu;
    return nvm_sta_scan(sta_log, STA_LOG_SLOTS, ch, next_seq, next_slot);
}

#define TEST_ASSERT_INFO(expected, actual) TEST_ASSERT_EQUAL_MEMORY(&(expected), &(actual), sizeof(expected))

// WCH's CRC example for the CH32V20x (openwch/ch32v20x, EVT/EXAM/CRC/CRC_Calculation/User/main.c at
// d161f84): CRC_CalcBlockCRC over these 32 words "should be 0x199AC3CA". CRC_CalcBlockCRC is
// crc32_hw_words without the reset.
static void test_crc32_model_matches_the_wch_example(void)
{
    uint32_t buf[32];
    for (uint32_t i = 0u; i < 32u; i++)
        buf[i] = ((4u * i + 1u) << 24) | ((4u * i + 2u) << 16) | ((4u * i + 3u) << 8) | (4u * i + 4u);
    TEST_ASSERT_EQUAL_HEX32(0x01020304u, buf[0]);
    TEST_ASSERT_EQUAL_HEX32(0x7D7E7F80u, buf[31]);
    TEST_ASSERT_EQUAL_HEX32(0x199AC3CAu, crc32_words(buf, sizeof(buf)));
}

// A packed record: MAGIC_FIL, the info as stored in RAM, the CRC of those 9 words. It reads back
// as the same info, and any single flipped bit, or another CRC, makes it invalid.
static void test_fil_record_round_trip(void)
{
    const Flash_FilamentInfo in = info_for(3u);
    uint32_t w[FIL_WORDS];
    nvm_fil_pack(w, &in, crc32_words);

    TEST_ASSERT_EQUAL_HEX32(MAGIC_FIL, w[0]);
    TEST_ASSERT_EQUAL_MEMORY(&in, &w[1], sizeof(in));
    TEST_ASSERT_EQUAL_HEX32(crc32_words(w, 36u), w[9]);

    Flash_FilamentInfo out;
    memset(&out, 0, sizeof(out));
    TEST_ASSERT_TRUE(nvm_fil_valid(w, &out, crc32_words));
    TEST_ASSERT_INFO(in, out);
    TEST_ASSERT_TRUE(nvm_fil_valid(w, nullptr, crc32_words));
    TEST_ASSERT_FALSE(nvm_fil_valid(w, &out, crc_other));

    for (uint32_t i = 0u; i < FIL_WORDS; i++)
    {
        for (uint32_t b = 0u; b < 32u; b++)
        {
            w[i] ^= 1u << b;
            TEST_ASSERT_FALSE(nvm_fil_valid(w, &out, crc32_words));
            w[i] ^= 1u << b;
        }
    }
    TEST_ASSERT_INFO(in, out);  // nothing copied out of an invalid record
}

// Records in slots 0..2: the newest (highest slot) is loaded and the next goes into slot 3. A full
// page gives slot 6, which makes the next write erase the page.
static void test_fil_scan_gives_the_last_record_and_the_next_slot(void)
{
    Flash_FilamentInfo last;
    uint32_t next;

    for (uint8_t s = 0u; s < 3u; s++) fil_record(s, (uint8_t)(10u + s));
    TEST_ASSERT_TRUE(fil_load(&last, &next));
    Flash_FilamentInfo want = info_for(12u);
    TEST_ASSERT_INFO(want, last);
    TEST_ASSERT_EQUAL_UINT32(3u, next);
    TEST_ASSERT_FALSE(nvm_fil_needs_erase(fil_page, next));

    for (uint8_t s = 3u; s < FIL_SLOTS; s++) fil_record(s, (uint8_t)(10u + s));
    TEST_ASSERT_TRUE(fil_load(&last, &next));
    want = info_for(15u);
    TEST_ASSERT_INFO(want, last);
    TEST_ASSERT_EQUAL_UINT32(FIL_SLOTS, next);
    TEST_ASSERT_TRUE(nvm_fil_needs_erase(fil_page, next));
}

// Power lost while slot 2 was being programmed (words 0..4 written, the rest erased): the info of
// slot 1 is loaded, and the next record goes into slot 3 without an erase (the torn slot is not
// blank, so it is skipped and kept). A torn slot between valid ones is skipped the same way.
static void test_fil_torn_record_mid_page(void)
{
    Flash_FilamentInfo last;
    uint32_t next;

    fil_record(0u, 1u);
    fil_record(1u, 2u);
    fil_record(2u, 3u);
    for (uint32_t w = 2u * FIL_WORDS + 5u; w < 3u * FIL_WORDS; w++) fil_page[w] = NVM_ERASED_WORD;

    TEST_ASSERT_TRUE(fil_load(&last, &next));
    Flash_FilamentInfo want = info_for(2u);
    TEST_ASSERT_INFO(want, last);
    TEST_ASSERT_EQUAL_UINT32(3u, next);
    TEST_ASSERT_FALSE(nvm_fil_needs_erase(fil_page, next));

    // Torn with only its CRC word missing, then a valid record after it.
    fil_record(2u, 3u);
    fil_page[3u * FIL_WORDS - 1u] = NVM_ERASED_WORD;
    fil_record(3u, 4u);
    TEST_ASSERT_TRUE(fil_load(&last, &next));
    want = info_for(4u);
    TEST_ASSERT_INFO(want, last);
    TEST_ASSERT_EQUAL_UINT32(4u, next);
}

// The only record of the page is torn: nothing saved (all-zero info), and the next record goes into
// slot 0, which the write then erases first.
static void test_fil_torn_first_record_gives_slot_0(void)
{
    Flash_FilamentInfo last;
    uint32_t next;
    const Flash_FilamentInfo zero = {};

    fil_record(0u, 7u);
    for (uint32_t w = 4u; w < FIL_WORDS; w++) fil_page[w] = NVM_ERASED_WORD;

    TEST_ASSERT_FALSE(fil_load(&last, &next));
    TEST_ASSERT_INFO(zero, last);
    TEST_ASSERT_EQUAL_UINT32(0u, next);
    TEST_ASSERT_TRUE(nvm_fil_needs_erase(fil_page, next));
}

// Older layouts pad with 0xFF, which the scan takes for blank slots. The V7..V10.1 whole-page
// 'FIL1' record (header, info, 0xFF padding, CRC in the last word; see test_nvm_journal) has the
// FIL1 magic in slot 0 but no valid record: nothing loaded, slot 0, erased on the first write.
// Valid records followed by 0xFF-padded slots: the last one is loaded, the next goes into the
// first padded slot, and that write erases the page.
static void test_fil_ff_padded_slots_from_an_older_layout(void)
{
    Flash_FilamentInfo last;
    uint32_t next;
    uint32_t first_empty = 0xDEADu;
    const Flash_FilamentInfo zero = {};

    fil_page[0] = MAGIC_FIL;
    fil_page[1] = 0x00200001u;
    fil_page[2] = 0x00FF0000u;
    for (uint32_t w = 3u; w < 11u; w++) fil_page[w] = 0x41474600u + w;
    for (uint32_t w = 11u; w < PAGE_WORDS - 1u; w++) fil_page[w] = 0xFFFFFFFFu;
    fil_page[PAGE_WORDS - 1u] = 0x1234ABCDu;

    TEST_ASSERT_FALSE(nvm_fil_scan_page(fil_page, &last, &first_empty, crc32_words));
    TEST_ASSERT_EQUAL_UINT32(2u, first_empty);  // slot 2 starts in the padding
    TEST_ASSERT_FALSE(fil_load(&last, &next));
    TEST_ASSERT_INFO(zero, last);
    TEST_ASSERT_EQUAL_UINT32(0u, next);
    TEST_ASSERT_TRUE(nvm_fil_needs_erase(fil_page, next));

    setUp();
    fil_record(0u, 5u);
    fil_record(1u, 6u);
    for (uint32_t w = 2u * FIL_WORDS; w < PAGE_WORDS; w++) fil_page[w] = 0xFFFFFFFFu;
    TEST_ASSERT_TRUE(fil_load(&last, &next));
    Flash_FilamentInfo want = info_for(6u);
    TEST_ASSERT_INFO(want, last);
    TEST_ASSERT_EQUAL_UINT32(2u, next);
    TEST_ASSERT_TRUE(nvm_fil_needs_erase(fil_page, next));
}

// An erased page (after a full NVM clear), and a page programmed with 0xFF: nothing saved, slot 0.
static void test_fil_blank_page(void)
{
    Flash_FilamentInfo last;
    uint32_t next;
    uint32_t first_empty = 0xDEADu;
    const Flash_FilamentInfo zero = {};

    TEST_ASSERT_FALSE(nvm_fil_scan_page(fil_page, &last, &first_empty, crc32_words));
    TEST_ASSERT_EQUAL_UINT32(0u, first_empty);
    TEST_ASSERT_FALSE(fil_load(&last, &next));
    TEST_ASSERT_INFO(zero, last);
    TEST_ASSERT_EQUAL_UINT32(0u, next);
    TEST_ASSERT_FALSE(nvm_fil_needs_erase(fil_page, next));

    for (uint32_t i = 0u; i < PAGE_WORDS; i++) fil_page[i] = 0xFFFFFFFFu;
    TEST_ASSERT_FALSE(fil_load(&last, &next));
    TEST_ASSERT_INFO(zero, last);
    TEST_ASSERT_EQUAL_UINT32(0u, next);
    TEST_ASSERT_TRUE(nvm_fil_needs_erase(fil_page, next));
}

// ---- adapted from Flash_saves.cpp: Flash_AMS_filament_write and Flash_saves_init, on RAM ----
// ---- anchor: Flash_AMS_filament_write ----
// ---- anchor: Flash_saves_init ----
// 40 writes of changing info into one page (erased when nvm_fil_needs_erase says so, next slot as
// the write sets it), each followed by a reset: the scan always gives back the info just written
// and the slot the firmware had cached.
static void test_fil_writes_then_reset_read_back(void)
{
    uint32_t cached_next = 0u;

    for (uint8_t i = 0u; i < 40u; i++)
    {
        uint32_t slot = cached_next;
        if (nvm_fil_needs_erase(fil_page, slot))
        {
            for (uint32_t w = 0u; w < PAGE_WORDS; w++) fil_page[w] = NVM_ERASED_WORD;
            slot = 0u;
        }
        fil_record(slot, i);
        cached_next = (slot + 1u < FIL_SLOTS) ? slot + 1u : FIL_SLOTS;

        Flash_FilamentInfo last;
        uint32_t next;
        TEST_ASSERT_TRUE(fil_load(&last, &next));
        const Flash_FilamentInfo want = info_for(i);
        TEST_ASSERT_INFO(want, last);
        TEST_ASSERT_EQUAL_UINT32(cached_next, next);
    }
}

// A loaded-channel record: 0xA5 tag, sequence, channel; w1 = w0 ^ MAGIC_STA. It reads back as the
// same sequence and channel; a torn (w1 erased), blank, retagged or altered record does not.
static void test_sta_record_round_trip(void)
{
    const uint16_t seqs[] = {0u, 1u, 0x1234u, 0x7FFFu, 0x8000u, 0xFFFFu};
    const uint8_t chs[] = {0u, 1u, 3u, 0xFFu};
    uint32_t w[STA_WORDS];
    uint16_t seq;
    uint8_t ch;

    for (uint32_t i = 0u; i < sizeof(seqs) / sizeof(seqs[0]); i++)
    {
        for (uint32_t j = 0u; j < sizeof(chs); j++)
        {
            nvm_sta_pack(w, seqs[i], chs[j]);
            TEST_ASSERT_EQUAL_HEX32((0xA5u << 24) | ((uint32_t)seqs[i] << 8) | chs[j], w[0]);
            TEST_ASSERT_EQUAL_HEX32(w[0] ^ MAGIC_STA, w[1]);
            seq = 0u;
            ch = 0u;
            TEST_ASSERT_TRUE(nvm_sta_valid(w, &seq, &ch));
            TEST_ASSERT_EQUAL_UINT16(seqs[i], seq);
            TEST_ASSERT_EQUAL_UINT8(chs[j], ch);
        }
    }

    nvm_sta_pack(w, 42u, 2u);
    const uint32_t w0 = w[0];
    w[1] = NVM_ERASED_WORD;
    TEST_ASSERT_FALSE(nvm_sta_valid(w, &seq, &ch));
    w[1] = 0xFFFFFFFFu;
    TEST_ASSERT_FALSE(nvm_sta_valid(w, &seq, &ch));
    w[0] = NVM_ERASED_WORD;
    w[1] = NVM_ERASED_WORD;
    TEST_ASSERT_FALSE(nvm_sta_valid(w, &seq, &ch));
    w[0] = 0xFFFFFFFFu;
    w[1] = 0xFFFFFFFFu;
    TEST_ASSERT_FALSE(nvm_sta_valid(w, &seq, &ch));
    for (uint32_t b = 0u; b < 32u; b++)
    {
        w[0] = w0 ^ (1u << b);
        w[1] = w0 ^ MAGIC_STA;
        TEST_ASSERT_FALSE(nvm_sta_valid(w, &seq, &ch));
        w[0] = w0;
        w[1] = (w0 ^ MAGIC_STA) ^ (1u << b);
        TEST_ASSERT_FALSE(nvm_sta_valid(w, &seq, &ch));
    }
    // Retagged but consistent: w1 still matches w0, the tag does not.
    w[0] = (w0 & 0x00FFFFFFu) | 0xA4000000u;
    w[1] = w[0] ^ MAGIC_STA;
    TEST_ASSERT_FALSE(nvm_sta_valid(w, &seq, &ch));
}

// Newer is up to 2^15 - 1 ahead, modulo 2^16: 0 follows 65535. Equal or 2^15 apart: neither.
static void test_sta_seq_compare_across_the_wrap(void)
{
    TEST_ASSERT_TRUE(nvm_sta_seq_newer(1u, 0u));
    TEST_ASSERT_FALSE(nvm_sta_seq_newer(0u, 1u));
    TEST_ASSERT_TRUE(nvm_sta_seq_newer(0u, 0xFFFFu));
    TEST_ASSERT_FALSE(nvm_sta_seq_newer(0xFFFFu, 0u));
    TEST_ASSERT_TRUE(nvm_sta_seq_newer(5u, 0xFFFAu));
    TEST_ASSERT_FALSE(nvm_sta_seq_newer(0xFFFAu, 5u));
    TEST_ASSERT_TRUE(nvm_sta_seq_newer(0x7FFFu, 0u));
    TEST_ASSERT_TRUE(nvm_sta_seq_newer(0x7FFEu, 0xFFFFu));
    TEST_ASSERT_FALSE(nvm_sta_seq_newer(0x8000u, 0u));
    TEST_ASSERT_FALSE(nvm_sta_seq_newer(0u, 0x8000u));
    TEST_ASSERT_FALSE(nvm_sta_seq_newer(1234u, 1234u));
}

// Erased log, or one programmed with 0xFF: no saved channel (0xFF), the next record is sequence 0
// in slot 0.
static void test_sta_scan_blank_log(void)
{
    uint8_t ch;
    uint16_t next_seq;
    uint32_t next_slot;

    TEST_ASSERT_FALSE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_HEX8(0xFFu, ch);
    TEST_ASSERT_EQUAL_UINT16(0u, next_seq);
    TEST_ASSERT_EQUAL_UINT32(0u, next_slot);

    for (uint32_t i = 0u; i < STA_LOG_SLOTS * STA_WORDS; i++) sta_log[i] = 0xFFFFFFFFu;
    TEST_ASSERT_FALSE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_HEX8(0xFFu, ch);
    TEST_ASSERT_EQUAL_UINT16(0u, next_seq);
    TEST_ASSERT_EQUAL_UINT32(0u, next_slot);
}

// The newest record wins across the 2^16 sequence wrap and across the log's slot wrap: slots 0..9
// hold the 10 records written after the log wrapped (sequences 65530.. through 0 to 3), slots
// 10..319 the older ones. The next record goes into slot 10 with sequence 4. The newest in the last
// slot gives slot 0.
static void test_sta_scan_newest_across_both_wraps(void)
{
    uint8_t ch;
    uint16_t next_seq;
    uint32_t next_slot;

    // 330 records: record k went into slot k % 320 with sequence k - 326 (mod 2^16), so records
    // 320..329, in slots 0..9, run from 65530 through 0 to 3.
    for (uint32_t k = 0u; k < STA_LOG_SLOTS + 10u; k++)
        sta_record(k % STA_LOG_SLOTS, (uint16_t)(3u - 329u + k), (uint8_t)(k & 3u));

    TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_UINT8(329u & 3u, ch);
    TEST_ASSERT_EQUAL_UINT16(4u, next_seq);
    TEST_ASSERT_EQUAL_UINT32(10u, next_slot);

    // Sequences 65534, 65535, 0, 1 in slots 0..3: slot 3.
    setUp();
    sta_record(0u, 0xFFFEu, 0u);
    sta_record(1u, 0xFFFFu, 1u);
    sta_record(2u, 0u, 2u);
    sta_record(3u, 1u, 3u);
    TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_UINT8(3u, ch);
    TEST_ASSERT_EQUAL_UINT16(2u, next_seq);
    TEST_ASSERT_EQUAL_UINT32(4u, next_slot);

    setUp();
    sta_record(STA_LOG_SLOTS - 2u, 99u, 1u);
    sta_record(STA_LOG_SLOTS - 1u, 100u, 2u);
    TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_UINT8(2u, ch);
    TEST_ASSERT_EQUAL_UINT16(101u, next_seq);
    TEST_ASSERT_EQUAL_UINT32(0u, next_slot);

    // Not ordered (equal sequences): the lower slot wins.
    setUp();
    sta_record(40u, 7u, 1u);
    sta_record(41u, 7u, 2u);
    TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_UINT8(1u, ch);
    TEST_ASSERT_EQUAL_UINT32(41u, next_slot);
}

// Power lost while slot 37 (page 1, slot 5) was being programmed (w0 written, w1 erased): the
// channel of slot 36 is restored, and the next record is its sequence + 1 in slot 37. A torn
// record whose w0 carries a newer sequence is ignored too. The write skips the torn slot (it cannot
// be programmed over) and programs slot 38 without erasing page 1, which holds slot 36.
static void test_sta_scan_after_a_torn_slot(void)
{
    uint8_t ch;
    uint16_t next_seq;
    uint32_t next_slot;

    for (uint32_t s = 0u; s < 37u; s++) sta_record(s, (uint16_t)(100u + s), (uint8_t)(s % 3u));
    sta_record(37u, 137u, 3u);  // newer than slot 36
    sta_log[37u * STA_WORDS + 1u] = NVM_ERASED_WORD;

    TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_UINT8(36u % 3u, ch);
    TEST_ASSERT_EQUAL_UINT16(137u, next_seq);  // 138 if the torn one counted
    TEST_ASSERT_EQUAL_UINT32(37u, next_slot);
    TEST_ASSERT_FALSE(nvm_sta_next_slot(sta_log, STA_LOG_SLOTS, &next_slot));
    TEST_ASSERT_EQUAL_UINT32(38u, next_slot);

    // The only record is torn: none saved, and the write erases page 0 and starts at slot 0.
    setUp();
    sta_record(0u, 5u, 1u);
    sta_log[1] = NVM_ERASED_WORD;
    TEST_ASSERT_FALSE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_HEX8(0xFFu, ch);
    TEST_ASSERT_EQUAL_UINT32(0u, next_slot);
    TEST_ASSERT_TRUE(nvm_sta_next_slot(sta_log, STA_LOG_SLOTS, &next_slot));
    TEST_ASSERT_EQUAL_UINT32(0u, next_slot);
}

// ---- adapted from Flash_saves.cpp: Flash_AMS_state_write, on RAM ----
// ---- anchor: Flash_AMS_state_write ----
// One write from the slot and sequence the firmware has cached: the slot and erase nvm_sta_next_slot
// gives, then `words` words of the record (2: written; 1: power lost between the two word programs;
// 0: power lost right after the erase, or before the first program). Returns the slot used. The
// erase and retry after a failed program is not modelled: programs here do not fail.
static uint32_t sta_erases;
static uint32_t sta_erases_off_page_start;

static uint32_t sta_write(uint32_t slot, uint16_t seq, uint8_t ch, uint32_t words)
{
    if (nvm_sta_next_slot(sta_log, STA_LOG_SLOTS, &slot))
    {
        uint32_t *const page = sta_log + (slot / STA_SLOTS) * PAGE_WORDS;
        for (uint32_t w = 0u; w < PAGE_WORDS; w++) page[w] = NVM_ERASED_WORD;
        sta_erases++;
        if (slot % STA_SLOTS != 0u) sta_erases_off_page_start++;
    }
    uint32_t w[STA_WORDS];
    nvm_sta_pack(w, seq, ch);
    for (uint32_t i = 0u; i < words; i++) sta_log[slot * STA_WORDS + i] = w[i];
    return slot;
}

// Slot 36 holds the newest record and slot 37 a torn one (the reset in the test above). The next
// load skips slot 37 and page 1 is not erased: with power lost right before its first program, or
// between its two words (in slot 38), the reset still restores slot 36's channel, never the older
// one at the end of page 0 (slot 31, channel 2), and the write after that lands in slot 39.
static void test_sta_torn_next_slot_keeps_the_newest_record(void)
{
    uint8_t ch;
    uint16_t next_seq;
    uint32_t next_slot;

    for (uint32_t s = 0u; s < 37u; s++) sta_record(s, (uint16_t)(100u + s), (s == 36u) ? 1u : 2u);
    sta_record(37u, 137u, 3u);
    sta_log[37u * STA_WORDS + 1u] = NVM_ERASED_WORD;
    sta_erases = 0u;
    sta_erases_off_page_start = 0u;

    TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_UINT32(38u, sta_write(next_slot, next_seq, 3u, 0u));
    TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_UINT8(1u, ch);
    TEST_ASSERT_EQUAL_UINT32(37u, next_slot);

    TEST_ASSERT_EQUAL_UINT32(38u, sta_write(next_slot, next_seq, 3u, 1u));  // torn again
    TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_UINT8(1u, ch);
    TEST_ASSERT_EQUAL_UINT16(137u, next_seq);
    TEST_ASSERT_EQUAL_UINT32(37u, next_slot);

    TEST_ASSERT_EQUAL_UINT32(39u, sta_write(next_slot, next_seq, 3u, 2u));
    TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_UINT8(3u, ch);
    TEST_ASSERT_EQUAL_UINT16(138u, next_seq);
    TEST_ASSERT_EQUAL_UINT32(40u, next_slot);
    TEST_ASSERT_EQUAL_UINT32(0u, sta_erases);

    uint16_t seq;
    uint8_t c;
    for (uint32_t s = 0u; s < 37u; s++) TEST_ASSERT_TRUE(nvm_sta_valid(sta_log + s * STA_WORDS, &seq, &c));
}

// The log's wrap: the newest record in slot 318, a torn one in slot 319 (the last). The write goes
// to slot 0, erasing page 0 (older records of the last pass); power lost right after that erase
// still restores slot 318's channel from page 9, which is kept.
static void test_sta_torn_last_slot_wraps_to_slot_0(void)
{
    uint8_t ch;
    uint16_t next_seq;
    uint32_t next_slot;

    for (uint32_t s = 0u; s < STA_LOG_SLOTS - 1u; s++)
        sta_record(s, (uint16_t)(1000u + s), (s == STA_LOG_SLOTS - 2u) ? 1u : 2u);
    sta_record(STA_LOG_SLOTS - 1u, 1319u, 3u);
    sta_log[(STA_LOG_SLOTS - 1u) * STA_WORDS + 1u] = NVM_ERASED_WORD;
    sta_erases = 0u;
    sta_erases_off_page_start = 0u;

    TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_UINT32(STA_LOG_SLOTS - 1u, next_slot);
    TEST_ASSERT_EQUAL_UINT32(0u, sta_write(next_slot, next_seq, 3u, 0u));
    TEST_ASSERT_EQUAL_UINT32(1u, sta_erases);
    TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_UINT8(1u, ch);
    TEST_ASSERT_EQUAL_UINT16(1319u, next_seq);

    TEST_ASSERT_EQUAL_UINT32(0u, sta_write(next_slot, next_seq, 3u, 2u));
    TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_UINT8(3u, ch);
    TEST_ASSERT_EQUAL_UINT16(1320u, next_seq);
    TEST_ASSERT_EQUAL_UINT32(1u, next_slot);
    TEST_ASSERT_EQUAL_UINT32(1u, sta_erases);
    TEST_ASSERT_EQUAL_UINT32(0u, sta_erases_off_page_start);
}

// 70000 loads and unloads (the sequence wraps at 65536, the log every 320 records, a page is erased
// whenever nvm_sta_next_slot says so), each followed by a reset: the scan always restores the
// channel just written, with the sequence and slot the firmware had cached.
static void test_sta_writes_then_reset_read_back(void)
{
    uint16_t seq = 0u;
    uint32_t slot = 0u;
    sta_erases = 0u;
    sta_erases_off_page_start = 0u;

    for (uint32_t i = 0u; i < 70000u; i++)
    {
        const uint8_t loaded = (i & 1u) ? 0xFFu : (uint8_t)((i >> 1) & 3u);
        slot = sta_write(slot, seq, loaded, 2u);
        seq = (uint16_t)(seq + 1u);
        slot = (slot + 1u) % STA_LOG_SLOTS;

        uint8_t ch;
        uint16_t next_seq;
        uint32_t next_slot;
        TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
        TEST_ASSERT_EQUAL_HEX8(loaded, ch);
        TEST_ASSERT_EQUAL_UINT16(seq, next_seq);
        TEST_ASSERT_EQUAL_UINT32(slot, next_slot);
    }
    // One erase per page each time the log comes round to it again, none on the first pass.
    TEST_ASSERT_EQUAL_UINT32((70000u - 1u) / STA_SLOTS - (STA_LOG_PAGES - 1u), sta_erases);
    TEST_ASSERT_EQUAL_UINT32(0u, sta_erases_off_page_start);
}

// 20000 loads and unloads with power lost in a fixed pattern: every 5th write between its two
// words, then its retry right after its erase (or before its first program when it has no erase);
// and the write after it right after its erase too. After every reset the scan restores the last
// channel that was completely written, and the write is retried from what the scan gives, as after
// a real reset. Pages are only erased when the log moves on to them, so no reset loses it.
static void test_sta_power_loss_during_writes_keeps_the_last_record(void)
{
    uint8_t ch = 0xFFu;
    uint16_t seq = 0u;
    uint32_t slot = 0u;
    uint8_t saved = 0xFFu;
    uint32_t resets = 0u;
    sta_erases = 0u;
    sta_erases_off_page_start = 0u;

    for (uint32_t i = 0u; i < 20000u; i++)
    {
        const uint8_t loaded = (i & 1u) ? 0xFFu : (uint8_t)((i >> 1) & 3u);
        uint32_t cut = (i % 5u == 3u) ? 1u : (i % 5u == 4u) ? 0u : 2u;

        for (;;)
        {
            const uint32_t used = sta_write(slot, seq, loaded, cut);
            if (cut == 2u)
            {
                slot = (used + 1u) % STA_LOG_SLOTS;
                break;
            }
            resets++;
            TEST_ASSERT_TRUE(sta_scan(&ch, &seq, &slot));
            TEST_ASSERT_EQUAL_HEX8_MESSAGE(saved, ch, "a reset lost the newest record");
            cut = (cut == 1u) ? 0u : 2u;
        }
        saved = loaded;
        seq = (uint16_t)(seq + 1u);

        uint16_t next_seq;
        uint32_t next_slot;
        TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
        TEST_ASSERT_EQUAL_HEX8(saved, ch);
        TEST_ASSERT_EQUAL_UINT16(seq, next_seq);
        TEST_ASSERT_EQUAL_UINT32(slot, next_slot);
    }
    TEST_ASSERT_EQUAL_UINT32(12000u, resets);
    TEST_ASSERT_TRUE(sta_erases > 0u);
    TEST_ASSERT_EQUAL_UINT32(0u, sta_erases_off_page_start);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_crc32_model_matches_the_wch_example);
    RUN_TEST(test_fil_record_round_trip);
    RUN_TEST(test_fil_scan_gives_the_last_record_and_the_next_slot);
    RUN_TEST(test_fil_torn_record_mid_page);
    RUN_TEST(test_fil_torn_first_record_gives_slot_0);
    RUN_TEST(test_fil_ff_padded_slots_from_an_older_layout);
    RUN_TEST(test_fil_blank_page);
    RUN_TEST(test_fil_writes_then_reset_read_back);
    RUN_TEST(test_sta_record_round_trip);
    RUN_TEST(test_sta_seq_compare_across_the_wrap);
    RUN_TEST(test_sta_scan_blank_log);
    RUN_TEST(test_sta_scan_newest_across_both_wraps);
    RUN_TEST(test_sta_scan_after_a_torn_slot);
    RUN_TEST(test_sta_torn_next_slot_keeps_the_newest_record);
    RUN_TEST(test_sta_torn_last_slot_wraps_to_slot_0);
    RUN_TEST(test_sta_writes_then_reset_read_back);
    RUN_TEST(test_sta_power_loss_during_writes_keeps_the_last_record);
    return UNITY_END();
}
