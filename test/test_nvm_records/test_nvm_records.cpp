// Host tests for src/nvm_records.h: the filament-info and loaded-channel journal records of
// Flash_saves.cpp, and what their scans at boot give back (the filament info and loaded channel the
// printer reads from the BMCU after a reset, and the slot the next record goes into), with valid,
// torn, blank and older-layout slots. Then RAM models of V11's background jobs (one page erase or one
// halfword program per main-loop pass, the magic word last, the filament rollover through the
// scratch page and its recovery at boot), cut by a power loss or a failed read-back at every step.
// Last, the buffer calibration record's rsv word (polarity, valid mask, sequence) and its two-slot
// read and write choice.

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

// The NVM pages Flash_saves.cpp uses (the page map in Flash_saves.h): the filament journals of
// channels 0..3, the loaded-channel log and the filament rollover scratch page.
static const uint32_t FIL_PAGE0 = 2u;
static const uint32_t STA_PAGE0 = 6u;
static const uint32_t SCRATCH_PAGE = 14u;
static const uint32_t ERASED_HALF = NVM_ERASED_WORD & 0xFFFFu;
static_assert(STA_PAGE0 + STA_LOG_PAGES == SCRATCH_PAGE, "the loaded-channel log ends at the scratch page");

// ---- Flash_saves.cpp at this commit: the scratch record's magic (OR the channel), verbatim ----
static constexpr uint32_t FIL_SCRATCH_MAGIC = 0x32524340u;
// ---- end of the Flash_saves.cpp copy ----

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

// The 4 KB NVM sector, 16 pages. The scan tests use channel 0's filament page (the page after it is
// erased, so a scan that ran past its 6 slots would find erased words there) and the loaded-channel
// log (followed by the scratch page, erased in setUp, for the same reason).
static uint32_t flash[FLASH_NVM_PAGE_COUNT][FLASH_NVM256_PAGE_SIZE / 4u];
static uint32_t *const fil_page = flash[FIL_PAGE0];
static uint32_t *const sta_log = flash[STA_PAGE0];

static void model_reset_counters(void);

void setUp(void)
{
    for (uint32_t pg = 0u; pg < FLASH_NVM_PAGE_COUNT; pg++)
        for (uint32_t w = 0u; w < PAGE_WORDS; w++) flash[pg][w] = NVM_ERASED_WORD;
    model_reset_counters();
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

static void fil_record_in(uint32_t ch, uint32_t slot, uint8_t tag)
{
    const Flash_FilamentInfo f = info_for(tag);
    nvm_fil_pack(flash[FIL_PAGE0 + ch] + slot * FIL_WORDS, &f, crc32_words);
}

static void fil_record(uint32_t slot, uint8_t tag)
{
    fil_record_in(0u, slot, tag);
}

static void sta_record(uint32_t slot, uint16_t seq, uint8_t ch)
{
    nvm_sta_pack(sta_log + slot * STA_WORDS, seq, ch);
}

// A loaded-channel record cut by a power loss after `halfwords` of its 4 halfword programs, in the
// order Flash_background_run programs them: w1 low, w1 high, w0 low, w0 high (with the tag).
static void sta_torn(uint32_t slot, uint16_t seq, uint8_t ch, uint32_t halfwords)
{
    uint32_t w[STA_WORDS];
    nvm_sta_pack(w, seq, ch);
    uint32_t *const p = sta_log + slot * STA_WORDS;
    p[0] = NVM_ERASED_WORD;
    p[1] = NVM_ERASED_WORD;
    for (uint32_t h = 0u; h < halfwords; h++)
    {
        const uint32_t index = (h + 2u) % 4u;
        const uint32_t shift = (index & 1u) * 16u;
        p[index >> 1] = (p[index >> 1] & ~(0xFFFFu << shift)) | (((w[index >> 1] >> shift) & 0xFFFFu) << shift);
    }
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
// page gives slot 6, which makes the next write roll the page over through the scratch page.
static void test_fil_scan_gives_the_last_record_and_the_next_slot(void)
{
    Flash_FilamentInfo last;
    uint32_t next;

    for (uint8_t s = 0u; s < 3u; s++) fil_record(s, (uint8_t)(10u + s));
    TEST_ASSERT_TRUE(fil_load(&last, &next));
    Flash_FilamentInfo want = info_for(12u);
    TEST_ASSERT_INFO(want, last);
    TEST_ASSERT_EQUAL_UINT32(3u, next);

    for (uint8_t s = 3u; s < FIL_SLOTS; s++) fil_record(s, (uint8_t)(10u + s));
    TEST_ASSERT_TRUE(fil_load(&last, &next));
    want = info_for(15u);
    TEST_ASSERT_INFO(want, last);
    TEST_ASSERT_EQUAL_UINT32(FIL_SLOTS, next);
}

// Power lost while slot 2 was being programmed (words 0..4 written, the rest erased): the info of
// slot 1 is loaded, and the next record goes into slot 3 (the torn slot is not erased, so it is
// skipped and kept). A torn slot between valid ones is skipped the same way.
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

    // Torn with only its CRC word missing, then a valid record after it.
    fil_record(2u, 3u);
    fil_page[3u * FIL_WORDS - 1u] = NVM_ERASED_WORD;
    fil_record(3u, 4u);
    TEST_ASSERT_TRUE(fil_load(&last, &next));
    want = info_for(4u);
    TEST_ASSERT_INFO(want, last);
    TEST_ASSERT_EQUAL_UINT32(4u, next);
}

// The only record of the page is torn (V11 programs word 0, the magic, last: here words 1..3 are
// written): nothing saved (all-zero info), and the next record goes into slot 1. The torn slot is
// never programmed over (that would fail its read-back and set the sticky flash fault).
static void test_fil_torn_first_record_is_skipped(void)
{
    Flash_FilamentInfo last;
    uint32_t next;
    const Flash_FilamentInfo zero = {};

    fil_record(0u, 7u);
    fil_page[0] = NVM_ERASED_WORD;
    for (uint32_t w = 4u; w < FIL_WORDS; w++) fil_page[w] = NVM_ERASED_WORD;

    TEST_ASSERT_FALSE(fil_load(&last, &next));
    TEST_ASSERT_INFO(zero, last);
    TEST_ASSERT_EQUAL_UINT32(1u, next);
}

// Older layouts pad with 0xFF, which is programmed, not erased. The V7..V10.1 whole-page 'FIL1'
// record (header, info, 0xFF padding, CRC in the last word; see test_nvm_journal) has the FIL1 magic
// in slot 0 but no valid record and no erased slot: nothing loaded, slot 6, so the first write rolls
// the page over. Valid records followed by 0xFF-padded slots: the last one is loaded, and again
// slot 6 (V11's lenient test would have programmed over the padding).
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
    TEST_ASSERT_EQUAL_UINT32(FIL_SLOTS, first_empty);
    TEST_ASSERT_FALSE(fil_load(&last, &next));
    TEST_ASSERT_INFO(zero, last);
    TEST_ASSERT_EQUAL_UINT32(FIL_SLOTS, next);

    setUp();
    fil_record(0u, 5u);
    fil_record(1u, 6u);
    for (uint32_t w = 2u * FIL_WORDS; w < PAGE_WORDS; w++) fil_page[w] = 0xFFFFFFFFu;
    TEST_ASSERT_TRUE(fil_load(&last, &next));
    Flash_FilamentInfo want = info_for(6u);
    TEST_ASSERT_INFO(want, last);
    TEST_ASSERT_EQUAL_UINT32(FIL_SLOTS, next);
}

// An erased page (after a page erase or a full NVM clear): nothing saved, slot 0. A page programmed
// with 0xFF: nothing saved, slot 6 (roll over: erase before the first record).
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

    for (uint32_t i = 0u; i < PAGE_WORDS; i++) fil_page[i] = 0xFFFFFFFFu;
    TEST_ASSERT_FALSE(fil_load(&last, &next));
    TEST_ASSERT_INFO(zero, last);
    TEST_ASSERT_EQUAL_UINT32(FIL_SLOTS, next);
}

// ---- adapted from Flash_saves.cpp: the flash operations of Flash_background_run, on RAM ----
// ---- anchor: flash_job_halfword ----
// ---- anchor: Flash_background_run ----
// One page erase or one halfword program per call, as Flash_background_run starts them. A power loss
// after power_budget operations stops everything (nothing more is erased or programmed); a failed
// read-back after fail_budget operations sets the sticky fault (V11's g_flash_fault, cleared only by
// a reboot) and drops the job. Programming a halfword that is not erased is recorded: on the chip
// it fails its read-back, which is exactly what the strict free-slot tests must never allow.
static int32_t power_budget;
static int32_t fail_budget;
static bool power_lost;
static bool fw_fault;
static bool programmed_over_used;
static uint32_t flash_ops;
static uint32_t erases;
static uint32_t sta_erased_newest;  // erases of the page that holds the newest loaded-channel record

static void model_reset_counters(void)
{
    power_budget = -1;
    fail_budget = -1;
    power_lost = false;
    fw_fault = false;
    programmed_over_used = false;
    flash_ops = 0u;
    erases = 0u;
    sta_erased_newest = 0u;
}

static bool flash_op(void)
{
    if (power_lost) return false;
    if (power_budget == 0)
    {
        power_lost = true;
        return false;
    }
    if (power_budget > 0) power_budget--;
    flash_ops++;
    return true;
}

static bool page_erase(uint32_t pg)
{
    if (!flash_op()) return false;
    for (uint32_t w = 0u; w < PAGE_WORDS; w++) flash[pg][w] = NVM_ERASED_WORD;
    erases++;
    return true;
}

static bool half_program(uint32_t pg, uint32_t word, uint32_t half, uint32_t value)
{
    if (!flash_op()) return false;
    const uint32_t shift = half * 16u;
    uint32_t &w = flash[pg][word];
    if (((w >> shift) & 0xFFFFu) != ERASED_HALF) programmed_over_used = true;
    bool ok = true;
    if (fail_budget == 0)
    {
        value = ~value;
        ok = false;
        fw_fault = true;
    }
    if (fail_budget >= 0) fail_budget--;
    w = (w & ~(0xFFFFu << shift)) | ((value & 0xFFFFu) << shift);
    return ok;
}

// flash_job_halfword: the halfword programmed at step `done` of a job of `halfwords` halfwords,
// so word 0 (the magic, with the tag) goes last.
static uint32_t job_halfword(uint32_t done, uint32_t halfwords)
{
    const uint32_t index = done + 2u;
    return index < halfwords ? index : index - halfwords;
}

static bool job_program(uint32_t pg, uint32_t word0, const uint32_t *words, uint32_t halfwords)
{
    for (uint32_t done = 0u; done < halfwords; done++)
    {
        const uint32_t index = job_halfword(done, halfwords);
        if (!half_program(pg, word0 + (index >> 1), index & 1u, words[index >> 1] >> ((index & 1u) * 16u)))
            return false;
    }
    return true;
}

// flash_word_prog_std (fil_scratch_recover's synchronous programs): a word that already holds the
// data is left alone, else its low then its high halfword.
static bool word_program(uint32_t pg, uint32_t word, uint32_t data)
{
    if (flash[pg][word] == data) return true;
    return half_program(pg, word, 0u, data) && half_program(pg, word, 1u, data >> 16);
}

// ---- adapted from Flash_saves.cpp: the filament journal (cache, write job, rollover, recovery) ----
// ---- anchor: Flash_AMS_filament_write ----
// ---- anchor: fil_scratch_recover ----
// ---- anchor: Flash_saves_init ----
// The firmware's RAM cache of the four filament pages, as Flash_saves_init fills it.
static uint8_t fw_fil_have[4];
static uint8_t fw_fil_first_empty[4];
static Flash_FilamentInfo fw_fil_last[4];

static bool fw_scratch_recover(void)
{
    if (fw_fault) return false;
    const uint32_t *saved = flash[SCRATCH_PAGE];
    if ((saved[0] & ~3u) != FIL_SCRATCH_MAGIC || crc32_words(saved, (FIL_WORDS - 1u) * 4u) != saved[FIL_WORDS - 1u])
        return true;
    const uint8_t ch = (uint8_t)(saved[0] & 3u);
    Flash_FilamentInfo info;
    memcpy(&info, &saved[1], sizeof(info));
    const bool restored = fw_fil_have[ch] && memcmp(&fw_fil_last[ch], &info, sizeof(info)) == 0;
    fw_fil_last[ch] = info;
    fw_fil_have[ch] = 1u;
    if (!restored)
    {
        uint32_t words[FIL_WORDS];
        nvm_fil_pack(words, &info, crc32_words);
        const uint32_t pg = FIL_PAGE0 + ch;
        if (!page_erase(pg)) return false;
        for (uint32_t w = 1u; w < FIL_WORDS; w++)
            if (!word_program(pg, w, words[w])) return false;
        if (!word_program(pg, 0u, words[0])) return false;
        fw_fil_first_empty[ch] = 1u;
    }
    return page_erase(SCRATCH_PAGE);
}

// Flash_AMS_filament_write, then Flash_background_run on every pass until its job is done. True
// when flash holds the info (at once if the cache says so, which is also what the firmware's next
// call returns once the job completed); false when refused (fault), or when the power is lost or a
// read-back fails during the job. The firmware's call returns false while the job runs.
static bool fw_fil_write(uint8_t ch, const Flash_FilamentInfo *info)
{
    if (ch >= 4u) return false;
    if (fw_fault) return false;
    if (fw_fil_have[ch] && memcmp(&fw_fil_last[ch], info, sizeof(*info)) == 0) return true;

    uint32_t slot = fw_fil_first_empty[ch];
    const bool rollover = slot >= FIL_SLOTS;
    if (rollover) slot = 0u;
    uint32_t words[FIL_WORDS];
    nvm_fil_pack(words, info, crc32_words);
    if (rollover)
    {
        words[0] = FIL_SCRATCH_MAGIC | ch;
        words[FIL_WORDS - 1u] = crc32_words(words, (FIL_WORDS - 1u) * 4u);
    }
    fw_fil_first_empty[ch] = (uint8_t)(slot + 1u);

    const uint32_t pg = FIL_PAGE0 + ch;
    if (rollover)
    {
        // Job kind 3: the record into the erased scratch page. Kind 1: the filament page erased and
        // the record (MAGIC_FIL again) in its slot 0. Kind 4: the scratch page erased.
        if (!page_erase(SCRATCH_PAGE) || !job_program(SCRATCH_PAGE, 0u, words, 2u * FIL_WORDS)) return false;
        words[0] = MAGIC_FIL;
        words[FIL_WORDS - 1u] = crc32_words(words, (FIL_WORDS - 1u) * 4u);
        if (!page_erase(pg) || !job_program(pg, 0u, words, 2u * FIL_WORDS)) return false;
        if (!page_erase(SCRATCH_PAGE)) return false;
    }
    else if (!job_program(pg, slot * FIL_WORDS, words, 2u * FIL_WORDS))
        return false;

    memcpy(&fw_fil_last[ch], &words[1], sizeof(Flash_FilamentInfo));
    fw_fil_have[ch] = 1u;
    return true;
}

// ---- adapted from Flash_saves.cpp: the loaded-channel log (cache, scan at boot, write job) ----
// ---- anchor: Flash_AMS_state_read ----
// ---- anchor: sta_find_slot ----
// ---- anchor: Flash_AMS_state_write ----
static uint16_t fw_sta_seq;
static uint32_t fw_sta_slot;
static uint32_t fw_sta_saved_slot;
static uint8_t fw_sta_have;
static uint8_t fw_sta_loaded;

// Flash_AMS_state_read's first call (the scan); later calls return the cache.
static void fw_sta_read(void)
{
    uint8_t ch = 0xFFu;
    uint32_t next = 0u;
    const bool have = nvm_sta_scan(sta_log, STA_LOG_SLOTS, &ch, &fw_sta_seq, &next);
    fw_sta_slot = next;
    fw_sta_saved_slot = have ? (next + STA_LOG_SLOTS - 1u) % STA_LOG_SLOTS : 0u;
    fw_sta_have = have ? 1u : 0u;
    fw_sta_loaded = ch;
}

// Flash_AMS_state_write, then its job (kind 2: the page erase if sta_find_slot asks for one, then
// w1 and w0, the tag last). True / false as fw_fil_write.
static bool fw_sta_write(uint8_t ch)
{
    if (ch >= 4u && ch != 0xFFu) return false;
    if (fw_fault) return false;
    if (fw_sta_have && fw_sta_loaded == ch) return true;

    bool erase;
    const uint32_t slot =
        nvm_sta_find_slot(sta_log, STA_LOG_SLOTS, fw_sta_slot, fw_sta_have != 0u, fw_sta_saved_slot, &erase);
    uint32_t words[STA_WORDS];
    nvm_sta_pack(words, fw_sta_seq, ch);
    fw_sta_slot = (slot + 1u) % STA_LOG_SLOTS;

    const uint32_t pg = STA_PAGE0 + slot / STA_SLOTS;
    if (erase)
    {
        if (fw_sta_have && slot / STA_SLOTS == fw_sta_saved_slot / STA_SLOTS) sta_erased_newest++;
        if (!page_erase(pg)) return false;
    }
    if (!job_program(pg, (slot % STA_SLOTS) * STA_WORDS, words, 2u * STA_WORDS)) return false;

    fw_sta_seq = (uint16_t)(fw_sta_seq + 1u);
    fw_sta_saved_slot = slot;
    fw_sta_loaded = ch;
    fw_sta_have = 1u;
    return true;
}

// Power back on (budget: operations before it is cut again, -1 none) and Flash_saves_init: the
// filament caches, the loaded-channel scan, then the scratch recovery. The fault is cleared.
static void fw_boot(int32_t budget)
{
    power_lost = false;
    power_budget = budget;
    fail_budget = -1;
    fw_fault = false;
    for (uint8_t ch = 0u; ch < 4u; ch++)
    {
        uint32_t next = 0u;
        fw_fil_have[ch] = nvm_fil_load(flash[FIL_PAGE0 + ch], &fw_fil_last[ch], &next, crc32_words) ? 1u : 0u;
        fw_fil_first_empty[ch] = (uint8_t)next;
    }
    fw_sta_read();
    if (!fw_scratch_recover()) fw_fault = true;
}

// Only the loaded-channel part of a reboot (the filament pages are not used by those tests).
static void fw_sta_boot(void)
{
    power_lost = false;
    power_budget = -1;
    fail_budget = -1;
    fw_fault = false;
    fw_sta_read();
}

// 40 writes of changing info to channel 2 (rollover every 6th once the page is full: 3 erases), each
// followed by a reboot: the boot gives back the info just written and the very cache the firmware
// had (info, cursor). A write of the info flash already holds touches nothing.
static void test_fil_writes_then_reset_read_back(void)
{
    fw_boot(-1);
    TEST_ASSERT_EQUAL_UINT8(0u, fw_fil_have[2]);
    TEST_ASSERT_EQUAL_UINT8(0u, fw_fil_first_empty[2]);

    for (uint8_t i = 0u; i < 40u; i++)
    {
        const Flash_FilamentInfo info = info_for(i);
        const uint32_t ops = flash_ops;
        TEST_ASSERT_TRUE(fw_fil_write(2u, &info));
        TEST_ASSERT_EQUAL_UINT32((i >= FIL_SLOTS && i % FIL_SLOTS == 0u) ? 43u : 20u, flash_ops - ops);
        const uint32_t ops_after = flash_ops;
        TEST_ASSERT_TRUE(fw_fil_write(2u, &info));
        TEST_ASSERT_EQUAL_UINT32(ops_after, flash_ops);

        uint8_t have[4], first_empty[4];
        Flash_FilamentInfo last[4];
        memcpy(have, fw_fil_have, sizeof(have));
        memcpy(first_empty, fw_fil_first_empty, sizeof(first_empty));
        memcpy(last, fw_fil_last, sizeof(last));

        fw_boot(-1);
        TEST_ASSERT_FALSE(fw_fault);
        TEST_ASSERT_EQUAL_UINT8_ARRAY(have, fw_fil_have, 4);
        TEST_ASSERT_EQUAL_UINT8_ARRAY(first_empty, fw_fil_first_empty, 4);
        TEST_ASSERT_EQUAL_MEMORY(last, fw_fil_last, sizeof(last));
        TEST_ASSERT_INFO(info, fw_fil_last[2]);
    }
    TEST_ASSERT_EQUAL_UINT32(6u * 3u, erases);  // rollovers at writes 6, 12, ..., 36
    TEST_ASSERT_FALSE(programmed_over_used);
    for (uint32_t w = 0u; w < PAGE_WORDS; w++) TEST_ASSERT_EQUAL_HEX32(NVM_ERASED_WORD, flash[SCRATCH_PAGE][w]);
}

// Channel 2 holds `used` records (the newest is info_for(used - 1)) and channel 0 one record.
static void fil_boot_with(uint32_t used)
{
    setUp();
    for (uint32_t s = 0u; s < used; s++) fil_record_in(2u, s, (uint8_t)(100u + s));
    fil_record_in(0u, 0u, 7u);
    fw_boot(-1);
}

// A write of new info to channel 2, cut by a power loss after every number of operations: in the
// middle of a page (slot 2), into the page's last slot (5), and a rollover (page full: scratch
// erase, 20 halfwords, page erase, 20 halfwords, scratch erase). After the reboot channel 2 holds
// the old info or the new one, never nothing: the new one once the magic word is programmed (the
// record's last step), or for a rollover once the scratch record is complete (its last step, 21),
// which the boot then replays. No halfword is ever programmed over a used one, channel 0 is never
// touched, and the next write completes and survives a reboot.
static void test_fil_power_loss_at_every_step_keeps_the_old_or_the_new_info(void)
{
    const uint32_t useds[] = {2u, 5u, FIL_SLOTS};
    const Flash_FilamentInfo fresh = info_for(200u);
    const Flash_FilamentInfo next = info_for(201u);
    const Flash_FilamentInfo ch0 = info_for(7u);

    for (uint32_t u = 0u; u < sizeof(useds) / sizeof(useds[0]); u++)
    {
        const uint32_t used = useds[u];
        const bool rollover = used == FIL_SLOTS;
        const uint32_t total = rollover ? 43u : 20u;
        const Flash_FilamentInfo old = info_for((uint8_t)(100u + used - 1u));

        fil_boot_with(used);
        const uint32_t ops = flash_ops;
        TEST_ASSERT_TRUE(fw_fil_write(2u, &fresh));
        TEST_ASSERT_EQUAL_UINT32(total, flash_ops - ops);

        for (uint32_t cut = 0u; cut <= total; cut++)
        {
            fil_boot_with(used);
            power_budget = (int32_t)cut;
            TEST_ASSERT_EQUAL(cut == total, fw_fil_write(2u, &fresh));

            fw_boot(-1);
            TEST_ASSERT_FALSE(fw_fault);
            TEST_ASSERT_EQUAL_UINT8(1u, fw_fil_have[2]);
            const bool is_new = rollover ? cut >= 21u : cut == total;
            if (is_new)
            {
                TEST_ASSERT_INFO(fresh, fw_fil_last[2]);
                TEST_ASSERT_EQUAL_UINT8(rollover ? 1u : used + 1u, fw_fil_first_empty[2]);
            }
            else
            {
                TEST_ASSERT_INFO(old, fw_fil_last[2]);
                // A torn record in the slot is skipped: after it (or a rollover if it was the last).
                const uint32_t expect = (cut == 0u) ? used : (used + 1u < FIL_SLOTS ? used + 1u : FIL_SLOTS);
                TEST_ASSERT_EQUAL_UINT8(rollover ? FIL_SLOTS : expect, fw_fil_first_empty[2]);
            }
            TEST_ASSERT_INFO(ch0, fw_fil_last[0]);

            TEST_ASSERT_TRUE(fw_fil_write(2u, &next));
            fw_boot(-1);
            TEST_ASSERT_INFO(next, fw_fil_last[2]);
            TEST_ASSERT_INFO(ch0, fw_fil_last[0]);
            TEST_ASSERT_FALSE(fw_fault);
            TEST_ASSERT_FALSE_MESSAGE(programmed_over_used, "a halfword was programmed over a used one");
        }
    }
}

// A rollover cut once the scratch record is complete, then the boot's recovery cut too, at every
// step (page erase, 18 halfwords of words 1..9, word 0, scratch erase): the next boot finishes it,
// with the new info in slot 0 and the scratch page erased.
static void test_fil_power_loss_during_the_boot_recovery(void)
{
    const uint32_t job_cuts[] = {21u, 22u, 30u, 41u, 42u};
    const Flash_FilamentInfo fresh = info_for(200u);

    for (uint32_t j = 0u; j < sizeof(job_cuts) / sizeof(job_cuts[0]); j++)
    {
        for (uint32_t boot_cut = 0u; boot_cut <= 22u; boot_cut++)
        {
            fil_boot_with(FIL_SLOTS);
            power_budget = (int32_t)job_cuts[j];
            TEST_ASSERT_FALSE(fw_fil_write(2u, &fresh));

            fw_boot((int32_t)boot_cut);
            fw_boot(-1);
            TEST_ASSERT_FALSE(fw_fault);
            TEST_ASSERT_INFO(fresh, fw_fil_last[2]);
            TEST_ASSERT_EQUAL_UINT8(1u, fw_fil_first_empty[2]);
            for (uint32_t w = 0u; w < PAGE_WORDS; w++)
                TEST_ASSERT_EQUAL_HEX32(NVM_ERASED_WORD, flash[SCRATCH_PAGE][w]);
            TEST_ASSERT_FALSE(programmed_over_used);
        }
    }
}

// What a non-erasing programmer leaves from older images: a V7 whole-page record in channel 1's
// page, 0xFF in channel 3's, and the fork's 0xA5 loaded-channel records in the scratch page and the
// whole log. The boot loads nothing, erases nothing and leaves the scratch page alone (its words
// never match the scratch magic). The first writes roll the pages over (the scratch page is erased
// before it is used) and the loaded-channel write erases page 0 first: nothing is programmed over.
static void test_foreign_pages_are_never_programmed_over(void)
{
    uint32_t *const p1 = flash[FIL_PAGE0 + 1u];
    p1[0] = MAGIC_FIL;
    p1[1] = 0x00200001u;
    for (uint32_t w = 2u; w < 11u; w++) p1[w] = 0x41474600u + w;
    for (uint32_t w = 11u; w < PAGE_WORDS - 1u; w++) p1[w] = 0xFFFFFFFFu;
    p1[PAGE_WORDS - 1u] = 0x1234ABCDu;
    for (uint32_t w = 0u; w < PAGE_WORDS; w++) flash[FIL_PAGE0 + 3u][w] = 0xFFFFFFFFu;
    for (uint32_t s = 0u; s < (STA_LOG_PAGES + 1u) * STA_SLOTS; s++)  // the log, then page 14
    {
        const uint32_t w0 = (0xA5u << 24) | ((s & 0xFFFFu) << 8) | (s & 3u);
        sta_log[s * STA_WORDS] = w0;
        sta_log[s * STA_WORDS + 1u] = w0 ^ MAGIC_STA;
    }
    uint32_t scratch[PAGE_WORDS];
    memcpy(scratch, flash[SCRATCH_PAGE], sizeof(scratch));

    fw_boot(-1);
    TEST_ASSERT_FALSE(fw_fault);
    TEST_ASSERT_EQUAL_UINT32(0u, erases);
    TEST_ASSERT_EQUAL_MEMORY(scratch, flash[SCRATCH_PAGE], sizeof(scratch));
    TEST_ASSERT_EQUAL_UINT8(0u, fw_fil_have[1]);
    TEST_ASSERT_EQUAL_UINT8(FIL_SLOTS, fw_fil_first_empty[1]);
    TEST_ASSERT_EQUAL_UINT8(0u, fw_fil_have[3]);
    TEST_ASSERT_EQUAL_UINT8(FIL_SLOTS, fw_fil_first_empty[3]);
    TEST_ASSERT_EQUAL_UINT8(0u, fw_sta_have);
    TEST_ASSERT_EQUAL_HEX8(0xFFu, fw_sta_loaded);

    const Flash_FilamentInfo a = info_for(1u);
    const Flash_FilamentInfo b = info_for(3u);
    TEST_ASSERT_TRUE(fw_fil_write(1u, &a));
    TEST_ASSERT_TRUE(fw_fil_write(3u, &b));
    TEST_ASSERT_EQUAL_UINT32(6u, erases);
    TEST_ASSERT_TRUE(fw_sta_write(2u));
    TEST_ASSERT_EQUAL_UINT32(7u, erases);
    TEST_ASSERT_FALSE(programmed_over_used);

    fw_boot(-1);
    TEST_ASSERT_INFO(a, fw_fil_last[1]);
    TEST_ASSERT_INFO(b, fw_fil_last[3]);
    TEST_ASSERT_EQUAL_UINT8(1u, fw_fil_first_empty[1]);
    TEST_ASSERT_EQUAL_UINT8(1u, fw_sta_have);
    TEST_ASSERT_EQUAL_UINT8(2u, fw_sta_loaded);
    TEST_ASSERT_EQUAL_UINT32(1u, fw_sta_slot);
}

// A program whose read-back fails (the 6th halfword of the job): the sticky fault is set, the job
// is dropped and the cache keeps the old info; every later write is refused, even of the info the
// cache holds, and the loaded-channel writes too, until a reboot. After it the old info is loaded,
// the broken slot is skipped, and the write goes into the slot after it.
static void test_fil_failed_program_leaves_the_cache(void)
{
    const Flash_FilamentInfo fresh = info_for(200u);
    const Flash_FilamentInfo old = info_for(101u);

    fil_boot_with(2u);
    fail_budget = 5;
    TEST_ASSERT_FALSE(fw_fil_write(2u, &fresh));
    TEST_ASSERT_TRUE(fw_fault);
    TEST_ASSERT_INFO(old, fw_fil_last[2]);
    const uint32_t ops = flash_ops;
    TEST_ASSERT_FALSE(fw_fil_write(2u, &old));
    TEST_ASSERT_FALSE(fw_fil_write(2u, &fresh));
    TEST_ASSERT_FALSE(fw_sta_write(1u));
    TEST_ASSERT_EQUAL_UINT32(ops, flash_ops);

    fw_boot(-1);
    TEST_ASSERT_FALSE(fw_fault);
    TEST_ASSERT_INFO(old, fw_fil_last[2]);
    TEST_ASSERT_EQUAL_UINT8(3u, fw_fil_first_empty[2]);
    TEST_ASSERT_TRUE(fw_fil_write(2u, &fresh));
    fw_boot(-1);
    TEST_ASSERT_INFO(fresh, fw_fil_last[2]);
    TEST_ASSERT_EQUAL_UINT8(4u, fw_fil_first_empty[2]);
}

// A loaded-channel record: 0xA6 tag (V11), sequence, channel; w1 = w0 ^ MAGIC_STA. It reads back as
// the same sequence and channel; a torn (w1 erased), blank, retagged or altered record does not, and
// neither does a channel other than 0..3 or 0xFF (V11's scan filter).
static void test_sta_record_round_trip(void)
{
    const uint16_t seqs[] = {0u, 1u, 0x1234u, 0x7FFFu, 0x8000u, 0xFFFFu};
    uint32_t w[STA_WORDS];
    uint16_t seq;
    uint8_t ch;

    for (uint32_t i = 0u; i < sizeof(seqs) / sizeof(seqs[0]); i++)
    {
        for (uint32_t c = 0u; c < 256u; c++)
        {
            nvm_sta_pack(w, seqs[i], (uint8_t)c);
            TEST_ASSERT_EQUAL_HEX32((0xA6u << 24) | ((uint32_t)seqs[i] << 8) | c, w[0]);
            TEST_ASSERT_EQUAL_HEX32(w[0] ^ MAGIC_STA, w[1]);
            seq = 0u;
            ch = 0u;
            const bool want = c < 4u || c == 0xFFu;
            TEST_ASSERT_EQUAL(want, nvm_sta_valid(w, &seq, &ch));
            if (want)
            {
                TEST_ASSERT_EQUAL_UINT16(seqs[i], seq);
                TEST_ASSERT_EQUAL_UINT8(c, ch);
            }
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
    // Retagged but consistent: w1 still matches w0, the tag does not. 0xA5 is the tag of V10.x and
    // of the fork before V11: those records are ignored.
    const uint32_t tags[] = {0xA5u, 0xA4u, 0xA7u, 0x26u};
    for (uint32_t i = 0u; i < sizeof(tags) / sizeof(tags[0]); i++)
    {
        w[0] = (w0 & 0x00FFFFFFu) | (tags[i] << 24);
        w[1] = w[0] ^ MAGIC_STA;
        TEST_ASSERT_FALSE(nvm_sta_valid(w, &seq, &ch));
    }
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
// 10..255 the older ones. The next record goes into slot 10 with sequence 4. The newest in the last
// slot gives slot 0.
static void test_sta_scan_newest_across_both_wraps(void)
{
    uint8_t ch;
    uint16_t next_seq;
    uint32_t next_slot;

    // 266 records: record k went into slot k % 256 with sequence k - 262 (mod 2^16), so records
    // 256..265, in slots 0..9, run from 65530 through 0 to 3.
    for (uint32_t k = 0u; k < STA_LOG_SLOTS + 10u; k++)
        sta_record(k % STA_LOG_SLOTS, (uint16_t)(3u - (STA_LOG_SLOTS + 9u) + k), (uint8_t)(k & 3u));

    TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_UINT8((STA_LOG_SLOTS + 9u) & 3u, ch);
    TEST_ASSERT_EQUAL_UINT16(4u, next_seq);
    TEST_ASSERT_EQUAL_UINT32(10u, next_slot);
    TEST_ASSERT_EQUAL_HEX32((0xA6u << 24) | (65530u << 8) | (STA_LOG_SLOTS & 3u), sta_log[0]);

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

// Power lost while slot 37 (page 1, slot 5) was being programmed (w1 and w0's low halfword written,
// the tag not): the channel of slot 36 is restored, and the next record is its sequence + 1 in slot
// 37. The torn record's newer sequence does not count. The write skips the torn slot (it cannot be
// programmed over) and programs slot 38 without an erase. A newer, complete record with channel 7 is
// ignored the same way (V11's channel filter).
static void test_sta_scan_after_a_torn_slot(void)
{
    uint8_t ch;
    uint16_t next_seq;
    uint32_t next_slot;
    bool erase = true;

    for (uint32_t s = 0u; s < 37u; s++) sta_record(s, (uint16_t)(100u + s), (uint8_t)(s % 3u));
    sta_torn(37u, 137u, 3u, 3u);

    TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_UINT8(36u % 3u, ch);
    TEST_ASSERT_EQUAL_UINT16(137u, next_seq);  // 138 if the torn one counted
    TEST_ASSERT_EQUAL_UINT32(37u, next_slot);
    TEST_ASSERT_EQUAL_UINT32(38u, nvm_sta_find_slot(sta_log, STA_LOG_SLOTS, next_slot, true, 36u, &erase));
    TEST_ASSERT_FALSE(erase);

    sta_record(37u, 137u, 7u);
    TEST_ASSERT_TRUE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_UINT8(36u % 3u, ch);
    TEST_ASSERT_EQUAL_UINT32(37u, next_slot);

    // The only record is torn: none saved. V11 moves on to slot 1 instead of erasing page 0.
    setUp();
    sta_torn(0u, 5u, 1u, 2u);
    TEST_ASSERT_FALSE(sta_scan(&ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_HEX8(0xFFu, ch);
    TEST_ASSERT_EQUAL_UINT32(0u, next_slot);
    erase = true;
    TEST_ASSERT_EQUAL_UINT32(1u, nvm_sta_find_slot(sta_log, STA_LOG_SLOTS, next_slot, false, 0u, &erase));
    TEST_ASSERT_FALSE(erase);
}

// Slot 36 holds the newest record and slot 37 a torn one (the reset in the test above). The next
// write skips slot 37 and erases nothing: with power lost right before its first program, or after
// w1 (in slot 38), the reboot still restores slot 36's channel, never the older one, and the write
// after that lands in slot 39.
static void test_sta_torn_next_slot_keeps_the_newest_record(void)
{
    for (uint32_t s = 0u; s < 37u; s++) sta_record(s, (uint16_t)(100u + s), (s == 36u) ? 1u : 2u);
    sta_torn(37u, 137u, 3u, 3u);
    fw_sta_boot();
    TEST_ASSERT_EQUAL_UINT32(37u, fw_sta_slot);
    TEST_ASSERT_EQUAL_UINT32(36u, fw_sta_saved_slot);

    power_budget = 0;
    TEST_ASSERT_FALSE(fw_sta_write(3u));
    fw_sta_boot();
    TEST_ASSERT_EQUAL_UINT8(1u, fw_sta_loaded);
    TEST_ASSERT_EQUAL_UINT32(37u, fw_sta_slot);

    power_budget = 2;  // w1 into slot 38: torn again
    TEST_ASSERT_FALSE(fw_sta_write(3u));
    fw_sta_boot();
    TEST_ASSERT_EQUAL_UINT8(1u, fw_sta_loaded);
    TEST_ASSERT_EQUAL_UINT16(137u, fw_sta_seq);
    TEST_ASSERT_EQUAL_UINT32(37u, fw_sta_slot);

    TEST_ASSERT_TRUE(fw_sta_write(3u));
    TEST_ASSERT_EQUAL_UINT32(39u, fw_sta_saved_slot);
    fw_sta_boot();
    TEST_ASSERT_EQUAL_UINT8(3u, fw_sta_loaded);
    TEST_ASSERT_EQUAL_UINT16(138u, fw_sta_seq);
    TEST_ASSERT_EQUAL_UINT32(40u, fw_sta_slot);
    TEST_ASSERT_EQUAL_UINT32(0u, erases);
    TEST_ASSERT_FALSE(programmed_over_used);

    uint16_t seq;
    uint8_t c;
    for (uint32_t s = 0u; s < 37u; s++) TEST_ASSERT_TRUE(nvm_sta_valid(sta_log + s * STA_WORDS, &seq, &c));
}

// The log's wrap: the newest record in slot 254 and a torn one in slot 255 (the last), every other
// slot used. No erased slot is left, and page 7 holds the newest record, so the write erases page 0
// and goes into slot 0. Power lost right after that erase still restores slot 254's channel; the
// write after the reboot finds slot 0 erased and needs no second erase.
static void test_sta_torn_last_slot_wraps_to_slot_0(void)
{
    for (uint32_t s = 0u; s < STA_LOG_SLOTS - 1u; s++)
        sta_record(s, (uint16_t)(1000u + s), (s == STA_LOG_SLOTS - 2u) ? 1u : 2u);
    sta_torn(STA_LOG_SLOTS - 1u, (uint16_t)(1000u + STA_LOG_SLOTS - 1u), 3u, 1u);
    fw_sta_boot();
    TEST_ASSERT_EQUAL_UINT32(STA_LOG_SLOTS - 1u, fw_sta_slot);

    power_budget = 1;
    TEST_ASSERT_FALSE(fw_sta_write(3u));
    TEST_ASSERT_EQUAL_UINT32(1u, erases);
    fw_sta_boot();
    TEST_ASSERT_EQUAL_UINT8(1u, fw_sta_loaded);
    TEST_ASSERT_EQUAL_UINT16(1000u + STA_LOG_SLOTS - 1u, fw_sta_seq);

    TEST_ASSERT_TRUE(fw_sta_write(3u));
    TEST_ASSERT_EQUAL_UINT32(0u, fw_sta_saved_slot);
    fw_sta_boot();
    TEST_ASSERT_EQUAL_UINT8(3u, fw_sta_loaded);
    TEST_ASSERT_EQUAL_UINT16(1000u + STA_LOG_SLOTS, fw_sta_seq);
    TEST_ASSERT_EQUAL_UINT32(1u, fw_sta_slot);
    TEST_ASSERT_EQUAL_UINT32(1u, erases);
    TEST_ASSERT_EQUAL_UINT32(0u, sta_erased_newest);
    TEST_ASSERT_FALSE(programmed_over_used);
}

// 70000 loads and unloads (the sequence wraps at 65536, the log every 256 records), each followed by
// a reboot: the scan always restores the channel just written, with the sequence and slot the
// firmware had cached. A page is erased only when the log comes round to it again (none on the
// first pass), never the one with the newest record.
static void test_sta_writes_then_reset_read_back(void)
{
    fw_sta_boot();

    for (uint32_t i = 0u; i < 70000u; i++)
    {
        const uint8_t loaded = (i & 1u) ? 0xFFu : (uint8_t)((i >> 1) & 3u);
        TEST_ASSERT_TRUE(fw_sta_write(loaded));
        const uint16_t seq = fw_sta_seq;
        const uint32_t slot = fw_sta_slot;
        TEST_ASSERT_EQUAL_UINT32(i % STA_LOG_SLOTS, fw_sta_saved_slot);

        fw_sta_boot();
        TEST_ASSERT_EQUAL_HEX8(loaded, fw_sta_loaded);
        TEST_ASSERT_EQUAL_UINT16(seq, fw_sta_seq);
        TEST_ASSERT_EQUAL_UINT32(slot, fw_sta_slot);
    }
    TEST_ASSERT_EQUAL_UINT32((70000u - 1u) / STA_SLOTS - (STA_LOG_PAGES - 1u), erases);
    TEST_ASSERT_EQUAL_UINT32(0u, sta_erased_newest);
    TEST_ASSERT_FALSE(programmed_over_used);
}

// 20000 loads and unloads with power lost in a fixed pattern: every 5th write after 3 of its
// operations (the erase if it has one, then w1, or w1 and w0's low halfword), then its retry after 1
// (the erase, or w1's low halfword); and the write after it before its first operation. After every
// reboot the scan restores the last channel that was completely written, and the write is retried
// from what the scan gives, as after a real reset. Torn slots are skipped, pages are only erased
// when no erased slot is left and never the one with the newest record.
static void test_sta_power_loss_during_writes_keeps_the_last_record(void)
{
    uint8_t saved = 0xFFu;
    uint32_t resets = 0u;
    fw_sta_boot();

    for (uint32_t i = 0u; i < 20000u; i++)
    {
        const uint8_t loaded = (i & 1u) ? 0xFFu : (uint8_t)((i >> 1) & 3u);
        const int32_t first = (i % 5u == 3u) ? 3 : (i % 5u == 4u) ? 0 : -1;
        int32_t budget = first;

        for (;;)
        {
            power_budget = budget;
            if (fw_sta_write(loaded)) break;
            TEST_ASSERT_TRUE(power_lost);
            resets++;
            fw_sta_boot();
            TEST_ASSERT_EQUAL_HEX8_MESSAGE(saved, fw_sta_loaded, "a reset lost the newest record");
            budget = (budget == 3) ? 1 : -1;
        }
        saved = loaded;
        const uint16_t seq = fw_sta_seq;
        const uint32_t slot = fw_sta_slot;

        fw_sta_boot();
        TEST_ASSERT_EQUAL_HEX8(saved, fw_sta_loaded);
        TEST_ASSERT_EQUAL_UINT16(seq, fw_sta_seq);
        TEST_ASSERT_EQUAL_UINT32(slot, fw_sta_slot);
    }
    TEST_ASSERT_EQUAL_UINT32(12000u, resets);
    TEST_ASSERT_TRUE(erases > 0u);
    TEST_ASSERT_EQUAL_UINT32(0u, sta_erased_newest);
    TEST_ASSERT_FALSE(programmed_over_used);
}

// The newest record (channel 2) in slot 33. An unload whose second halfword fails its read-back:
// the sticky fault is set, the job dropped, and the cache still says channel 2 is saved, which is
// what flash holds. Every write is refused until the reboot, even of channel 2 (V11 checks the fault
// first). After it channel 2 is restored, the broken slot 34 is skipped and the unload goes into 35.
// No retry, so a failing page is not worn by repeated erases.
static void test_sta_failed_program_leaves_the_cache(void)
{
    for (uint32_t s = 0u; s < 34u; s++) sta_record(s, (uint16_t)(100u + s), (s == 33u) ? 2u : 3u);
    fw_sta_boot();

    fail_budget = 1;
    TEST_ASSERT_FALSE(fw_sta_write(0xFFu));
    TEST_ASSERT_TRUE(fw_fault);
    TEST_ASSERT_FALSE(power_lost);
    TEST_ASSERT_EQUAL_UINT8(1u, fw_sta_have);
    TEST_ASSERT_EQUAL_UINT8(2u, fw_sta_loaded);
    TEST_ASSERT_EQUAL_UINT32(33u, fw_sta_saved_slot);
    const uint32_t ops = flash_ops;
    TEST_ASSERT_FALSE(fw_sta_write(2u));
    TEST_ASSERT_FALSE(fw_sta_write(0xFFu));
    TEST_ASSERT_EQUAL_UINT32(ops, flash_ops);
    TEST_ASSERT_EQUAL_UINT32(0u, erases);

    fw_sta_boot();
    TEST_ASSERT_EQUAL_UINT8(2u, fw_sta_loaded);
    TEST_ASSERT_EQUAL_UINT32(34u, fw_sta_slot);
    TEST_ASSERT_TRUE(fw_sta_write(0xFFu));
    TEST_ASSERT_EQUAL_UINT32(35u, fw_sta_saved_slot);
    fw_sta_boot();
    TEST_ASSERT_EQUAL_HEX8(0xFFu, fw_sta_loaded);
    TEST_ASSERT_EQUAL_UINT16(135u, fw_sta_seq);
    TEST_ASSERT_EQUAL_UINT32(36u, fw_sta_slot);
}

// Flash_AMS_state_write refuses a channel that is neither 0..3 nor 0xFF at once: no job, nothing
// programmed. Writing the channel flash already holds touches nothing either.
static void test_sta_write_refuses_a_channel_out_of_range(void)
{
    fw_sta_boot();
    TEST_ASSERT_FALSE(fw_sta_write(4u));
    TEST_ASSERT_FALSE(fw_sta_write(0x80u));
    TEST_ASSERT_FALSE(fw_sta_write(0xFEu));
    TEST_ASSERT_EQUAL_UINT32(0u, flash_ops);

    TEST_ASSERT_TRUE(fw_sta_write(3u));
    TEST_ASSERT_EQUAL_UINT32(4u, flash_ops);
    TEST_ASSERT_TRUE(fw_sta_write(3u));
    TEST_ASSERT_EQUAL_UINT32(4u, flash_ops);
    TEST_ASSERT_TRUE(fw_sta_write(0xFFu));
    TEST_ASSERT_EQUAL_UINT32(8u, flash_ops);
}

// ---- Buffer calibration record (nvm_records.h nvm_cal_*, used by Flash_MC_PULL_cal_*) ----

static void test_cal_rsv_round_trips_every_polarity_and_mask(void)
{
    for (uint32_t pols = 0u; pols < 16u; pols++)
    {
        int8_t pol[4];
        for (uint8_t ch = 0u; ch < 4u; ch++)
            pol[ch] = (pols & (1u << ch)) ? -1 : 1;
        for (uint32_t mask = 0u; mask < 16u; mask++)
        {
            const uint32_t flags = nvm_cal_rsv_flags(pol, (uint8_t)(mask | 0xF0u)); // high bits ignored
            TEST_ASSERT_EQUAL_HEX32(pols | (mask << 8), flags);
            const uint32_t rsv = nvm_cal_rsv(0xBEEFu, flags);
            TEST_ASSERT_EQUAL_UINT16(0xBEEFu, nvm_cal_rsv_seq(rsv));
            TEST_ASSERT_EQUAL_UINT8(mask, nvm_cal_rsv_valid(rsv));
            for (uint8_t ch = 0u; ch < 4u; ch++)
                TEST_ASSERT_EQUAL_INT8(pol[ch], nvm_cal_rsv_pol(rsv, ch));
        }
    }
    // No polarity array: all +1, only the mask.
    TEST_ASSERT_EQUAL_HEX32(0x00000500u, nvm_cal_rsv_flags(nullptr, 0x05u));
}

// ---- adapted from Flash_saves.cpp: cal_latest's scan of slot 0 then slot 1 (whether each read is the test's) ----
// ---- anchor: cal_latest ----
static bool cal_scan(bool have0, uint32_t rsv0, bool have1, uint32_t rsv1, uint32_t *rsv, uint8_t *slot)
{
    const bool have_slot[2] = {have0, have1};
    const uint32_t rsv_slot[2] = {rsv0, rsv1};
    bool have = false;
    for (uint8_t i = 0u; i < 2u; i++)
    {
        if (!have_slot[i]) continue;
        if (nvm_cal_take(have, rsv_slot[i], *rsv))
        {
            *rsv = rsv_slot[i];
            *slot = i;
            have = true;
        }
    }
    return have;
}

static void test_cal_reads_the_newer_slot_and_writes_the_other(void)
{
    uint32_t rsv = 0u;
    uint8_t slot = 0u;

    // Neither slot: the first write goes to slot 0 with sequence 0.
    TEST_ASSERT_FALSE(cal_scan(false, 0u, false, 0u, &rsv, &slot));
    TEST_ASSERT_EQUAL_UINT8(0u, nvm_cal_write_slot(false, slot));
    TEST_ASSERT_EQUAL_UINT16(0u, nvm_cal_next_seq(false, rsv));

    // Only one slot: it is read whatever its sequence, and the write goes to the other.
    TEST_ASSERT_TRUE(cal_scan(true, nvm_cal_rsv(7u, 0x0F0Au), false, 0u, &rsv, &slot));
    TEST_ASSERT_EQUAL_UINT8(0u, slot);
    TEST_ASSERT_EQUAL_HEX32(nvm_cal_rsv(7u, 0x0F0Au), rsv);
    TEST_ASSERT_EQUAL_UINT8(1u, nvm_cal_write_slot(true, slot));
    TEST_ASSERT_EQUAL_UINT16(8u, nvm_cal_next_seq(true, rsv));
    rsv = 0u;
    TEST_ASSERT_TRUE(cal_scan(false, 0u, true, nvm_cal_rsv(0x9000u, 0u), &rsv, &slot));
    TEST_ASSERT_EQUAL_UINT8(1u, slot);
    TEST_ASSERT_EQUAL_UINT8(0u, nvm_cal_write_slot(true, slot));
    TEST_ASSERT_EQUAL_UINT16(0x9001u, nvm_cal_next_seq(true, rsv));

    // Both: the newer by sequence, either way round.
    TEST_ASSERT_TRUE(cal_scan(true, nvm_cal_rsv(4u, 0x0100u), true, nvm_cal_rsv(5u, 0x0200u), &rsv, &slot));
    TEST_ASSERT_EQUAL_UINT8(1u, slot);
    TEST_ASSERT_EQUAL_UINT8(2u, nvm_cal_rsv_valid(rsv));
    TEST_ASSERT_TRUE(cal_scan(true, nvm_cal_rsv(6u, 0x0100u), true, nvm_cal_rsv(5u, 0x0200u), &rsv, &slot));
    TEST_ASSERT_EQUAL_UINT8(0u, slot);
    TEST_ASSERT_EQUAL_UINT8(1u, nvm_cal_rsv_valid(rsv));

    // Across the wrap: 0 was written after 0xFFFF.
    TEST_ASSERT_TRUE(cal_scan(true, nvm_cal_rsv(0xFFFFu, 0u), true, nvm_cal_rsv(0u, 0u), &rsv, &slot));
    TEST_ASSERT_EQUAL_UINT8(1u, slot);
    TEST_ASSERT_EQUAL_UINT16(1u, nvm_cal_next_seq(true, rsv));
    TEST_ASSERT_TRUE(cal_scan(true, nvm_cal_rsv(0u, 0u), true, nvm_cal_rsv(0xFFFFu, 0u), &rsv, &slot));
    TEST_ASSERT_EQUAL_UINT8(0u, slot);
    // From 0xFFFF the next is 0.
    TEST_ASSERT_EQUAL_UINT16(0u, nvm_cal_next_seq(true, nvm_cal_rsv(0xFFFFu, 0u)));

    // Not ordered (equal): slot 0.
    TEST_ASSERT_TRUE(cal_scan(true, nvm_cal_rsv(9u, 0u), true, nvm_cal_rsv(9u, 0u), &rsv, &slot));
    TEST_ASSERT_EQUAL_UINT8(0u, slot);

    // Alternation: each write lands on the other slot with the next sequence, and is read back.
    bool have[2] = {false, false};
    uint32_t page[2] = {0u, 0u};
    for (uint32_t n = 0u; n < 6u; n++)
    {
        rsv = 0u;
        slot = 0u;
        const bool any = cal_scan(have[0], page[0], have[1], page[1], &rsv, &slot);
        const uint8_t w = nvm_cal_write_slot(any, slot);
        TEST_ASSERT_EQUAL_UINT8((uint8_t)(n & 1u), w);
        page[w] = nvm_cal_rsv(nvm_cal_next_seq(any, rsv), (uint32_t)n);
        have[w] = true;
        TEST_ASSERT_TRUE(cal_scan(have[0], page[0], have[1], page[1], &rsv, &slot));
        TEST_ASSERT_EQUAL_UINT8(w, slot);
        TEST_ASSERT_EQUAL_UINT16((uint16_t)n, nvm_cal_rsv_seq(rsv));
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_crc32_model_matches_the_wch_example);
    RUN_TEST(test_fil_record_round_trip);
    RUN_TEST(test_fil_scan_gives_the_last_record_and_the_next_slot);
    RUN_TEST(test_fil_torn_record_mid_page);
    RUN_TEST(test_fil_torn_first_record_is_skipped);
    RUN_TEST(test_fil_ff_padded_slots_from_an_older_layout);
    RUN_TEST(test_fil_blank_page);
    RUN_TEST(test_fil_writes_then_reset_read_back);
    RUN_TEST(test_fil_power_loss_at_every_step_keeps_the_old_or_the_new_info);
    RUN_TEST(test_fil_power_loss_during_the_boot_recovery);
    RUN_TEST(test_foreign_pages_are_never_programmed_over);
    RUN_TEST(test_fil_failed_program_leaves_the_cache);
    RUN_TEST(test_sta_record_round_trip);
    RUN_TEST(test_sta_seq_compare_across_the_wrap);
    RUN_TEST(test_sta_scan_blank_log);
    RUN_TEST(test_sta_scan_newest_across_both_wraps);
    RUN_TEST(test_sta_scan_after_a_torn_slot);
    RUN_TEST(test_sta_torn_next_slot_keeps_the_newest_record);
    RUN_TEST(test_sta_torn_last_slot_wraps_to_slot_0);
    RUN_TEST(test_sta_writes_then_reset_read_back);
    RUN_TEST(test_sta_power_loss_during_writes_keeps_the_last_record);
    RUN_TEST(test_sta_failed_program_leaves_the_cache);
    RUN_TEST(test_sta_write_refuses_a_channel_out_of_range);
    RUN_TEST(test_cal_rsv_round_trips_every_polarity_and_mask);
    RUN_TEST(test_cal_reads_the_newer_slot_and_writes_the_other);
    return UNITY_END();
}
