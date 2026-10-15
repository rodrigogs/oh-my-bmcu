// Host tests for where Flash_saves.cpp programs the next journal record: the filament-info cursor
// (nvm_fil_scan_page, nvm_records.h) and the loaded-channel slot (nvm_sta_find_slot, nvm_journal.h).
// Both must pick only a slot whose words are all exactly erased (0xE339E339): a torn record, an
// older firmware's layout or a 0xFF fill is skipped, never programmed over. Programming over it
// fails the read-back, and V11's program path then sets its sticky flash fault, which stops every
// NVM write (CAL included) until reboot, and again at every boot that picks the same slot.

#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "Flash_saves.h"
#include "nvm_journal.h"
#include "nvm_records.h"

// Journal geometry of Flash_saves.cpp (nvm_journal.h). Filament info: one 256-byte page per slot, 6
// records of 10 words (MAGIC_FIL, 32-byte info, CRC). Loaded channel: 32 records of 2 words per page,
// over NVM_STA_LOG_PAGES pages.
static const uint32_t PAGE_WORDS = FLASH_NVM256_PAGE_SIZE / 4u;
static const uint32_t FIL_WORDS = NVM_FIL_SLOT_WORDS;
static const uint32_t FIL_SLOTS = NVM_FIL_SLOTS_PER_PAGE;
static const uint32_t STA_WORDS = NVM_STA_SLOT_WORDS;
static const uint32_t STA_SLOTS = NVM_STA_SLOTS_PER_PAGE;
static const uint32_t STA_PAGES = NVM_STA_LOG_PAGES;
static const uint32_t STA_LOG_SLOTS = STA_PAGES * STA_SLOTS;
static const uint32_t MAGIC_FIL2 = 0x324C4946u; // 'FIL2', V10.0 filament records

// The page or log under test followed by an erased guard page, so a helper that looks past its
// range reads erased words there and gives itself away. The filament tests use the first page.
static uint32_t mem[(NVM_STA_LOG_PAGES + 1u) * (FLASH_NVM256_PAGE_SIZE / 4u)];
static uint32_t *const page = mem;
static uint32_t *const sta_log = mem;

void setUp(void)
{
    for (uint32_t i = 0; i < (STA_PAGES + 1u) * PAGE_WORDS; i++) mem[i] = NVM_ERASED_WORD;
}

void tearDown(void) {}

// The filament cursor only asks whether words are erased, so the CRC is a stand-in.
static uint32_t crc_stand_in(const void *, uint32_t)
{
    return 0x5A5A5A5Au;
}

// fil_cache_load_one's cursor for the page: the slot the next record goes into, 6 = roll over.
static uint32_t fil_cursor(void)
{
    uint32_t first_empty = 0xDEADu;
    Flash_FilamentInfo last;
    (void)nvm_fil_scan_page(page, &last, &first_empty, crc_stand_in);
    return first_empty;
}

// A filament record in slot `slot`, its info filled from `tag` (nvm_records.h packing).
static void fil_record(uint32_t slot, uint32_t tag)
{
    uint32_t info_words[sizeof(Flash_FilamentInfo) / 4u];
    for (uint32_t w = 0u; w < sizeof(info_words) / 4u; w++) info_words[w] = tag * 0x01010101u + w + 1u;
    Flash_FilamentInfo info;
    memcpy(&info, info_words, sizeof(info));
    nvm_fil_pack(page + slot * FIL_WORDS, &info, crc_stand_in);
}

// A loaded-channel record: 0xA6 tag, sequence number, channel, then w0 ^ MAGIC_STA.
static void sta_record(uint32_t slot, uint16_t seq, uint8_t ch)
{
    nvm_sta_pack(sta_log + slot * STA_WORDS, seq, ch);
}

// A record cut by a power loss: Flash_saves.cpp programs w1 (two halfwords), then w0's low and high
// halfwords. `halfwords` of the four were programmed.
static void sta_torn(uint32_t slot, uint16_t seq, uint8_t ch, uint32_t halfwords)
{
    uint32_t w[STA_WORDS];
    nvm_sta_pack(w, seq, ch);
    uint32_t *const p = sta_log + slot * STA_WORDS;
    p[0] = NVM_ERASED_WORD;
    p[1] = NVM_ERASED_WORD;
    for (uint32_t h = 0u; h < halfwords; h++)
    {
        const uint32_t index = (h + 2u) % 4u; // flash_job_halfword: 2, 3, 0, 1
        const uint32_t shift = (index & 1u) * 16u;
        p[index >> 1] = (p[index >> 1] & ~(0xFFFFu << shift)) | (((w[index >> 1] >> shift) & 0xFFFFu) << shift);
    }
}

// Flash_AMS_state_write's choice: from start (the slot after the newest), with the newest record
// in saved_slot (have_saved), the slot it programs and whether it erases that slot's page first.
static uint32_t sta_find(uint32_t start, bool have_saved, uint32_t saved_slot, bool *erase)
{
    *erase = !*erase; // must be written either way
    return nvm_sta_find_slot(sta_log, STA_LOG_SLOTS, start, have_saved, saved_slot, erase);
}

static void sta_fill_log(uint32_t first_slot, uint32_t end_slot, uint16_t seq0)
{
    for (uint32_t s = first_slot; s < end_slot; s++) sta_record(s, (uint16_t)(seq0 + s), (uint8_t)(s & 3u));
}

static void test_words_erased_checks_exactly_count_words(void)
{
    TEST_ASSERT_TRUE(nvm_words_erased(page, 0u));
    TEST_ASSERT_TRUE(nvm_words_erased(page, PAGE_WORDS));

    mem[PAGE_WORDS] = 0x00000000u; // just past the range
    TEST_ASSERT_TRUE(nvm_words_erased(page, PAGE_WORDS));

    page[PAGE_WORDS - 1u] = 0x00000000u;
    TEST_ASSERT_FALSE(nvm_words_erased(page, PAGE_WORDS));
}

// Erased reads as 0xE339E339 on CH32V20x. 0xFFFFFFFF, which record detection and erase verification
// take for blank (nvm_word_is_blank), is a programmed word and must not be written over.
static void test_ff_word_is_not_erased(void)
{
    page[0] = 0xFFFFFFFFu;
    TEST_ASSERT_FALSE(nvm_words_erased(page, 1u));
    TEST_ASSERT_TRUE(nvm_word_is_blank(page[0]));
    page[0] = 0xE339FFFFu; // one half-word programmed
    TEST_ASSERT_FALSE(nvm_words_erased(page, 1u));
    page[0] = 0xFFFFE339u;
    TEST_ASSERT_FALSE(nvm_words_erased(page, 1u));
}

// Erased page (after a page erase or a full NVM clear): the cursor is slot 0. Records in slots 0..2:
// slot 3. Full page: 6, so the next write rolls the page over through the scratch page.
static void test_fil_cursor_blank_valid_and_full_page(void)
{
    TEST_ASSERT_EQUAL_UINT32(0u, fil_cursor());

    for (uint32_t s = 0u; s < 3u; s++) fil_record(s, s + 1u);
    TEST_ASSERT_EQUAL_UINT32(3u, fil_cursor());

    for (uint32_t s = 3u; s < FIL_SLOTS; s++) fil_record(s, s + 1u);
    TEST_ASSERT_EQUAL_UINT32(FIL_SLOTS, fil_cursor());
}

// V11's rule: the cursor is the first erased slot AFTER the last slot that is not erased. An erased
// slot before a used one is never the cursor (the scan and the write would disagree on the order).
static void test_fil_cursor_goes_after_the_last_used_slot(void)
{
    fil_record(1u, 1u);
    TEST_ASSERT_EQUAL_UINT32(2u, fil_cursor());

    setUp();
    fil_record(0u, 1u);
    fil_record(4u, 2u);
    TEST_ASSERT_EQUAL_UINT32(5u, fil_cursor());

    setUp();
    fil_record(5u, 1u);
    TEST_ASSERT_EQUAL_UINT32(FIL_SLOTS, fil_cursor());
}

// Power lost during the first record after a page erase. V11 programs halfwords from word 1 on and
// word 0 (the magic) last: words 1..3 written, the rest erased. Or every word but the magic's upper
// half. Either way the slot is not erased, so the cursor moves past it: slot 1, never slot 0.
static void test_fil_torn_first_record_is_skipped(void)
{
    fil_record(0u, 7u);
    page[0] = NVM_ERASED_WORD;
    for (uint32_t w = 4u; w < FIL_WORDS; w++) page[w] = NVM_ERASED_WORD;
    TEST_ASSERT_EQUAL_UINT32(1u, fil_cursor());

    setUp();
    fil_record(0u, 7u);
    page[0] = (page[0] & 0x0000FFFFu) | 0xE3390000u;
    TEST_ASSERT_EQUAL_UINT32(1u, fil_cursor());

    // Only one word of the slot programmed (its last one, the CRC): still not free.
    setUp();
    page[FIL_WORDS - 1u] = 0x12345678u;
    TEST_ASSERT_EQUAL_UINT32(1u, fil_cursor());
}

// A torn record after valid ones: the cursor goes to the slot after it, and the valid records stay.
// Torn in the last slot: the next write rolls the page over.
static void test_fil_torn_record_after_valid_ones(void)
{
    fil_record(0u, 1u);
    fil_record(1u, 2u);
    fil_record(2u, 3u);
    for (uint32_t w = 2u * FIL_WORDS + 5u; w < 3u * FIL_WORDS; w++) page[w] = NVM_ERASED_WORD;
    TEST_ASSERT_EQUAL_UINT32(3u, fil_cursor());

    for (uint32_t s = 3u; s < FIL_SLOTS; s++) fil_record(s, s + 1u);
    page[5u * FIL_WORDS] = NVM_ERASED_WORD;
    TEST_ASSERT_EQUAL_UINT32(FIL_SLOTS, fil_cursor());
}

// V7..V10.1 whole-page nvm256 'FIL1' record (git 4716fd2) left by a programmer that did not erase
// the NVM sector: header, 32-byte info, 0xFF padding, CRC in the last word. No slot is erased (the
// 0xFF padding is programmed), so the cursor is 6 and the first write rolls the page over (erases
// it) instead of programming over the padding. V11's lenient test would have given slot 2.
static void test_v7_whole_page_record_rolls_over(void)
{
    page[0] = MAGIC_FIL;
    page[1] = 0x00200001u; // ver 1, len 32
    page[2] = 0x00FF0000u; // rsv: AMS 0, filament 0, not loaded
    for (uint32_t w = 3u; w < 11u; w++) page[w] = 0x41474600u + w;
    for (uint32_t w = 11u; w < PAGE_WORDS - 1u; w++) page[w] = 0xFFFFFFFFu;
    page[PAGE_WORDS - 1u] = 0x1234ABCDu;

    TEST_ASSERT_EQUAL_UINT32(FIL_SLOTS, fil_cursor());
}

// V10.0 'FIL2' 64-byte record (git 524bcd3), programmed into an erased page: words 0..15, which
// cover slots 0 and 1. Slot 2 onwards is erased, so the next record goes there.
static void test_v10_fil2_record_is_skipped(void)
{
    page[0] = MAGIC_FIL2;
    page[1] = 0x00200002u;
    for (uint32_t w = 2u; w < 15u; w++) page[w] = 0x10203040u + w;
    page[15] = 0xCAFEF00Du;

    TEST_ASSERT_EQUAL_UINT32(2u, fil_cursor());
}

// NVM sector programmed with 0xFF (a 0xFF-padded image written over it, a non-erasing programmer):
// every slot looks blank to V11's lenient test but none is erased. Cursor 6: roll over. The same
// for 0xFF slots after valid records.
static void test_ff_filled_page_rolls_over(void)
{
    for (uint32_t i = 0u; i < PAGE_WORDS; i++) page[i] = 0xFFFFFFFFu;
    TEST_ASSERT_EQUAL_UINT32(FIL_SLOTS, fil_cursor());

    fil_record(0u, 1u);
    fil_record(1u, 2u);
    TEST_ASSERT_EQUAL_UINT32(FIL_SLOTS, fil_cursor());
}

// Any single word of a slot that is not erased makes it used, and only that slot's words count:
// the last word of the previous slot and the first word of the next one do not.
static void test_every_word_of_the_slot_and_only_it_is_checked(void)
{
    for (uint32_t w = 0u; w < FIL_WORDS; w++)
    {
        setUp();
        page[2u * FIL_WORDS + w] = 0xFFFFFFFFu;
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(3u, fil_cursor(), "word inside the slot");
    }

    setUp();
    page[2u * FIL_WORDS - 1u] = 0x00000000u; // slot 1's last word
    TEST_ASSERT_EQUAL_UINT32(2u, fil_cursor());

    setUp();
    page[3u * FIL_WORDS] = 0x00000000u; // slot 3's first word
    TEST_ASSERT_EQUAL_UINT32(4u, fil_cursor());

    // The 4 words after the 6 slots (60..63) are not a slot.
    setUp();
    page[FIL_SLOTS * FIL_WORDS] = 0x00000000u;
    TEST_ASSERT_EQUAL_UINT32(0u, fil_cursor());

    // The same for a loaded-channel slot: either of its words.
    for (uint32_t w = 0u; w < STA_WORDS; w++)
    {
        setUp();
        sta_log[5u * STA_WORDS + w] = 0x00000000u;
        bool erase = false;
        TEST_ASSERT_EQUAL_UINT32(6u, sta_find(5u, false, 0u, &erase));
        TEST_ASSERT_FALSE(erase);
        TEST_ASSERT_EQUAL_UINT32(4u, sta_find(4u, false, 0u, &erase));
        TEST_ASSERT_EQUAL_UINT32(6u, sta_find(6u, false, 0u, &erase));
    }
}

// Erased log: the slot after the newest record is used as it is, wherever it is, with no erase.
static void test_sta_blank_log_uses_the_start_slot(void)
{
    const uint32_t starts[] = {0u, 1u, 31u, 32u, 100u, STA_LOG_SLOTS - 1u};
    for (uint32_t i = 0u; i < sizeof(starts) / sizeof(starts[0]); i++)
    {
        bool erase = true;
        TEST_ASSERT_EQUAL_UINT32(starts[i], sta_find(starts[i], false, 0u, &erase));
        TEST_ASSERT_FALSE(erase);
    }
}

// Records in slots 0..9, the newest in 9: the next goes into slot 10, no erase.
static void test_sta_free_slot_after_the_newest(void)
{
    bool erase = true;
    sta_fill_log(0u, 10u, 200u);
    TEST_ASSERT_EQUAL_UINT32(10u, sta_find(10u, true, 9u, &erase));
    TEST_ASSERT_FALSE(erase);
}

// Torn records after the newest one (slot 2), for every way a power loss can cut one (1..3 of its 4
// halfwords): skipped, so slot 4, then 5 when slot 4 is torn too. The newest record's page is never
// erased for it.
static void test_sta_torn_slot_after_the_newest_is_skipped(void)
{
    for (uint32_t cut = 1u; cut < 4u; cut++)
    {
        bool erase = true;
        setUp();
        sta_fill_log(0u, 3u, 40u);
        sta_torn(3u, 43u, 2u, cut);
        TEST_ASSERT_EQUAL_UINT32(4u, sta_find(3u, true, 2u, &erase));
        TEST_ASSERT_FALSE(erase);

        sta_torn(4u, 43u, 2u, cut);
        TEST_ASSERT_EQUAL_UINT32(5u, sta_find(3u, true, 2u, &erase));
        TEST_ASSERT_FALSE(erase);
    }
}

// Torn slots up to the end of the page with the newest record: the search goes on into the next
// page. Erased there: used without an erase. That page full of older records: V11 keeps going round
// the log to the first erased slot (page 2 here), still with no erase.
static void test_sta_search_goes_on_over_the_whole_log(void)
{
    bool erase = true;

    sta_fill_log(0u, 30u, 500u);
    for (uint32_t s = 30u; s < STA_SLOTS; s++) sta_torn(s, 530u, 2u, 3u);
    TEST_ASSERT_EQUAL_UINT32(STA_SLOTS, sta_find(30u, true, 29u, &erase));
    TEST_ASSERT_FALSE(erase);

    sta_fill_log(STA_SLOTS, 2u * STA_SLOTS, 100u);
    TEST_ASSERT_EQUAL_UINT32(2u * STA_SLOTS, sta_find(30u, true, 29u, &erase));
    TEST_ASSERT_FALSE(erase);

    // Only slots 0..4 are erased: from slot 200 the search wraps round to slot 0.
    setUp();
    sta_fill_log(5u, STA_LOG_SLOTS, 1000u);
    TEST_ASSERT_EQUAL_UINT32(0u, sta_find(200u, true, 199u, &erase));
    TEST_ASSERT_FALSE(erase);
}

// Every slot used: erase the start slot's page and use its slot 0, unless that page holds the
// newest record: then the page after it, wrapping from the last page (7) to page 0.
static void test_sta_full_log_erases_a_page_without_the_newest(void)
{
    bool erase = false;
    sta_fill_log(0u, STA_LOG_SLOTS, 3000u);

    TEST_ASSERT_EQUAL_UINT32(3u * STA_SLOTS, sta_find(3u * STA_SLOTS, true, 3u * STA_SLOTS - 1u, &erase));
    TEST_ASSERT_TRUE(erase);

    TEST_ASSERT_EQUAL_UINT32(4u * STA_SLOTS, sta_find(3u * STA_SLOTS + 4u, true, 3u * STA_SLOTS + 3u, &erase));
    TEST_ASSERT_TRUE(erase);

    // The newest in the log's last slot: start wraps to 0, page 0 holds nothing newer.
    TEST_ASSERT_EQUAL_UINT32(0u, sta_find(0u, true, STA_LOG_SLOTS - 1u, &erase));
    TEST_ASSERT_TRUE(erase);

    // The newest in page 7 with a torn slot after it in the same page: page 0.
    TEST_ASSERT_EQUAL_UINT32(0u, sta_find(STA_LOG_SLOTS - 5u, true, STA_LOG_SLOTS - 6u, &erase));
    TEST_ASSERT_TRUE(erase);

    // For every newest slot, the erased page never holds it.
    for (uint32_t saved = 0u; saved < STA_LOG_SLOTS; saved++)
    {
        for (uint32_t ahead = 1u; ahead <= 2u; ahead++)
        {
            const uint32_t start = (saved + ahead) % STA_LOG_SLOTS;
            const uint32_t slot = sta_find(start, true, saved, &erase);
            TEST_ASSERT_TRUE(erase);
            TEST_ASSERT_EQUAL_UINT32(0u, slot % STA_SLOTS);
            TEST_ASSERT_NOT_EQUAL(saved / STA_SLOTS, slot / STA_SLOTS);
        }
    }
}

// 0xFFFFFFFF slots are never free: a log programmed with 0xFF is a full log (erase), and a single
// 0xFF slot among erased ones is skipped like a torn record.
static void test_sta_ff_slots_are_never_free(void)
{
    bool erase = false;
    for (uint32_t i = 0u; i < STA_LOG_SLOTS * STA_WORDS; i++) sta_log[i] = 0xFFFFFFFFu;
    TEST_ASSERT_EQUAL_UINT32(0u, sta_find(0u, false, 0u, &erase));
    TEST_ASSERT_TRUE(erase);

    setUp();
    sta_log[7u * STA_WORDS] = 0xFFFFFFFFu;
    sta_log[7u * STA_WORDS + 1u] = 0xFFFFFFFFu;
    TEST_ASSERT_EQUAL_UINT32(8u, sta_find(7u, false, 0u, &erase));
    TEST_ASSERT_FALSE(erase);
}

// No saved record and a log full of foreign data (V10.0 'FIL2' page-B records, or the fork's 0xA5
// loaded-channel records, which V11 ignores): the scan gives start 0, so page 0 is erased first.
static void test_sta_foreign_full_log_erases_page_0(void)
{
    bool erase = false;

    for (uint32_t r = 0u; r < STA_LOG_SLOTS * STA_WORDS / 16u; r++) // 64-byte records
    {
        sta_log[r * 16u] = MAGIC_FIL2;
        for (uint32_t w = 1u; w < 16u; w++) sta_log[r * 16u + w] = 0x00010000u * r + w;
    }
    TEST_ASSERT_EQUAL_UINT32(0u, sta_find(0u, false, 0u, &erase));
    TEST_ASSERT_TRUE(erase);

    setUp();
    for (uint32_t s = 0u; s < STA_LOG_SLOTS; s++)
    {
        const uint32_t w0 = (0xA5u << 24) | ((uint32_t)s << 8) | (s & 3u);
        sta_log[s * STA_WORDS] = w0;
        sta_log[s * STA_WORDS + 1u] = w0 ^ MAGIC_STA;
    }
    uint8_t ch;
    uint16_t next_seq;
    uint32_t next_slot;
    TEST_ASSERT_FALSE(nvm_sta_scan(sta_log, STA_LOG_SLOTS, &ch, &next_seq, &next_slot));
    TEST_ASSERT_EQUAL_UINT32(0u, sta_find(next_slot, false, 0u, &erase));
    TEST_ASSERT_TRUE(erase);
}

// The log is 8 pages (6..13): page 14 is the filament rollover scratch page and 15 the CAL backup.
static void test_sta_log_geometry(void)
{
    TEST_ASSERT_EQUAL_UINT32(8u, NVM_STA_LOG_PAGES);
    TEST_ASSERT_EQUAL_UINT32(256u, STA_LOG_SLOTS);
    TEST_ASSERT_EQUAL_UINT32(FLASH_NVM256_PAGE_SIZE, NVM_STA_SLOTS_PER_PAGE * NVM_STA_SLOT_WORDS * 4u);
    TEST_ASSERT_TRUE(NVM_FIL_SLOTS_PER_PAGE * NVM_FIL_SLOT_WORDS * 4u <= FLASH_NVM256_PAGE_SIZE);
    TEST_ASSERT_EQUAL_UINT32(FLASH_NVM_PAGE_COUNT, 6u + NVM_STA_LOG_PAGES + 2u);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_words_erased_checks_exactly_count_words);
    RUN_TEST(test_ff_word_is_not_erased);
    RUN_TEST(test_fil_cursor_blank_valid_and_full_page);
    RUN_TEST(test_fil_cursor_goes_after_the_last_used_slot);
    RUN_TEST(test_fil_torn_first_record_is_skipped);
    RUN_TEST(test_fil_torn_record_after_valid_ones);
    RUN_TEST(test_v7_whole_page_record_rolls_over);
    RUN_TEST(test_v10_fil2_record_is_skipped);
    RUN_TEST(test_ff_filled_page_rolls_over);
    RUN_TEST(test_every_word_of_the_slot_and_only_it_is_checked);
    RUN_TEST(test_sta_blank_log_uses_the_start_slot);
    RUN_TEST(test_sta_free_slot_after_the_newest);
    RUN_TEST(test_sta_torn_slot_after_the_newest_is_skipped);
    RUN_TEST(test_sta_search_goes_on_over_the_whole_log);
    RUN_TEST(test_sta_full_log_erases_a_page_without_the_newest);
    RUN_TEST(test_sta_ff_slots_are_never_free);
    RUN_TEST(test_sta_foreign_full_log_erases_page_0);
    RUN_TEST(test_sta_log_geometry);
    return UNITY_END();
}
