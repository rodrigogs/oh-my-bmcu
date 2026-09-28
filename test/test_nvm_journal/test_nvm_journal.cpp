// Host tests for src/nvm_journal.h: before Flash_saves.cpp programs the next filament-info or
// loaded-channel record, a slot that is not erased (a torn record, an older firmware's layout, a
// 0xFF fill) must make it erase the page first, or in the loaded-channel log skip the slot while
// its page holds the newest record. Otherwise the program fails its read-back and every later save
// of that slot fails the same way, until a full NVM wipe.

#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "Flash_saves.h"
#include "nvm_journal.h"
#include "nvm_records.h"

// Journal geometry of Flash_saves.cpp (nvm_journal.h). Filament info: one 256-byte page per slot, 6
// records of 10 words (MAGIC_FIL, 32-byte info, CRC). Loaded channel: 32 records of 2 words per page.
static const uint32_t PAGE_WORDS = FLASH_NVM256_PAGE_SIZE / 4u;
static const uint32_t FIL_WORDS = NVM_FIL_SLOT_WORDS;
static const uint32_t FIL_SLOTS = NVM_FIL_SLOTS_PER_PAGE;
static const uint32_t STA_WORDS = NVM_STA_SLOT_WORDS;
static const uint32_t STA_SLOTS = NVM_STA_SLOTS_PER_PAGE;
static const uint32_t MAGIC_FIL2 = 0x324C4946u; // 'FIL2', V10.0 filament records

// The page under test followed by a second, erased page, so a helper that looks past its page (a
// full journal checked as if slot 6 existed) reads erased words there and gives itself away. For
// nvm_sta_next_slot the two pages are a two-page loaded-channel log (slots 0..63).
static uint32_t mem[2u * PAGE_WORDS];
static uint32_t *const page = mem;
static const uint32_t STA_LOG_SLOTS = 2u * STA_SLOTS;

void setUp(void)
{
    for (uint32_t i = 0; i < 2u * PAGE_WORDS; i++) mem[i] = NVM_ERASED_WORD;
}

void tearDown(void) {}

// The calls Flash_saves.cpp makes before it programs a record (slot < STA_SLOTS: the first page).
static bool fil_needs_erase(uint32_t slot)
{
    return nvm_fil_needs_erase(page, slot);
}

static bool sta_needs_erase(uint32_t slot)
{
    return nvm_sta_needs_erase(page, slot);
}

// Flash_AMS_state_write's choice of slot in the two-page log: from `slot` (the one after the
// newest record), the slot it programs and whether it erases that slot's page first.
static uint32_t sta_next(uint32_t slot, bool *erase)
{
    *erase = nvm_sta_next_slot(mem, STA_LOG_SLOTS, &slot);
    return slot;
}

// Journal records as Flash_saves.cpp packs them (nvm_records.h). A filament record in slot `slot`,
// its info filled from `tag`: the helper only asks whether words are erased, so the CRC is a
// stand-in.
static uint32_t crc_stand_in(const void *, uint32_t)
{
    return 0x5A5A5A5Au;
}

static void fil_record(uint32_t slot, uint32_t tag)
{
    uint32_t info_words[sizeof(Flash_FilamentInfo) / 4u];
    for (uint32_t w = 0u; w < sizeof(info_words) / 4u; w++) info_words[w] = tag * 0x01010101u + w + 1u;
    Flash_FilamentInfo info;
    memcpy(&info, info_words, sizeof(info));
    nvm_fil_pack(page + slot * FIL_WORDS, &info, crc_stand_in);
}

// A loaded-channel record: 0xA5 tag, sequence number, channel, then w0 ^ MAGIC_STA.
static void sta_record(uint32_t slot, uint16_t seq, uint8_t ch)
{
    nvm_sta_pack(page + slot * STA_WORDS, seq, ch);
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

// Erased reads as 0xE339E339 on CH32V20x. 0xFFFFFFFF, which the journal scans take for an empty
// slot, is a programmed word and must not be written over.
static void test_ff_word_is_not_erased(void)
{
    page[0] = 0xFFFFFFFFu;
    TEST_ASSERT_FALSE(nvm_words_erased(page, 1u));
    page[0] = 0xE339FFFFu; // one half-word programmed
    TEST_ASSERT_FALSE(nvm_words_erased(page, 1u));
    page[0] = 0xFFFFE339u;
    TEST_ASSERT_FALSE(nvm_words_erased(page, 1u));
}

// Blank page (after a full NVM clear or a page erase): fil_cache_load_one starts at slot 0, the
// loaded-channel log may be at any slot. Nothing to erase.
static void test_blank_page_is_written_without_erase(void)
{
    for (uint32_t s = 0u; s < FIL_SLOTS; s++) TEST_ASSERT_FALSE(fil_needs_erase(s));
    for (uint32_t s = 0u; s < STA_SLOTS; s++) TEST_ASSERT_FALSE(sta_needs_erase(s));
}

// Valid journal with records in slots 0..2: the scan gives slot 3, which is appended without an
// erase, so the existing records stay until the page is full. Slots already used would need one.
static void test_valid_journal_appends_to_next_free_slot(void)
{
    for (uint32_t s = 0u; s < 3u; s++) fil_record(s, s + 1u);

    TEST_ASSERT_FALSE(fil_needs_erase(3u));
    TEST_ASSERT_FALSE(fil_needs_erase(5u));
    for (uint32_t s = 0u; s < 3u; s++) TEST_ASSERT_TRUE(fil_needs_erase(s));
}

// Full journal: the scan gives FIL_SLOTS (no free slot), so the page is erased, as before.
static void test_full_journal_is_erased(void)
{
    for (uint32_t s = 0u; s < FIL_SLOTS; s++) fil_record(s, s + 1u);
    TEST_ASSERT_TRUE(fil_needs_erase(FIL_SLOTS));

    setUp();
    TEST_ASSERT_TRUE(fil_needs_erase(FIL_SLOTS)); // even with the rest of the page blank
    TEST_ASSERT_TRUE(fil_needs_erase(0xFFu));
}

// Power lost during the first record after a page erase: 4 of its 10 words were programmed. No
// valid record, so fil_cache_load_one gives slot 0; that slot must be erased, not programmed over.
static void test_torn_first_record_is_erased(void)
{
    fil_record(0u, 7u);
    for (uint32_t w = 4u; w < FIL_WORDS; w++) page[w] = NVM_ERASED_WORD;
    TEST_ASSERT_TRUE(fil_needs_erase(0u));

    // Lost after the first half-word of the magic.
    setUp();
    page[0] = 0xE3394946u;
    TEST_ASSERT_TRUE(fil_needs_erase(0u));
}

// A torn record after valid ones: the scan skips it and gives the next blank slot, which is still
// written without an erase (the valid records are kept).
static void test_torn_record_after_valid_ones_keeps_the_page(void)
{
    fil_record(0u, 1u);
    fil_record(1u, 2u);
    fil_record(2u, 3u);
    for (uint32_t w = 2u * FIL_WORDS + 5u; w < 3u * FIL_WORDS; w++) page[w] = NVM_ERASED_WORD;

    TEST_ASSERT_TRUE(fil_needs_erase(2u));
    TEST_ASSERT_FALSE(fil_needs_erase(3u));
}

// V7..V10.1 whole-page nvm256 'FIL1' record (git 4716fd2) left by a programmer that did not erase
// the NVM sector: header, 32-byte info, 0xFF padding, CRC in the last word. Its slot-0 magic
// matches MAGIC_FIL but the CRC does not, so fil_cache_load_one gives slot 0. The old code skipped
// the matching magic and programmed word 1 over 0x00200001, failing on every retry.
static void test_v7_whole_page_record_is_erased(void)
{
    page[0] = MAGIC_FIL;
    page[1] = 0x00200001u; // ver 1, len 32
    page[2] = 0x00FF0000u; // rsv: AMS 0, filament 0, not loaded
    for (uint32_t w = 3u; w < 11u; w++) page[w] = 0x41474600u + w;
    for (uint32_t w = 11u; w < PAGE_WORDS - 1u; w++) page[w] = 0xFFFFFFFFu;
    page[PAGE_WORDS - 1u] = 0x1234ABCDu;

    TEST_ASSERT_TRUE(fil_needs_erase(0u));
    // Slots 2..5 start with 0xFFFFFFFF padding, which the scan treats as blank.
    for (uint32_t s = 2u; s < FIL_SLOTS; s++) TEST_ASSERT_TRUE(fil_needs_erase(s));
}

// V10.0 'FIL2' 64-byte record in the same page (git 524bcd3): no valid FIL1 record, slot 0.
static void test_v10_fil2_record_is_erased(void)
{
    page[0] = MAGIC_FIL2;
    page[1] = 0x00200002u;
    for (uint32_t w = 2u; w < 15u; w++) page[w] = 0x10203040u + w;
    page[15] = 0xCAFEF00Du;

    TEST_ASSERT_TRUE(fil_needs_erase(0u));
    TEST_ASSERT_TRUE(fil_needs_erase(1u)); // words 10..15 hold the rest of the record
    TEST_ASSERT_FALSE(fil_needs_erase(2u));
}

// NVM sector programmed with 0xFF (a 0xFF-padded image written over it): every slot looks blank to
// the scans but none is erased.
static void test_ff_filled_page_is_erased(void)
{
    for (uint32_t i = 0u; i < PAGE_WORDS; i++) page[i] = 0xFFFFFFFFu;

    for (uint32_t s = 0u; s < FIL_SLOTS; s++) TEST_ASSERT_TRUE(fil_needs_erase(s));
    for (uint32_t s = 0u; s < STA_SLOTS; s++) TEST_ASSERT_TRUE(sta_needs_erase(s));
}

// Any single word of the target slot that is not erased counts, and only the target slot's words:
// the last word of the previous slot and the first word of the next one do not.
static void test_every_word_of_the_slot_and_only_it_is_checked(void)
{
    for (uint32_t w = 0u; w < FIL_WORDS; w++)
    {
        setUp();
        page[2u * FIL_WORDS + w] = 0xFFFFFFFFu;
        TEST_ASSERT_TRUE_MESSAGE(fil_needs_erase(2u), "word inside the slot");
    }

    setUp();
    page[2u * FIL_WORDS - 1u] = 0x00000000u;
    page[3u * FIL_WORDS] = 0x00000000u;
    TEST_ASSERT_FALSE(fil_needs_erase(2u));

    setUp();
    page[5u * STA_WORDS + 1u] = 0x00000000u;
    TEST_ASSERT_TRUE(sta_needs_erase(5u));
    TEST_ASSERT_FALSE(sta_needs_erase(4u));
    TEST_ASSERT_FALSE(sta_needs_erase(6u));
}

// Loaded-channel log that has wrapped (320 slots over 10 pages) onto a page of older records: the
// old code programmed over its slot 0, failed, and only then erased. Now it erases first.
static void test_sta_wrap_onto_older_records_is_erased(void)
{
    for (uint32_t s = 0u; s < STA_SLOTS; s++) sta_record(s, (uint16_t)(1000u + s), (uint8_t)(s & 3u));
    TEST_ASSERT_TRUE(sta_needs_erase(0u));
}

// Loaded-channel log part way through a page it erased itself: the next slot is blank.
static void test_sta_next_slot_after_own_records_is_not_erased(void)
{
    for (uint32_t s = 0u; s < 10u; s++) sta_record(s, (uint16_t)(2000u + s), 0xFFu);
    TEST_ASSERT_FALSE(sta_needs_erase(10u));
    TEST_ASSERT_FALSE(sta_needs_erase(STA_SLOTS - 1u));
}

// Torn loaded-channel record in slot 3 (power lost between its two words: w0 programmed, w1
// erased), after the newest one in slot 2: the slot itself cannot be programmed over, but the write
// skips it and programs slot 4 without an erase, so slots 0..2 stay. Torn again there: slot 5.
// Nothing moves when the slot is erased. The same for a record torn inside its first word.
static void test_sta_torn_slot_after_the_newest_is_skipped(void)
{
    bool erase = true;

    for (uint32_t s = 0u; s < 3u; s++) sta_record(s, (uint16_t)(40u + s), 1u);
    sta_record(3u, 43u, 2u);
    page[3u * STA_WORDS + 1u] = NVM_ERASED_WORD;

    TEST_ASSERT_TRUE(sta_needs_erase(3u));
    TEST_ASSERT_EQUAL_UINT32(4u, sta_next(3u, &erase));
    TEST_ASSERT_FALSE(erase);

    sta_record(4u, 43u, 2u);
    page[4u * STA_WORDS + 1u] = NVM_ERASED_WORD;
    TEST_ASSERT_EQUAL_UINT32(5u, sta_next(3u, &erase));
    TEST_ASSERT_FALSE(erase);

    TEST_ASSERT_EQUAL_UINT32(5u, sta_next(5u, &erase));
    TEST_ASSERT_FALSE(erase);

    // Only the first half-word of w0 programmed.
    setUp();
    sta_record(0u, 9u, 0u);
    page[2] = 0xE3390A00u;
    page[3] = NVM_ERASED_WORD;
    TEST_ASSERT_EQUAL_UINT32(2u, sta_next(1u, &erase));
    TEST_ASSERT_FALSE(erase);
}

// V10.0 'FIL2' page-B records sat where the loaded-channel log now starts (pages 6..9): no valid
// loaded-channel record, so the scan gives slot 0, the first slot of a page, and that page is
// erased first. The same for a torn record in slot 0 of a page, whose newest record is in the page
// before: that page holds nothing newer.
static void test_sta_foreign_or_torn_page_start_is_erased(void)
{
    bool erase = false;

    for (uint32_t r = 0u; r < 4u; r++) // four 64-byte records
    {
        page[r * 16u] = MAGIC_FIL2;
        for (uint32_t w = 1u; w < 16u; w++) page[r * 16u + w] = 0x00010000u * r + w;
    }
    TEST_ASSERT_TRUE(sta_needs_erase(0u));
    TEST_ASSERT_TRUE(sta_needs_erase(STA_SLOTS - 1u));
    TEST_ASSERT_EQUAL_UINT32(0u, sta_next(0u, &erase));
    TEST_ASSERT_TRUE(erase);

    setUp();
    for (uint32_t s = 0u; s < STA_SLOTS; s++) sta_record(s, (uint16_t)(100u + s), 3u);
    sta_record(STA_SLOTS, 132u, 0u);
    page[STA_SLOTS * STA_WORDS + 1u] = NVM_ERASED_WORD;
    TEST_ASSERT_EQUAL_UINT32(STA_SLOTS, sta_next(STA_SLOTS, &erase));
    TEST_ASSERT_TRUE(erase);
}

// Torn slots up to the end of the page with the newest record: the write moves on to slot 0 of the
// next page, which it erases only if that slot is not erased (older records from the last pass).
// At the end of the log it wraps to slot 0.
static void test_sta_torn_slots_to_the_end_of_the_page_move_to_the_next(void)
{
    bool erase = true;

    for (uint32_t s = 0u; s < 30u; s++) sta_record(s, (uint16_t)(500u + s), 1u);
    for (uint32_t s = 30u; s < STA_SLOTS; s++)
    {
        sta_record(s, 530u, 2u);
        page[s * STA_WORDS + 1u] = NVM_ERASED_WORD;
    }
    TEST_ASSERT_EQUAL_UINT32(STA_SLOTS, sta_next(30u, &erase));
    TEST_ASSERT_FALSE(erase);

    for (uint32_t s = STA_SLOTS; s < STA_LOG_SLOTS; s++) sta_record(s, (uint16_t)(400u + s), 0u);
    TEST_ASSERT_EQUAL_UINT32(STA_SLOTS, sta_next(30u, &erase));
    TEST_ASSERT_TRUE(erase);

    // The wrap: the newest record in slot 62, a torn one in 63, the log's last slot.
    setUp();
    for (uint32_t s = 0u; s < STA_LOG_SLOTS - 1u; s++) sta_record(s, (uint16_t)(700u + s), 1u);
    sta_record(STA_LOG_SLOTS - 1u, 763u, 2u);
    page[(STA_LOG_SLOTS - 1u) * STA_WORDS + 1u] = NVM_ERASED_WORD;
    TEST_ASSERT_EQUAL_UINT32(0u, sta_next(STA_LOG_SLOTS - 1u, &erase));
    TEST_ASSERT_TRUE(erase);

    // Slot 0 erased (power lost after that erase, before the record): no erase.
    for (uint32_t w = 0u; w < STA_WORDS; w++) page[w] = NVM_ERASED_WORD;
    TEST_ASSERT_EQUAL_UINT32(0u, sta_next(STA_LOG_SLOTS - 1u, &erase));
    TEST_ASSERT_FALSE(erase);
}

// The wrappers Flash_saves.cpp calls: the filament one takes first_empty, the loaded-channel one
// the slot index across all log pages, so slot 32 is slot 0 of its page and 319 is slot 31.
static void test_geometry_wrappers(void)
{
    TEST_ASSERT_FALSE(nvm_fil_needs_erase(page, 0u));
    TEST_ASSERT_FALSE(nvm_fil_needs_erase(page, 5u));
    TEST_ASSERT_TRUE(nvm_fil_needs_erase(page, 6u));

    TEST_ASSERT_FALSE(nvm_sta_needs_erase(page, 0u));
    TEST_ASSERT_FALSE(nvm_sta_needs_erase(page, 31u));
    TEST_ASSERT_FALSE(nvm_sta_needs_erase(page, 32u));
    TEST_ASSERT_FALSE(nvm_sta_needs_erase(page, 319u));

    sta_record(31u, 7u, 0u);
    TEST_ASSERT_TRUE(nvm_sta_needs_erase(page, 31u));
    TEST_ASSERT_TRUE(nvm_sta_needs_erase(page, 319u));
    TEST_ASSERT_FALSE(nvm_sta_needs_erase(page, 32u));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_words_erased_checks_exactly_count_words);
    RUN_TEST(test_ff_word_is_not_erased);
    RUN_TEST(test_blank_page_is_written_without_erase);
    RUN_TEST(test_valid_journal_appends_to_next_free_slot);
    RUN_TEST(test_full_journal_is_erased);
    RUN_TEST(test_torn_first_record_is_erased);
    RUN_TEST(test_torn_record_after_valid_ones_keeps_the_page);
    RUN_TEST(test_v7_whole_page_record_is_erased);
    RUN_TEST(test_v10_fil2_record_is_erased);
    RUN_TEST(test_ff_filled_page_is_erased);
    RUN_TEST(test_every_word_of_the_slot_and_only_it_is_checked);
    RUN_TEST(test_sta_wrap_onto_older_records_is_erased);
    RUN_TEST(test_sta_next_slot_after_own_records_is_not_erased);
    RUN_TEST(test_sta_torn_slot_after_the_newest_is_skipped);
    RUN_TEST(test_sta_foreign_or_torn_page_start_is_erased);
    RUN_TEST(test_sta_torn_slots_to_the_end_of_the_page_move_to_the_next);
    RUN_TEST(test_geometry_wrappers);
    return UNITY_END();
}
