#pragma once
// Where the loaded-channel log (Flash_saves.cpp) may program its next record, and the strict test
// for a slot that is free to program. Hardware-free, so the decision is tested on the host
// (test/test_nvm_journal).
//
// A flash word may only be programmed while it is erased, and CH32V20x flash reads an erased word
// as 0xE339E339. A word that reads 0xFFFFFFFF has been programmed (the 0xFF padding older firmware
// wrote into these pages, a 0xFF-padded image written over them, or a non-erasing programmer). The
// SDK reports no error for programming a word that is not erased: only the read-back fails, and V11's
// program path then sets its sticky flash fault, which stops every NVM write (CAL included) until
// reboot, and again at every boot that picks the same slot. So the two places that choose a slot to
// program (nvm_fil_scan_page in nvm_records.h, nvm_sta_find_slot here) take only exactly erased
// words as free. Erase verification keeps V11's lenient test (nvm_word_is_blank, nvm_records.h).
#include <stdint.h>

#define NVM_ERASED_WORD 0xE339E339u

// True if all `count` words at `p` are erased.
static inline bool nvm_words_erased(const uint32_t *p, uint32_t count)
{
    for (uint32_t i = 0u; i < count; i++)
    {
        if (p[i] != NVM_ERASED_WORD) return false;
    }
    return true;
}

// Journal geometry of Flash_saves.cpp (256-byte pages), checked there with static_assert.
#define NVM_FIL_SLOT_WORDS 10u      // 9 data words + CRC32
#define NVM_FIL_SLOTS_PER_PAGE 6u
#define NVM_STA_SLOT_WORDS 2u
#define NVM_STA_SLOTS_PER_PAGE 32u  // 256 / 8
#define NVM_STA_LOG_PAGES 8u        // pages 6..13; 14 is the filament scratch page, 15 the CAL backup (V11)

// Before a record goes into the loaded-channel log (`slots` slots, whole pages, at log): the slot
// after the newest record (start) if it is erased, else the next erased slot going forward round
// the whole log (torn records and older data are skipped, never programmed over). None left: *erase,
// and the record goes into slot 0 of start's page, or of the page after it if that page holds the
// newest record (saved_slot), so the newest record is never erased. Only exactly erased words count
// as free: 0xFFFFFFFF is programmed on CH32V20x and would fail the read-back.
static inline uint32_t nvm_sta_find_slot(const uint32_t *log, uint32_t slots, uint32_t start, bool have_saved,
                                         uint32_t saved_slot, bool *erase)
{
    uint32_t slot = start;
    for (uint32_t scanned = 0u; scanned < slots; scanned++)
    {
        if (nvm_words_erased(log + slot * NVM_STA_SLOT_WORDS, NVM_STA_SLOT_WORDS))
        {
            *erase = false;
            return slot;
        }
        slot = (slot + 1u) % slots;
    }
    *erase = true;
    const uint32_t pages = slots / NVM_STA_SLOTS_PER_PAGE;
    uint32_t page = start / NVM_STA_SLOTS_PER_PAGE;
    if (have_saved && page == saved_slot / NVM_STA_SLOTS_PER_PAGE) page = (page + 1u) % pages;
    return page * NVM_STA_SLOTS_PER_PAGE;
}
