#pragma once
// Where the filament-info and loaded-channel journals (Flash_saves.cpp) may program their next
// record. Hardware-free, so the decision is tested on the host (test/test_nvm_journal).
//
// A flash word may only be programmed while it is erased, and CH32V20x flash reads an erased word
// as 0xE339E339. A word that reads 0xFFFFFFFF has been programmed (the 0xFF padding older firmware
// wrote into these pages, or a 0xFF-padded image written over them), even though the journal scans
// treat it as an empty slot. The SDK reports no error for programming a word that is not erased:
// only the read-back fails, and a retry of the same slot then fails the same way every time.
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

// Before a record goes into slot `slot` of a journal page (`slots` slots of `slot_words` words at
// `page`): true if the page must be erased first, and the record then goes into slot 0. That is the
// case when the page is full, and also when any word of the slot is not erased: a torn record from
// a power loss, or data from an older NVM layout.
static inline bool nvm_journal_needs_erase(const uint32_t *page, uint32_t slot, uint32_t slot_words,
                                           uint32_t slots)
{
    return slot >= slots || !nvm_words_erased(page + slot * slot_words, slot_words);
}

// Journal geometry of Flash_saves.cpp (256-byte pages), checked there with static_assert.
#define NVM_FIL_SLOT_WORDS 10u      // 9 data words + CRC32
#define NVM_FIL_SLOTS_PER_PAGE 6u
#define NVM_STA_SLOT_WORDS 2u
#define NVM_STA_SLOTS_PER_PAGE 32u  // 256 / 8

// Filament-info journal: the page of one filament, next record at slot first_empty (6 = full).
static inline bool nvm_fil_needs_erase(const uint32_t *page, uint32_t first_empty)
{
    return nvm_journal_needs_erase(page, first_empty, NVM_FIL_SLOT_WORDS, NVM_FIL_SLOTS_PER_PAGE);
}

// Loaded-channel log: global_slot counts across all log pages; page is the one that holds it.
static inline bool nvm_sta_needs_erase(const uint32_t *page, uint32_t global_slot)
{
    return nvm_journal_needs_erase(page, global_slot % NVM_STA_SLOTS_PER_PAGE, NVM_STA_SLOT_WORDS,
                                   NVM_STA_SLOTS_PER_PAGE);
}
