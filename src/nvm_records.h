#pragma once
// The records of the filament-info and loaded-channel journals (Flash_saves.cpp): how one is packed,
// when one is valid, and what a scan of the journal finds at boot. Hardware-free, so what the
// printer sees after a reset is tested on the host (test/test_nvm_records). Flash_saves.cpp keeps
// the flash addressing, erase and program, and passes its CRC32 (the CH32 CRC unit) as crc.
#include <stdint.h>
#include <string.h>

#include "Flash_saves.h"
#include "nvm_journal.h"

// CRC32 of `bytes` bytes (a multiple of 4) at data, fed as 32-bit words.
typedef uint32_t (*nvm_crc32_fn)(const void *data, uint32_t bytes);

// A word the journal scans take for an empty slot: erased, or the 0xFF padding of older layouts
// (nvm_journal.h: only the first is really erased).
static inline bool nvm_word_is_blank(uint32_t v)
{
    return v == NVM_ERASED_WORD || v == 0xFFFFFFFFu;
}

// ---- Filament info: MAGIC_FIL, the 32-byte Flash_FilamentInfo, CRC32 of the 9 words before it ----

static inline void nvm_fil_pack(uint32_t w[NVM_FIL_SLOT_WORDS], const Flash_FilamentInfo *info,
                                nvm_crc32_fn crc)
{
    w[0] = MAGIC_FIL;
    memcpy(&w[1], info, sizeof(*info));
    w[NVM_FIL_SLOT_WORDS - 1u] = crc(w, (NVM_FIL_SLOT_WORDS - 1u) * 4u);
}

static inline bool nvm_fil_valid(const uint32_t *p, Flash_FilamentInfo *out, nvm_crc32_fn crc)
{
    if (p[0] != MAGIC_FIL) return false;
    if (crc(p, (NVM_FIL_SLOT_WORDS - 1u) * 4u) != p[NVM_FIL_SLOT_WORDS - 1u]) return false;
    if (out) memcpy(out, &p[1], sizeof(*out));
    return true;
}

// One filament page: true if it holds a valid record, and *last is the one in the highest slot.
// *first_empty: the lowest slot whose first word is blank, NVM_FIL_SLOTS_PER_PAGE if none. Slots
// that are neither blank nor valid (a torn record, another layout's data) are skipped.
static inline bool nvm_fil_scan_page(const uint32_t *page, Flash_FilamentInfo *last, uint32_t *first_empty,
                                     nvm_crc32_fn crc)
{
    bool found = false;

    if (first_empty) *first_empty = NVM_FIL_SLOTS_PER_PAGE;

    for (uint32_t s = 0u; s < NVM_FIL_SLOTS_PER_PAGE; s++)
    {
        const uint32_t *p = page + s * NVM_FIL_SLOT_WORDS;

        if (nvm_word_is_blank(p[0]))
        {
            if (first_empty && *first_empty == NVM_FIL_SLOTS_PER_PAGE)
                *first_empty = s;
            continue;
        }

        Flash_FilamentInfo tmp;
        if (nvm_fil_valid(p, &tmp, crc))
        {
            if (last) *last = tmp;
            found = true;
        }
    }

    return found;
}

// At boot: the saved info of one filament page (zeroed if none) and the slot the next record goes
// into. A page with no valid record gives slot 0, so its first write erases it if it is not erased.
static inline bool nvm_fil_load(const uint32_t *page, Flash_FilamentInfo *last, uint32_t *next_slot,
                                nvm_crc32_fn crc)
{
    uint32_t first_empty = NVM_FIL_SLOTS_PER_PAGE;

    if (nvm_fil_scan_page(page, last, &first_empty, crc))
    {
        *next_slot = first_empty;
        return true;
    }

    *next_slot = 0u;
    memset(last, 0, sizeof(*last));
    return false;
}

// ---- Loaded channel: w0 = 0xA5 tag, 16-bit sequence number, channel; w1 = w0 ^ MAGIC_STA ----

#define NVM_STA_TAG 0xA5u

static inline void nvm_sta_pack(uint32_t w[NVM_STA_SLOT_WORDS], uint16_t seq, uint8_t ch)
{
    w[0] = ((uint32_t)NVM_STA_TAG << 24) | ((uint32_t)seq << 8) | (uint32_t)ch;
    w[1] = w[0] ^ MAGIC_STA;
}

static inline bool nvm_sta_valid(const uint32_t *p, uint16_t *seq, uint8_t *ch)
{
    const uint32_t w0 = p[0];
    const uint32_t w1 = p[1];

    if (nvm_word_is_blank(w0) && nvm_word_is_blank(w1)) return false;
    if ((w0 >> 24) != NVM_STA_TAG) return false;
    if ((w0 ^ w1) != MAGIC_STA) return false;

    *seq = (uint16_t)((w0 >> 8) & 0xFFFFu);
    *ch = (uint8_t)(w0 & 0xFFu);
    return true;
}

// Sequence a written after b, across the 2^16 wrap (the log holds far fewer than 2^15 records).
static inline bool nvm_sta_seq_newer(uint16_t a, uint16_t b)
{
    return (int16_t)(a - b) > 0;
}

// At boot: the newest record of the log (`slots` records of NVM_STA_SLOT_WORDS words at log, over
// its pages): its channel, and the sequence number and slot of the next record (the slot after it,
// wrapping to 0). Invalid slots (torn, blank, another layout's data) are skipped; if two records are
// not ordered (equal, or 2^15 apart), the lower slot wins. None: channel 0xFF, sequence 0, slot 0.
static inline bool nvm_sta_scan(const uint32_t *log, uint32_t slots, uint8_t *ch, uint16_t *next_seq,
                                uint32_t *next_slot)
{
    uint8_t best_ch = 0xFFu;
    uint16_t best_seq = 0u;
    uint32_t best_slot = 0u;
    bool have = false;

    for (uint32_t slot = 0u; slot < slots; slot++)
    {
        uint16_t seq;
        uint8_t c;
        if (!nvm_sta_valid(log + slot * NVM_STA_SLOT_WORDS, &seq, &c)) continue;

        if (!have || nvm_sta_seq_newer(seq, best_seq))
        {
            have = true;
            best_seq = seq;
            best_ch = c;
            best_slot = slot;
        }
    }

    *ch = best_ch;
    *next_seq = have ? (uint16_t)(best_seq + 1u) : 0u;
    *next_slot = have ? (best_slot + 1u) % slots : 0u;
    return have;
}
