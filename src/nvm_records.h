#pragma once
// The records of the filament-info and loaded-channel journals (Flash_saves.cpp): how one is packed,
// when one is valid, and what a scan of the journal finds at boot; and the buffer calibration
// record's rsv word and two-slot choice. Hardware-free, so what the
// printer sees after a reset is tested on the host (test/test_nvm_records). Flash_saves.cpp keeps
// the flash addressing, erase and program, and passes its CRC32 (the CH32 CRC unit) as crc.
#include <stdint.h>
#include <string.h>

#include "Flash_saves.h"
#include "nvm_journal.h"

// CRC32 of `bytes` bytes (a multiple of 4) at data, fed as 32-bit words.
typedef uint32_t (*nvm_crc32_fn)(const void *data, uint32_t bytes);

// A word that reads as never written (V11): erased, or the 0xFF padding of older layouts. Used to
// tell "no record" from a record (nvm256_read's CRC word, nvm_sta_valid) and for erase verification.
// Only the first is really erased: whether a slot is free to program is the strict
// nvm_words_erased() (nvm_journal.h).
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
// Slots that are neither erased nor valid (a torn record, another layout's data) are skipped.
// *first_empty: the first erased slot after the last slot that is not erased, NVM_FIL_SLOTS_PER_PAGE
// if none (the next write then rolls the page over through the scratch page). So every slot from
// *first_empty on is erased (V11's cursor rule, with the strict erased test of nvm_journal.h: a slot
// of 0xFFFFFFFF words is programmed, not free).
static inline bool nvm_fil_scan_page(const uint32_t *page, Flash_FilamentInfo *last, uint32_t *first_empty,
                                     nvm_crc32_fn crc)
{
    bool found = false;

    if (first_empty) *first_empty = NVM_FIL_SLOTS_PER_PAGE;

    for (uint32_t s = 0u; s < NVM_FIL_SLOTS_PER_PAGE; s++)
    {
        const uint32_t *p = page + s * NVM_FIL_SLOT_WORDS;

        if (nvm_words_erased(p, NVM_FIL_SLOT_WORDS))
        {
            if (first_empty && *first_empty == NVM_FIL_SLOTS_PER_PAGE)
                *first_empty = s;
            continue;
        }

        if (first_empty) *first_empty = NVM_FIL_SLOTS_PER_PAGE; // V11: the cursor goes after the last used slot
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
// into, the scan's cursor in every case (V11 fil_cache_load_one): a page whose only record is torn
// gives the slot after it, never the torn slot, and a page with no erased slot left gives
// NVM_FIL_SLOTS_PER_PAGE, so the next write rolls it over (erase) instead of programming over it.
static inline bool nvm_fil_load(const uint32_t *page, Flash_FilamentInfo *last, uint32_t *next_slot,
                                nvm_crc32_fn crc)
{
    uint32_t first_empty = NVM_FIL_SLOTS_PER_PAGE;

    const bool found = nvm_fil_scan_page(page, last, &first_empty, crc);
    *next_slot = first_empty;
    if (!found) memset(last, 0, sizeof(*last));
    return found;
}

// ---- Loaded channel: w0 = 0xA6 tag, 16-bit sequence number, channel; w1 = w0 ^ MAGIC_STA ----
// 0xA6 is V11's tag; the 0xA5 records of V10.x and the fork are ignored. Flash_saves.cpp programs w1
// first and w0 last (upper halfword, with the tag, at the very end), so a torn record never passes
// nvm_sta_valid.

#define NVM_STA_TAG 0xA6u

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

    const uint8_t c = (uint8_t)(w0 & 0xFFu);
    if (c >= 4u && c != 0xFFu) return false; // V11: only channels 0..3 or 0xFF (none loaded)
    *seq = (uint16_t)((w0 >> 8) & 0xFFFFu);
    *ch = c;
    return true;
}

// Sequence a written after b, across the 2^16 wrap (the log holds far fewer than 2^15 records).
static inline bool nvm_sta_seq_newer(uint16_t a, uint16_t b)
{
    return (int16_t)(a - b) > 0;
}

// At boot: the newest record of the log (`slots` records of NVM_STA_SLOT_WORDS words at log, over
// its pages): its channel, and the sequence number and slot of the next record (the slot after it,
// wrapping to 0). Invalid slots (torn, blank, another layout's data, a channel that is neither 0..3
// nor 0xFF) are skipped; if two records are not ordered (equal, or 2^15 apart), the lower slot wins.
// None: channel 0xFF, sequence 0, slot 0. Same result as V11's scan in Flash_AMS_state_read.
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

// ---- Buffer calibration (Flash_MC_PULL_cal_*, V11's CAL v2): the 256-B record's rsv word ----
// Polarity in bits 0..3 (bit ch set: -1), valid_mask in bits 8..11, the record's sequence number in
// bits 16..31. Two pages alternate (slot 0: FLASH_NVM_CAL_ADDR, slot 1: its backup): a read takes the
// newer by sequence number across the 2^16 wrap (slot 0 when not ordered), and a write goes to the
// other slot with the sequence after it (slot 0 and sequence 0 when neither holds a record).

static inline uint32_t nvm_cal_rsv_flags(const int8_t pol[4], uint8_t valid_mask)
{
    uint32_t rsv = (uint32_t)(valid_mask & 0x0Fu) << 8;
    for (uint8_t ch = 0u; ch < 4u; ch++)
    {
        if (pol && pol[ch] < 0) rsv |= (1u << ch);
    }
    return rsv;
}

static inline uint32_t nvm_cal_rsv(uint16_t seq, uint32_t flags)
{
    return ((uint32_t)seq << 16) | flags;
}

static inline uint8_t nvm_cal_rsv_valid(uint32_t rsv)
{
    return (uint8_t)((rsv >> 8) & 0x0Fu);
}

static inline int8_t nvm_cal_rsv_pol(uint32_t rsv, uint8_t ch)
{
    return (rsv & (1u << ch)) ? -1 : 1;
}

static inline uint16_t nvm_cal_rsv_seq(uint32_t rsv)
{
    return (uint16_t)(rsv >> 16);
}

// The two-slot scan: take this record (rsv `candidate`) over the newest so far (rsv `best`, if have)?
static inline bool nvm_cal_take(bool have, uint32_t candidate, uint32_t best)
{
    return !have || (int16_t)(nvm_cal_rsv_seq(candidate) - nvm_cal_rsv_seq(best)) > 0;
}

// The slot a write goes to, and its sequence number, after a scan that found the newest record in
// slot latest_slot (rsv latest_rsv) if have.
static inline uint8_t nvm_cal_write_slot(bool have, uint8_t latest_slot)
{
    return (have && latest_slot == 0u) ? 1u : 0u;
}

static inline uint16_t nvm_cal_next_seq(bool have, uint32_t latest_rsv)
{
    return have ? (uint16_t)(nvm_cal_rsv_seq(latest_rsv) + 1u) : 0u;
}
