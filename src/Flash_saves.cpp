#include "Flash_saves.h"
#include "mc_pull_cal_range.h"
#include "hal/irq_wch.h"
#include "nvm_journal.h"
#include <string.h>

#include "ch32v20x_rcc.h"
#include "ch32v20x_crc.h"
#include "ch32v20x_flash.h"

static_assert(sizeof(Flash_FilamentInfo) == 32u, "Flash_FilamentInfo must be 32 bytes");
static_assert(alignof(Flash_FilamentInfo) >= 4u, "Flash_FilamentInfo must be 4-byte aligned");

static uint32_t crc32_hw_words(const void* data, uint32_t bytes)
{
    const uint32_t* p = (const uint32_t*)data;
    CRC->CTLR = 1u;
    for (uint32_t i = 0u; i < (bytes >> 2); i++) CRC->DATAR = p[i];
    return CRC->DATAR;
}

static inline uint32_t ams_fil_page(uint8_t filament_idx)
{
    return FLASH_NVM_AMS_ADDR + (uint32_t)filament_idx * FLASH_NVM256_PAGE_SIZE;
}

static inline bool flash_word_is_blank(uint32_t v)
{
    return v == NVM_ERASED_WORD || v == 0xFFFFFFFFu;
}

static bool flash_range_is_erased(uint32_t base_addr, uint32_t bytes)
{
    return nvm_words_erased((const uint32_t*)base_addr, bytes >> 2);
}

static bool flash256_prog(uint32_t page_addr, const uint32_t w[64])
{
    if (page_addr & (FLASH_NVM256_PAGE_SIZE - 1u)) return false;

    if (memcmp((const void*)page_addr, (const void*)w, FLASH_NVM256_PAGE_SIZE) == 0)
        return true;

    const uint32_t irq = irq_save_wch();
    FLASH_Unlock_Fast();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage_Fast(page_addr);
    FLASH_ProgramPage_Fast(page_addr, (uint32_t*)w);
    FLASH_Lock_Fast();
    FLASH_Lock();
    irq_restore_wch(irq);

    return (memcmp((const void*)page_addr, (const void*)w, FLASH_NVM256_PAGE_SIZE) == 0);
}

static bool flash256_erase(uint32_t page_addr)
{
    if (page_addr & (FLASH_NVM256_PAGE_SIZE - 1u)) return false;

    const uint32_t irq = irq_save_wch();
    FLASH_Unlock_Fast();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage_Fast(page_addr);
    FLASH_Lock_Fast();
    FLASH_Lock();
    irq_restore_wch(irq);

    return flash_range_is_erased(page_addr, FLASH_NVM256_PAGE_SIZE);
}

static bool flash_word_prog_std(uint32_t addr, uint32_t data)
{
    if (addr & 3u) return false;

    const uint32_t cur = *(const volatile uint32_t*)addr;
    if (cur == data) return true;

    const uint32_t irq = irq_save_wch();
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
    const FLASH_Status st = FLASH_ProgramWord(addr, data);
    FLASH_Lock();
    irq_restore_wch(irq);

    return (st == FLASH_COMPLETE) && (*(const volatile uint32_t*)addr == data);
}

static bool flash_prog_words(uint32_t base_addr, const uint32_t* words, uint32_t count)
{
    if (base_addr & 3u) return false;

    for (uint32_t i = 0u; i < count; i++)
    {
        if (!flash_word_prog_std(base_addr + (i << 2), words[i]))
            return false;
    }

    return true;
}

static bool nvm256_write(uint32_t page_addr, uint32_t magic, uint16_t ver, uint32_t rsv,
                         const void* payload, uint16_t len)
{
    if (len > (uint16_t)(NVM256_CRC_OFF - sizeof(NVM256_HDR))) return false;

    alignas(4) uint32_t w[64];
    uint8_t* b = (uint8_t*)w;
    memset(b, 0xFF, FLASH_NVM256_PAGE_SIZE);

    NVM256_HDR h{};
    h.magic = magic;
    h.ver = ver;
    h.len = len;
    h.rsv = rsv;

    memcpy(b, &h, sizeof(h));
    if (len) memcpy(b + sizeof(h), payload, len);

    const uint32_t crc = crc32_hw_words(b, NVM256_CRC_OFF);
    memcpy(b + NVM256_CRC_OFF, &crc, 4u);

    return flash256_prog(page_addr, w);
}

static bool nvm256_read(uint32_t page_addr, uint32_t magic, uint16_t ver,
                        void* out, uint16_t max_len, uint16_t* got_len, uint32_t* rsv_out)
{
    const uint8_t* b = (const uint8_t*)page_addr;

    const uint32_t stored = *(const uint32_t*)(b + NVM256_CRC_OFF);
    if (flash_word_is_blank(stored)) return false;

    const uint32_t crc = crc32_hw_words(b, NVM256_CRC_OFF);
    if (crc != stored) return false;

    NVM256_HDR h{};
    memcpy(&h, b, sizeof(h));

    if (h.magic != magic) return false;
    if (h.ver != ver) return false;
    if (h.len > max_len) return false;

    if (h.len) memcpy(out, b + sizeof(h), h.len);
    if (got_len) *got_len = h.len;
    if (rsv_out) *rsv_out = h.rsv;
    return true;
}

static constexpr uint32_t FIL_SLOT_WORDS = 10u;
static constexpr uint32_t FIL_SLOT_BYTES = FIL_SLOT_WORDS * 4u;
static constexpr uint32_t FIL_SLOTS_PER_PAGE = 6u;

static_assert(FIL_SLOT_BYTES * FIL_SLOTS_PER_PAGE <= FLASH_NVM256_PAGE_SIZE, "FIL journal too large");
static_assert(FIL_SLOT_WORDS == NVM_FIL_SLOT_WORDS && FIL_SLOTS_PER_PAGE == NVM_FIL_SLOTS_PER_PAGE,
              "nvm_journal.h filament geometry out of date");

static inline uint32_t fil_page_addr(uint8_t filament_idx)
{
    return ams_fil_page(filament_idx);
}

static inline void fil_slot_pack(uint32_t w[FIL_SLOT_WORDS], const Flash_FilamentInfo* info)
{
    w[0] = MAGIC_FIL;
    memcpy(&w[1], info, sizeof(*info));
    w[FIL_SLOT_WORDS - 1u] = crc32_hw_words(w, (FIL_SLOT_WORDS - 1u) * 4u);
}

static inline bool fil_slot_valid(const uint32_t* p, Flash_FilamentInfo* out)
{
    if (p[0] != MAGIC_FIL) return false;
    if (crc32_hw_words(p, (FIL_SLOT_WORDS - 1u) * 4u) != p[FIL_SLOT_WORDS - 1u]) return false;
    if (out) memcpy(out, &p[1], sizeof(*out));
    return true;
}

static bool fil_scan_page(uint32_t base, Flash_FilamentInfo* last, uint32_t* first_empty)
{
    bool found = false;

    if (first_empty) *first_empty = FIL_SLOTS_PER_PAGE;

    for (uint32_t s = 0u; s < FIL_SLOTS_PER_PAGE; s++)
    {
        const uint32_t* p = (const uint32_t*)(base + s * FIL_SLOT_BYTES);

        if (flash_word_is_blank(p[0]))
        {
            if (first_empty && *first_empty == FIL_SLOTS_PER_PAGE)
                *first_empty = s;
            continue;
        }

        Flash_FilamentInfo tmp;
        if (fil_slot_valid(p, &tmp))
        {
            if (last) *last = tmp;
            found = true;
        }
    }

    return found;
}

static uint8_t g_fil_have[4] = {0u, 0u, 0u, 0u};
static uint8_t g_fil_first_empty[4] = {0u, 0u, 0u, 0u};
static Flash_FilamentInfo g_fil_last[4];

static void fil_cache_load_one(uint8_t filament_idx)
{
    Flash_FilamentInfo last;
    uint32_t first_empty = FIL_SLOTS_PER_PAGE;
    const uint32_t base = fil_page_addr(filament_idx);

    if (fil_scan_page(base, &last, &first_empty))
    {
        g_fil_have[filament_idx] = 1u;
        g_fil_first_empty[filament_idx] = (uint8_t)first_empty;
        memcpy(&g_fil_last[filament_idx], &last, sizeof(last));
        return;
    }

    g_fil_have[filament_idx] = 0u;
    g_fil_first_empty[filament_idx] = 0u;
    memset(&g_fil_last[filament_idx], 0, sizeof(g_fil_last[filament_idx]));
}

static constexpr uint32_t STA_TAG = 0xA5u;
static constexpr uint32_t STA_PAGE_FIRST = 6u;
static constexpr uint32_t STA_PAGE_COUNT = 10u;
static constexpr uint32_t STA_SLOT_BYTES = 8u;
static constexpr uint32_t STA_SLOTS_PER_PAGE = (FLASH_NVM256_PAGE_SIZE / STA_SLOT_BYTES);
static constexpr uint32_t STA_TOTAL_SLOTS = (STA_PAGE_COUNT * STA_SLOTS_PER_PAGE);
static_assert(STA_SLOT_BYTES == NVM_STA_SLOT_WORDS * 4u && STA_SLOTS_PER_PAGE == NVM_STA_SLOTS_PER_PAGE,
              "nvm_journal.h loaded-channel geometry out of date");

static uint16_t g_sta_seq = 0u;
static uint16_t g_sta_slot = 0u;
static uint8_t g_sta_have_saved = 0u;
static uint8_t g_sta_saved_loaded = 0xFFu;

static void flash_runtime_cache_clear(void)
{
    memset(g_fil_have, 0, sizeof(g_fil_have));
    memset(g_fil_first_empty, 0, sizeof(g_fil_first_empty));
    memset(g_fil_last, 0, sizeof(g_fil_last));

    g_sta_seq = 0u;
    g_sta_slot = 0u;
    g_sta_have_saved = 0u;
    g_sta_saved_loaded = 0xFFu;
}

bool Flash_NVM_full_clear(void)
{
    const uint32_t irq = irq_save_wch();

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
    const FLASH_Status st = FLASH_ErasePage(FLASH_NVM_BASE_ADDR);
    FLASH_Lock();
    irq_restore_wch(irq);

    if (st != FLASH_COMPLETE) return false;
    if (!flash_range_is_erased(FLASH_NVM_BASE_ADDR, FLASH_NVM_TOTAL_SIZE)) return false;

    flash_runtime_cache_clear();
    return true;
}

static inline uint32_t sta_page_addr(uint32_t page_i)
{
    return FLASH_NVM_BASE_ADDR + (STA_PAGE_FIRST + page_i) * FLASH_NVM256_PAGE_SIZE;
}

static inline uint32_t sta_slot_addr(uint32_t slot)
{
    const uint32_t page_i = slot / STA_SLOTS_PER_PAGE;
    const uint32_t slot_i = slot - page_i * STA_SLOTS_PER_PAGE;
    return sta_page_addr(page_i) + slot_i * STA_SLOT_BYTES;
}

void Flash_saves_init(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

    flash_runtime_cache_clear();

    for (uint8_t i = 0u; i < 4u; i++)
        fil_cache_load_one(i);
}

bool Flash_AMS_filament_write(uint8_t filament_idx, const Flash_FilamentInfo* info)
{
    if (!info || filament_idx >= 4u) return false;

    if (g_fil_have[filament_idx] &&
        memcmp(&g_fil_last[filament_idx], info, sizeof(*info)) == 0)
        return true;

    uint32_t first_empty = g_fil_first_empty[filament_idx];
    const uint32_t base = fil_page_addr(filament_idx);

    // Erase first when the page is full, and also when the slot is not erased (a torn record,
    // another layout's data, an earlier failed write here): programming over it fails its
    // read-back on every retry. A page with no valid record gives slot 0 (fil_cache_load_one).
    if (nvm_fil_needs_erase((const uint32_t*)base, first_empty))
    {
        if (!flash256_erase(base)) return false;
        // The page no longer holds g_fil_last: if the program below fails, the retry must start
        // again at slot 0 of this page instead of the stale slot, and reverting to the old info
        // must not look already saved.
        g_fil_have[filament_idx] = 0u;
        g_fil_first_empty[filament_idx] = 0u;
        first_empty = 0u;
    }

    alignas(4) uint32_t w[FIL_SLOT_WORDS];
    fil_slot_pack(w, info);

    if (!flash_prog_words(base + first_empty * FIL_SLOT_BYTES, w, FIL_SLOT_WORDS))
        return false;

    memcpy(&g_fil_last[filament_idx], info, sizeof(*info));
    g_fil_have[filament_idx] = 1u;
    g_fil_first_empty[filament_idx] =
        (uint8_t)(((first_empty + 1u) < FIL_SLOTS_PER_PAGE) ? (first_empty + 1u) : FIL_SLOTS_PER_PAGE);

    return true;
}

bool Flash_AMS_filament_read(uint8_t filament_idx, Flash_FilamentInfo* out)
{
    if (!out || filament_idx >= 4u) return false;
    if (!g_fil_have[filament_idx]) return false;

    memcpy(out, &g_fil_last[filament_idx], sizeof(*out));
    return true;
}

bool Flash_AMS_filament_clear(uint8_t filament_idx)
{
    if (filament_idx >= 4u) return false;
    if (!flash256_erase(fil_page_addr(filament_idx))) return false;

    g_fil_have[filament_idx] = 0u;
    g_fil_first_empty[filament_idx] = 0u;
    memset(&g_fil_last[filament_idx], 0, sizeof(g_fil_last[filament_idx]));
    return true;
}

bool Flash_AMS_state_read(uint8_t* loaded_ch)
{
    if (!loaded_ch) return false;

    uint8_t best_ch = 0xFFu;
    uint16_t best_seq = 0u;
    uint32_t best_slot = 0u;
    uint8_t have = 0u;

    for (uint32_t slot = 0u; slot < STA_TOTAL_SLOTS; slot++)
    {
        const uint32_t a = sta_slot_addr(slot);
        const uint32_t w0 = *(const volatile uint32_t*)(a + 0u);
        const uint32_t w1 = *(const volatile uint32_t*)(a + 4u);

        if (flash_word_is_blank(w0) && flash_word_is_blank(w1)) continue;
        if ((w0 >> 24) != STA_TAG) continue;
        if ((w0 ^ w1) != MAGIC_STA) continue;

        const uint16_t seq = (uint16_t)((w0 >> 8) & 0xFFFFu);
        const uint8_t ch = (uint8_t)(w0 & 0xFFu);

        if (!have || (int16_t)(seq - best_seq) > 0)
        {
            have = 1u;
            best_seq = seq;
            best_ch = ch;
            best_slot = slot;
        }
    }

    if (have)
    {
        g_sta_seq = (uint16_t)(best_seq + 1u);
        g_sta_slot = (uint16_t)((best_slot + 1u) % STA_TOTAL_SLOTS);
        g_sta_have_saved = 1u;
        g_sta_saved_loaded = best_ch;
    }
    else
    {
        g_sta_seq = 0u;
        g_sta_slot = 0u;
        g_sta_have_saved = 0u;
        g_sta_saved_loaded = 0xFFu;
    }

    *loaded_ch = best_ch;
    return true;
}

bool Flash_AMS_state_write(uint8_t loaded_ch)
{
    if (g_sta_have_saved && g_sta_saved_loaded == loaded_ch)
        return true;

    const uint16_t seq = g_sta_seq;
    const uint32_t w0 = ((uint32_t)STA_TAG << 24) | ((uint32_t)seq << 8) | (uint32_t)loaded_ch;
    const uint32_t w1 = w0 ^ MAGIC_STA;
    const uint32_t slot = (uint32_t)g_sta_slot;
    const uint32_t addr = sta_slot_addr(slot);
    const uint32_t buf[2] = { w0, w1 };
    const uint32_t page_i = slot / STA_SLOTS_PER_PAGE;
    const uint32_t page = sta_page_addr(page_i);

    // Erase first when the slot is not erased (the log has wrapped onto older records, or the page
    // holds a torn record or another layout's data) instead of programming over it; a program that
    // still fails erases the page and retries once, as before.
    if (nvm_sta_needs_erase((const uint32_t*)page, slot))
    {
        if (!flash256_erase(page)) return false;
    }

    if (!flash_prog_words(addr, buf, 2u))
    {
        if (!flash256_erase(page)) return false;
        if (!flash_prog_words(addr, buf, 2u)) return false;
    }

    g_sta_seq = (uint16_t)(seq + 1u);
    g_sta_slot = (uint16_t)((slot + 1u) % STA_TOTAL_SLOTS);
    g_sta_have_saved = 1u;
    g_sta_saved_loaded = loaded_ch;

    return true;
}

struct alignas(4) Flash_CAL_payload
{
    float offs[4];
    float vmin[4];
    float vmax[4];
};

bool Flash_MC_PULL_cal_write_all(const float offs[4], const float vmin[4], const float vmax[4], const int8_t pol[4],
                                 uint8_t fallback_mask)
{
    Flash_CAL_payload p;
    memcpy(p.offs, offs, sizeof(p.offs));
    memcpy(p.vmin, vmin, sizeof(p.vmin));
    memcpy(p.vmax, vmax, sizeof(p.vmax));

    const uint32_t rsv = mc_pull_cal_rsv_pack(pol, fallback_mask);

    return nvm256_write(FLASH_NVM_CAL_ADDR, MAGIC_CAL, VER_1, rsv, &p, (uint16_t)sizeof(p));
}

bool Flash_MC_PULL_cal_read(float offs[4], float vmin[4], float vmax[4], int8_t pol[4], uint8_t* fallback_mask)
{
    Flash_CAL_payload p;
    uint16_t got = 0u;
    uint32_t rsv = 0u;

    if (!nvm256_read(FLASH_NVM_CAL_ADDR, MAGIC_CAL, VER_1,
                     &p, (uint16_t)sizeof(p), &got, &rsv))
        return false;

    if (got != sizeof(p)) return false;

    memcpy(offs, p.offs, sizeof(p.offs));
    memcpy(vmin, p.vmin, sizeof(p.vmin));
    memcpy(vmax, p.vmax, sizeof(p.vmax));

    if (pol)
    {
        for (uint8_t ch = 0u; ch < 4u; ch++)
            pol[ch] = mc_pull_cal_rsv_pol(rsv, ch);
    }
    if (fallback_mask) *fallback_mask = mc_pull_cal_rsv_fallback(rsv);

    return true;
}

bool Flash_MC_PULL_cal_clear(void)
{
    return flash256_erase(FLASH_NVM_CAL_ADDR);
}

bool Flash_Motion_write(const void* in, uint16_t bytes)
{
    if (!in || bytes == 0u) return false;
    return nvm256_write(FLASH_NVM_MOTION_ADDR, MAGIC_MOT, VER_1, 0u, in, bytes);
}

bool Flash_Motion_read(void* out, uint16_t bytes)
{
    if (!out || bytes == 0u) return false;

    uint16_t got = 0u;
    if (!nvm256_read(FLASH_NVM_MOTION_ADDR, MAGIC_MOT, VER_1,
                     out, bytes, &got, nullptr))
        return false;

    return (got != 0u) && (got <= bytes);
}

bool Flash_Motion_clear(void)
{
    return flash256_erase(FLASH_NVM_MOTION_ADDR);
}