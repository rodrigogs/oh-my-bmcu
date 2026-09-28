// Host tests for src/blackbox.h: the fault record watchdog.cpp keeps in the 10 backup data
// registers across resets, and the SYS LED boot code main.cpp shows from it. A cold backup domain
// (all 0), all 0xFFFF, random words and a header torn by a reset must read as no record; a record
// must survive encode and decode; the boots of a power-on, hangs, traps and software resets must
// count and report as the firmware means them; and the boot code must be the watchdog flash for
// exactly the IWDG resets, plus the trap flash only after a trap.
//
// watchdog.cpp and main.cpp need the CH32 SDK and are not built on the host. The backup registers
// are an array here; the boot and the trap handler's writes are modelled below. The RCC_RSTSCKR flag
// values are the RCC_*RSTF values of ch32v20x.h (watchdog.cpp checks watchdog_cfg.h against them).

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "blackbox.h"

// ch32v20x.h, RCC_RSTSCKR
#define SDK_PINRSTF  0x04000000u
#define SDK_PORRSTF  0x08000000u
#define SDK_SFTRSTF  0x10000000u
#define SDK_IWDGRSTF 0x20000000u
#define SDK_WWDGRSTF 0x40000000u
#define SDK_LPWRRSTF 0x80000000u
#define POWER_ON     (SDK_PORRSTF | SDK_PINRSTF) // RSTSCKR reset value
#define IWDG         (SDK_IWDGRSTF | SDK_PINRSTF) // an internal reset can set PINRSTF too

// SysTick runs at HCLK/8 = 18 MHz.
#define TPMS 18000u

void setUp(void) {}
void tearDown(void) {}

// ---- model of the backup registers and of the firmware's writes ----

static uint16_t bkp[BLACKBOX_WORDS];

static void power_loss(void)
{
    memset(bkp, 0, sizeof(bkp)); // VDD and VBAT off: backup domain reset, every register 0
}

// ---- adapted from watchdog.cpp: blackbox_boot(), read, blackbox_boot_words, write back ----
static blackbox_t boot(uint32_t rstsckr)
{
    uint16_t w[BLACKBOX_WORDS];
    for (uint32_t i = 0u; i < BLACKBOX_WORDS; i++) w[i] = bkp[i];
    blackbox_t last;
    blackbox_boot_words(w, rstsckr, &last);
    for (uint32_t i = 0u; i < BLACKBOX_WORDS; i++) bkp[i] = w[i];
    return last;
}

// ---- adapted from watchdog.h: blackbox_phase_set and blackbox_pass_mark ----
static void phase(blackbox_phase p)
{
    bkp[BLACKBOX_W_PHASE] = (uint16_t)p;
}

static void pass_mark(blackbox_pass_t *p, uint32_t now)
{
    if (blackbox_pass_end(p, now))
    {
        bkp[BLACKBOX_W_PASS_LO] = (uint16_t)p->max;
        bkp[BLACKBOX_W_PASS_HI] = (uint16_t)(p->max >> 16);
    }
}

// ---- adapted from watchdog.cpp: failsafe_stop's record writes (mepc and mcause from the CSRs) ----
static void trap(uint32_t mepc, uint32_t mcause)
{
    const uint16_t pc_lo = (uint16_t)mepc;
    const uint16_t pc_hi = (uint16_t)(mepc >> 16);
    const uint16_t code = blackbox_mcause_code(mcause);
    bkp[BLACKBOX_W_MEPC_LO] = pc_lo;
    bkp[BLACKBOX_W_MEPC_HI] = pc_hi;
    bkp[BLACKBOX_W_MCAUSE] = code;
    bkp[BLACKBOX_W_TRAP_CHECK] = blackbox_trap_check(pc_lo, pc_hi, code);
}

static blackbox_t decoded(const uint16_t w[BLACKBOX_WORDS])
{
    blackbox_t r;
    blackbox_decode(w, &r);
    return r;
}

static uint32_t lcg = 12345u;
static uint32_t rnd(void)
{
    lcg = lcg * 1664525u + 1013904223u;
    return lcg;
}

// ---- layout ----

static void test_the_record_is_the_ten_backup_registers_of_the_v203c8(void)
{
    // CH32FV2x_V3xRM table 4-1: DATAR1-10 on CH32V20x_D6; DATAR11-42 are D8 parts only.
    TEST_ASSERT_EQUAL_UINT32(10u, BLACKBOX_WORDS);

    const uint32_t idx[] = {BLACKBOX_W_MAGIC,   BLACKBOX_W_RESET,   BLACKBOX_W_CHECK,   BLACKBOX_W_PHASE,
                            BLACKBOX_W_PASS_LO, BLACKBOX_W_PASS_HI, BLACKBOX_W_MEPC_LO, BLACKBOX_W_MEPC_HI,
                            BLACKBOX_W_MCAUSE,  BLACKBOX_W_TRAP_CHECK};
    TEST_ASSERT_EQUAL_UINT32(BLACKBOX_WORDS, sizeof(idx) / sizeof(idx[0]));
    uint32_t seen = 0u;
    for (uint32_t i = 0u; i < BLACKBOX_WORDS; i++)
    {
        TEST_ASSERT_TRUE(idx[i] < BLACKBOX_WORDS);
        seen |= 1u << idx[i];
    }
    TEST_ASSERT_EQUAL_HEX32(0x3FFu, seen); // each register once

    // The six reset flags fit the low byte of the reset word, WDG_RSTF_PIN at bit 0.
    TEST_ASSERT_EQUAL_HEX32(SDK_PINRSTF, 1u << BLACKBOX_RSTF_SHIFT);
    TEST_ASSERT_EQUAL_HEX32(0x3Fu, (SDK_PINRSTF | SDK_PORRSTF | SDK_SFTRSTF | SDK_IWDGRSTF | SDK_WWDGRSTF |
                                    SDK_LPWRRSTF) >> BLACKBOX_RSTF_SHIFT);

    // A cleared or erased-looking register is neither magic.
    TEST_ASSERT_TRUE(BLACKBOX_MAGIC != 0u && BLACKBOX_MAGIC != 0xFFFFu);
    TEST_ASSERT_TRUE(BLACKBOX_TRAP_MAGIC != 0u && BLACKBOX_TRAP_MAGIC != 0xFFFFu);
    TEST_ASSERT_TRUE(BLACKBOX_PHASE_RGB <= 0xFFu); // one byte
}

// ---- no record ----

static void test_a_cold_backup_domain_reads_as_no_record(void)
{
    uint16_t w[BLACKBOX_WORDS];

    memset(w, 0, sizeof(w)); // the registers' reset value
    blackbox_t r = decoded(w);
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_FALSE(r.trapped);

    memset(w, 0xFF, sizeof(w));
    r = decoded(w);
    TEST_ASSERT_FALSE(r.valid);
    TEST_ASSERT_FALSE(r.trapped);

    // 0 words with a valid trap check and nothing else: still no record, so no trap.
    memset(w, 0, sizeof(w));
    w[BLACKBOX_W_TRAP_CHECK] = blackbox_trap_check(0u, 0u, 0u);
    TEST_ASSERT_FALSE(decoded(w).trapped);
}

static void test_random_words_read_as_no_record(void)
{
    uint32_t valid = 0u, trapped = 0u;
    for (uint32_t n = 0u; n < 200000u; n++)
    {
        uint16_t w[BLACKBOX_WORDS];
        for (uint32_t i = 0u; i < BLACKBOX_WORDS; i++) w[i] = (uint16_t)(rnd() >> 16);
        const blackbox_t r = decoded(w);
        valid += r.valid;
        trapped += r.trapped;
    }
    // 32 checked bits: expected 200000 / 2^32 valid headers.
    TEST_ASSERT_EQUAL_UINT32(0u, valid);
    TEST_ASSERT_EQUAL_UINT32(0u, trapped);
}

static void test_any_bit_changed_in_the_header_reads_as_no_record(void)
{
    // A reset during the boot write, or another layout: every single-bit change of the three header
    // words fails the check.
    const blackbox_t last = {false, 0u, 0u, 0u, 0u, false, 0u, 0u};
    blackbox_t r = blackbox_next(&last, IWDG);
    r.trapped = true;
    r.mepc = 0x00001234u;
    r.mcause = 2u;
    uint16_t w[BLACKBOX_WORDS];
    blackbox_encode(&r, w);
    TEST_ASSERT_TRUE(decoded(w).valid);

    const uint32_t header[3] = {BLACKBOX_W_MAGIC, BLACKBOX_W_RESET, BLACKBOX_W_CHECK};
    for (uint32_t h = 0u; h < 3u; h++)
    {
        for (uint32_t b = 0u; b < 16u; b++)
        {
            uint16_t t[BLACKBOX_WORDS];
            memcpy(t, w, sizeof(t));
            t[header[h]] ^= (uint16_t)(1u << b);
            const blackbox_t d = decoded(t);
            TEST_ASSERT_FALSE(d.valid);
            TEST_ASSERT_FALSE(d.trapped);
        }
    }

    // A new reset word written, the old check still there.
    uint16_t t[BLACKBOX_WORDS];
    memcpy(t, w, sizeof(t));
    t[BLACKBOX_W_RESET] = (uint16_t)(t[BLACKBOX_W_RESET] + 0x0100u);
    TEST_ASSERT_FALSE(decoded(t).valid);

    // Layout 0 (another magic) with its own consistent check.
    memcpy(t, w, sizeof(t));
    t[BLACKBOX_W_MAGIC] = (uint16_t)(BLACKBOX_MAGIC - 1u);
    t[BLACKBOX_W_CHECK] = blackbox_header_check(t[BLACKBOX_W_RESET]);
    TEST_ASSERT_FALSE(decoded(t).valid);
}

static void test_any_bit_changed_in_the_trap_words_reads_as_no_trap(void)
{
    // A reset between the handler's four writes: the header still checks, the trap does not.
    const blackbox_t last = {false, 0u, 0u, 0u, 0u, false, 0u, 0u};
    blackbox_t r = blackbox_next(&last, IWDG);
    r.trapped = true;
    r.mepc = 0x0000A5C2u;
    r.mcause = 7u;
    uint16_t w[BLACKBOX_WORDS];
    blackbox_encode(&r, w);
    TEST_ASSERT_TRUE(decoded(w).trapped);

    const uint32_t words[4] = {BLACKBOX_W_MEPC_LO, BLACKBOX_W_MEPC_HI, BLACKBOX_W_MCAUSE, BLACKBOX_W_TRAP_CHECK};
    for (uint32_t k = 0u; k < 4u; k++)
    {
        for (uint32_t b = 0u; b < 16u; b++)
        {
            uint16_t t[BLACKBOX_WORDS];
            memcpy(t, w, sizeof(t));
            t[words[k]] ^= (uint16_t)(1u << b);
            const blackbox_t d = decoded(t);
            TEST_ASSERT_TRUE(d.valid);
            TEST_ASSERT_FALSE(d.trapped);
        }
    }

    // Only the first write done (the other trap words still the boot's 0): no trap.
    blackbox_t n = r;
    n.trapped = false;
    blackbox_encode(&n, w);
    w[BLACKBOX_W_MEPC_LO] = 0xA5C2u;
    TEST_ASSERT_FALSE(decoded(w).trapped);
}

// ---- encode and decode ----

static void assert_same(const blackbox_t *a, const blackbox_t *b)
{
    TEST_ASSERT_EQUAL(a->valid, b->valid);
    TEST_ASSERT_EQUAL_HEX8(a->reset_flags, b->reset_flags);
    TEST_ASSERT_EQUAL_UINT8(a->resets, b->resets);
    TEST_ASSERT_EQUAL_UINT8(a->phase, b->phase);
    TEST_ASSERT_EQUAL_UINT32(a->pass_max, b->pass_max);
    TEST_ASSERT_EQUAL(a->trapped, b->trapped);
    if (a->trapped)
    {
        TEST_ASSERT_EQUAL_HEX8(a->mcause, b->mcause);
        TEST_ASSERT_EQUAL_HEX32(a->mepc, b->mepc);
    }
}

static void test_a_record_survives_encode_and_decode(void)
{
    const blackbox_t cases[] = {
        {true, 0x03u, 0u, BLACKBOX_PHASE_BOOT, 0u, false, 0u, 0u},
        {true, 0x09u, 1u, BLACKBOX_PHASE_MOTION, 648000u, false, 0u, 0u}, // a 36 ms pass: over 16 bits
        {true, 0x3Fu, 255u, BLACKBOX_PHASE_RGB, 0xFFFFFFFFu, true, 0x82u, 0xFFFFFFFEu},
        {true, 0x08u, 7u, BLACKBOX_PHASE_BAMBUBUS, 0x00010000u, true, 0x02u, 0x00000000u},
        {true, 0x09u, 2u, BLACKBOX_PHASE_CALIBRATION, 17u, true, 0x05u, 0x00004F3Au},
    };
    for (uint32_t i = 0u; i < sizeof(cases) / sizeof(cases[0]); i++)
    {
        uint16_t w[BLACKBOX_WORDS];
        blackbox_encode(&cases[i], w);
        const blackbox_t d = decoded(w);
        assert_same(&cases[i], &d);
    }

    // Without a trap the trap words are 0, as every boot leaves them.
    uint16_t w[BLACKBOX_WORDS];
    const blackbox_t n = {true, 0x09u, 1u, BLACKBOX_PHASE_NVM, 5u, false, 0x02u, 0x1234u};
    blackbox_encode(&n, w);
    TEST_ASSERT_EQUAL_HEX16(0u, w[BLACKBOX_W_MEPC_LO] | w[BLACKBOX_W_MEPC_HI] | w[BLACKBOX_W_MCAUSE] |
                                    w[BLACKBOX_W_TRAP_CHECK]);
}

static void test_mcause_keeps_the_code_and_the_interrupt_bit(void)
{
    // QingKe V4 exceptions (fetch, illegal instruction, breakpoint, load/store misaligned and
    // access faults, ecall) keep their code; the NMI (interrupt 2) gets bit 7.
    const uint32_t codes[] = {0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 11u};
    for (uint32_t i = 0u; i < sizeof(codes) / sizeof(codes[0]); i++)
        TEST_ASSERT_EQUAL_HEX16(codes[i], blackbox_mcause_code(codes[i]));
    TEST_ASSERT_EQUAL_HEX16(0x82u, blackbox_mcause_code(0x80000002u));
    TEST_ASSERT_EQUAL_HEX16(0x80u, blackbox_mcause_code(0x80000000u));
    TEST_ASSERT_EQUAL_HEX16(0x7Fu, blackbox_mcause_code(0x7FFFFFFFu)); // bits 7-30 dropped
    TEST_ASSERT_EQUAL_HEX16(0xFFu, blackbox_mcause_code(0xFFFFFFFFu));
    TEST_ASSERT_TRUE(blackbox_mcause_code(0xFFFFFFFFu) <= 0xFFu); // one byte
}

// ---- boots ----

static void test_boots_from_power_on_through_hangs_traps_and_resets(void)
{
    blackbox_pass_t pass;

    // Power-on: no record yet; the new one has the POR and PIN flags, no reset counted, phase BOOT.
    power_loss();
    blackbox_t last = boot(POWER_ON);
    TEST_ASSERT_FALSE(last.valid);
    TEST_ASSERT_EQUAL_INT(BLACKBOX_FLASH_NONE, blackbox_boot_flash(&last, POWER_ON));
    blackbox_t now = decoded(bkp);
    TEST_ASSERT_TRUE(now.valid);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(POWER_ON >> 26), now.reset_flags);
    TEST_ASSERT_EQUAL_UINT8(0u, now.resets);
    TEST_ASSERT_EQUAL_UINT8(BLACKBOX_PHASE_BOOT, now.phase);
    TEST_ASSERT_EQUAL_UINT32(0u, now.pass_max);
    TEST_ASSERT_FALSE(now.trapped);

    // The run: boot steps, then the loop with passes of 0.5, 36 (an NVM job) and 1 ms, and it hangs
    // in Motion_control_run.
    phase(BLACKBOX_PHASE_CALIBRATION);
    phase(BLACKBOX_PHASE_BUS_INIT);
    blackbox_pass_begin(&pass, 1000u);
    phase(BLACKBOX_PHASE_AHUB);
    pass_mark(&pass, 1000u + TPMS / 2u);
    pass_mark(&pass, 1000u + TPMS / 2u + 36u * TPMS);
    pass_mark(&pass, 1000u + TPMS / 2u + 37u * TPMS);
    phase(BLACKBOX_PHASE_MOTION);

    // IWDG reset: the last run hung in MOTION, its longest pass was 36 ms; the watchdog flash only.
    last = boot(IWDG);
    TEST_ASSERT_TRUE(last.valid);
    TEST_ASSERT_EQUAL_UINT8(BLACKBOX_PHASE_MOTION, last.phase);
    TEST_ASSERT_EQUAL_UINT32(36u * TPMS, last.pass_max);
    TEST_ASSERT_FALSE(last.trapped);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(POWER_ON >> 26), last.reset_flags); // the reset that started it
    TEST_ASSERT_EQUAL_INT(BLACKBOX_FLASH_WATCHDOG, blackbox_boot_flash(&last, IWDG));
    now = decoded(bkp);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(IWDG >> 26), now.reset_flags);
    TEST_ASSERT_EQUAL_UINT8(1u, now.resets);
    TEST_ASSERT_EQUAL_UINT8(BLACKBOX_PHASE_BOOT, now.phase);
    TEST_ASSERT_EQUAL_UINT32(0u, now.pass_max); // a new run, a new maximum

    // This run traps (illegal instruction) in bambubus_run; failsafe_stop starts the IWDG.
    blackbox_pass_begin(&pass, 0xFFFFF000u);
    phase(BLACKBOX_PHASE_BAMBUBUS);
    pass_mark(&pass, 0x00000800u); // 0.34 ms across the SysTick wrap
    trap(0x00003A6Eu, 2u);

    last = boot(IWDG);
    TEST_ASSERT_TRUE(last.trapped);
    TEST_ASSERT_EQUAL_HEX32(0x00003A6Eu, last.mepc);
    TEST_ASSERT_EQUAL_HEX8(0x02u, last.mcause);
    TEST_ASSERT_EQUAL_UINT8(BLACKBOX_PHASE_BAMBUBUS, last.phase);
    TEST_ASSERT_EQUAL_UINT32(0x1800u, last.pass_max);
    TEST_ASSERT_EQUAL_UINT8(1u, last.resets);
    TEST_ASSERT_EQUAL_INT(BLACKBOX_FLASH_TRAP, blackbox_boot_flash(&last, IWDG));
    now = decoded(bkp);
    TEST_ASSERT_EQUAL_UINT8(2u, now.resets);
    TEST_ASSERT_FALSE(now.trapped); // cleared for the new run

    // The next run hangs in the NVM step: a hang, not the earlier trap again.
    phase(BLACKBOX_PHASE_NVM);
    last = boot(IWDG);
    TEST_ASSERT_FALSE(last.trapped);
    TEST_ASSERT_EQUAL_UINT8(BLACKBOX_PHASE_NVM, last.phase);
    TEST_ASSERT_EQUAL_INT(BLACKBOX_FLASH_WATCHDOG, blackbox_boot_flash(&last, IWDG));

    // A trap during the first-boot calibration (the watchdog did not run; the handler starts it).
    phase(BLACKBOX_PHASE_CALIBRATION);
    trap(0x00001000u, 5u);
    last = boot(IWDG);
    TEST_ASSERT_TRUE(last.trapped);
    TEST_ASSERT_EQUAL_UINT8(BLACKBOX_PHASE_CALIBRATION, last.phase);

    // The recalibration reboot (software reset) and a WCH-Link reset (pin): counted, no flash.
    phase(BLACKBOX_PHASE_MOTION);
    last = boot(SDK_SFTRSTF | SDK_PINRSTF);
    TEST_ASSERT_EQUAL_INT(BLACKBOX_FLASH_NONE, blackbox_boot_flash(&last, SDK_SFTRSTF | SDK_PINRSTF));
    last = boot(SDK_PINRSTF);
    TEST_ASSERT_EQUAL_INT(BLACKBOX_FLASH_NONE, blackbox_boot_flash(&last, SDK_PINRSTF));
    now = decoded(bkp);
    TEST_ASSERT_EQUAL_UINT8(6u, now.resets);
    TEST_ASSERT_EQUAL_HEX8((uint8_t)(SDK_PINRSTF >> 26), now.reset_flags);

    // A trap, then someone presses reset while the handler spins: no IWDG, no flash.
    trap(0x00002000u, 7u);
    last = boot(SDK_PINRSTF);
    TEST_ASSERT_TRUE(last.trapped);
    TEST_ASSERT_EQUAL_INT(BLACKBOX_FLASH_NONE, blackbox_boot_flash(&last, SDK_PINRSTF));

    // A power-on with the backup domain kept (VBAT fed): the record is read, the count restarts.
    last = boot(POWER_ON);
    TEST_ASSERT_TRUE(last.valid);
    TEST_ASSERT_EQUAL_UINT8(7u, last.resets);
    TEST_ASSERT_EQUAL_UINT8(0u, decoded(bkp).resets);

    // A power cycle with VBAT on VDD: no record, and a new one.
    power_loss();
    last = boot(POWER_ON);
    TEST_ASSERT_FALSE(last.valid);
    TEST_ASSERT_EQUAL_UINT8(0u, decoded(bkp).resets);
    TEST_ASSERT_TRUE(decoded(bkp).valid);
}

static void test_a_reset_without_a_record_starts_the_count_at_one(void)
{
    // The first boot of this firmware after flashing: the flasher resets the chip (no power-on) and
    // the backup registers hold nothing of ours.
    power_loss();
    blackbox_t last = boot(SDK_SFTRSTF | SDK_PINRSTF);
    TEST_ASSERT_FALSE(last.valid);
    TEST_ASSERT_EQUAL_UINT8(1u, decoded(bkp).resets);

    // Garbage of an earlier firmware: the same.
    for (uint32_t i = 0u; i < BLACKBOX_WORDS; i++) bkp[i] = (uint16_t)(0x1111u * (i + 1u));
    last = boot(IWDG);
    TEST_ASSERT_FALSE(last.valid);
    TEST_ASSERT_EQUAL_INT(BLACKBOX_FLASH_WATCHDOG, blackbox_boot_flash(&last, IWDG));
    TEST_ASSERT_EQUAL_UINT8(1u, decoded(bkp).resets);

    // A reset with no flag at all counts as a reset too.
    boot(0u);
    TEST_ASSERT_EQUAL_UINT8(2u, decoded(bkp).resets);
}

static void test_the_reset_count_stops_at_255(void)
{
    power_loss();
    boot(POWER_ON);
    for (uint32_t n = 1u; n <= 300u; n++)
    {
        const blackbox_t last = boot(IWDG);
        TEST_ASSERT_TRUE(last.valid);
        TEST_ASSERT_EQUAL_UINT8(n - 1u < 255u ? n - 1u : 255u, last.resets);
        TEST_ASSERT_EQUAL_UINT8(n < 255u ? n : 255u, decoded(bkp).resets);
    }
    boot(POWER_ON);
    TEST_ASSERT_EQUAL_UINT8(0u, decoded(bkp).resets);
}

// ---- boot code ----

static void test_the_watchdog_flash_shows_for_the_iwdg_resets_only(void)
{
    // As before this record: the magenta flash for every flag combination with IWDGRSTF and none
    // without it, whatever the record holds; the blue one only on top of it, after a trap.
    for (uint32_t m = 0u; m < 64u; m++)
    {
        const uint32_t f = m << 26;
        const bool iwdg = (f & SDK_IWDGRSTF) != 0u;
        TEST_ASSERT_EQUAL(wdg_reset_cause_decode(f) == WDG_RESET_IWDG, iwdg);

        for (uint32_t k = 0u; k < 3u; k++)
        {
            blackbox_t last = {k != 0u, 0x09u, 3u, BLACKBOX_PHASE_MOTION, 99u, k == 2u, 2u, 0x100u};
            const blackbox_flash b = blackbox_boot_flash(&last, f);
            TEST_ASSERT_EQUAL(iwdg, b != BLACKBOX_FLASH_NONE);
            TEST_ASSERT_EQUAL(iwdg && k == 2u, b == BLACKBOX_FLASH_TRAP);
        }
    }
}

// ---- longest pass ----

static void test_the_longest_pass_is_kept_and_reported_once(void)
{
    blackbox_pass_t p;
    blackbox_pass_begin(&p, 500u);
    TEST_ASSERT_EQUAL_UINT32(0u, p.max);

    TEST_ASSERT_TRUE(blackbox_pass_end(&p, 600u)); // 100
    TEST_ASSERT_FALSE(blackbox_pass_end(&p, 650u)); // 50
    TEST_ASSERT_FALSE(blackbox_pass_end(&p, 750u)); // 100 again: not longer
    TEST_ASSERT_TRUE(blackbox_pass_end(&p, 851u));  // 101
    TEST_ASSERT_EQUAL_UINT32(101u, p.max);

    // Across the SysTick wrap (2^32 ticks, 238 s at 18 MHz).
    blackbox_pass_begin(&p, 0xFFFFFF00u);
    TEST_ASSERT_TRUE(blackbox_pass_end(&p, 0x00000100u));
    TEST_ASSERT_EQUAL_UINT32(0x200u, p.max);

    // A 36 ms pass (one NVM job) and the watchdog's longest 1.6 s both fit 32 bits.
    blackbox_pass_begin(&p, 0u);
    TEST_ASSERT_TRUE(blackbox_pass_end(&p, 36u * TPMS));
    TEST_ASSERT_TRUE(blackbox_pass_end(&p, 36u * TPMS + 1601u * TPMS));
    TEST_ASSERT_EQUAL_UINT32(1601u * TPMS, p.max);

    // Over many random passes the maximum is the largest one, and each new one is reported once.
    blackbox_pass_begin(&p, 7u);
    uint32_t t = 7u, want = 0u, reports = 0u, increases = 0u;
    for (uint32_t n = 0u; n < 100000u; n++)
    {
        const uint32_t dt = rnd() % (40u * TPMS);
        t += dt;
        if (dt > want)
        {
            want = dt;
            increases++;
        }
        reports += blackbox_pass_end(&p, t);
        TEST_ASSERT_EQUAL_UINT32(want, p.max);
    }
    TEST_ASSERT_EQUAL_UINT32(increases, reports);
    TEST_ASSERT_TRUE(reports < 100u); // the register writes are rare
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_the_record_is_the_ten_backup_registers_of_the_v203c8);
    RUN_TEST(test_a_cold_backup_domain_reads_as_no_record);
    RUN_TEST(test_random_words_read_as_no_record);
    RUN_TEST(test_any_bit_changed_in_the_header_reads_as_no_record);
    RUN_TEST(test_any_bit_changed_in_the_trap_words_reads_as_no_trap);
    RUN_TEST(test_a_record_survives_encode_and_decode);
    RUN_TEST(test_mcause_keeps_the_code_and_the_interrupt_bit);
    RUN_TEST(test_boots_from_power_on_through_hangs_traps_and_resets);
    RUN_TEST(test_a_reset_without_a_record_starts_the_count_at_one);
    RUN_TEST(test_the_reset_count_stops_at_255);
    RUN_TEST(test_the_watchdog_flash_shows_for_the_iwdg_resets_only);
    RUN_TEST(test_the_longest_pass_is_kept_and_reported_once);
    return UNITY_END();
}
