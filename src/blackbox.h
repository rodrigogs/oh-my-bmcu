#pragma once
// A small fault record kept across resets in the backup data registers (watchdog.cpp writes it,
// main.cpp shows it at boot; hardware-free, host-tested in test/test_blackbox).
//
// CH32FV2x_V3xRM chapter 4 (BKP): the CH32V203C8 (CH32V20x_D6) has 10 backup data registers,
// BKP_DATAR1-10, 16 bits each at 4-byte steps from 0x40006C04; DATAR11-42 are D8 parts only. Reads
// need the PWR and BKP clocks (RCC_APB1PCENR PWREN, BKPEN) and writes also PWR_CTLR.DBP, all off
// after a reset. Chapter 3.2: a system reset (NRST pin, IWDG, WWDG, software, low-power) resets
// every register but the RCC_RSTSCKR flags and the backup domain; a power reset (POR/PDR) every
// register but the backup domain, which is only reset by BDRST, BKPRST, a tamper event (TPE, off)
// or VDD coming up after VDD and VBAT were both off. With VBAT tied to VDD (the RM's advice when no
// battery is used) a power cycle clears the registers to 0, so the record lives from one power-on
// to the next.
//
// The record is written at every boot (blackbox_boot_words) and then updated with plain register
// writes, no read-modify-write:
// - header, written at boot: a magic that also holds the layout version, the RCC_RSTSCKR flags of
//   the reset that started this run, the resets in a row without a power-on, and a check word;
// - the run's phase (blackbox_phase), rewritten as main() goes from one step to the next;
// - the longest main-loop pass so far, in SysTick ticks (HCLK/8, 18 MHz), when it grows;
// - on a trap, failsafe_stop writes mepc, mcause and their check, the check last. Before
//   blackbox_boot turns the backup domain on (startup code, SystemInit, time_hw_init) the record
//   is skipped: failsafe_stop reads the clocks and DBP back first (blackbox_bkp_clocked, _writable).
// All zero (a cold backup domain), all 0xFFFF, or a record left by a reset during the boot write
// (the magic is cleared first and written last, blackbox_boot_store_word) does not check, and reads
// as no record. The trap words only count with a checked header and their own check, and every
// boot clears them, so a hang after an earlier trap reads as a hang.
#include <stdbool.h>
#include <stdint.h>

#include "watchdog_cfg.h"

#define BLACKBOX_WORDS 10u // BKP_DATAR1..BKP_DATAR10

// Word i is BKP_DATAR(i + 1).
#define BLACKBOX_W_MAGIC      0u
#define BLACKBOX_W_RESET      1u // low byte: RCC_RSTSCKR >> 26 (WDG_RSTF_*); high byte: resets in a row
#define BLACKBOX_W_CHECK      2u
#define BLACKBOX_W_PHASE      3u
#define BLACKBOX_W_PASS_LO    4u
#define BLACKBOX_W_PASS_HI    5u
#define BLACKBOX_W_MEPC_LO    6u
#define BLACKBOX_W_MEPC_HI    7u
#define BLACKBOX_W_MCAUSE     8u
#define BLACKBOX_W_TRAP_CHECK 9u

#define BLACKBOX_MAGIC      0xBB01u // layout 1
#define BLACKBOX_TRAP_MAGIC 0x7A9Cu
#define BLACKBOX_RSTF_SHIFT 26u     // WDG_RSTF_PIN, the lowest reset flag
#define BLACKBOX_RESETS_MAX 255u

// The backup domain's clocks and write enable (watchdog.cpp checks them against ch32v20x.h).
#define BLACKBOX_RCC_PWREN (1u << 28) // RCC_APB1PCENR
#define BLACKBOX_RCC_BKPEN (1u << 27) // RCC_APB1PCENR
#define BLACKBOX_PWR_DBP   (1u << 8)  // PWR_CTLR

// What main() was doing. Interrupt handlers have no phase of their own: a hang or trap in one shows
// the phase it interrupted (mepc tells the handler).
typedef enum
{
    BLACKBOX_PHASE_NONE = 0,    // never written
    BLACKBOX_PHASE_BOOT,        // main() up to the first-boot calibration: LEDs, NVM and ADC init
    BLACKBOX_PHASE_CALIBRATION, // MC_PULL_calibration_boot (the watchdog does not run yet: only a trap resets)
    BLACKBOX_PHASE_NVM_READ,    // watchdog_start, the filament and loaded-channel records
    BLACKBOX_PHASE_MOTION_INIT, // Motion_control_init: AS5600, PWM, the motor-direction test
    BLACKBOX_PHASE_BUS_INIT,    // bambubus_init, bus_init
    BLACKBOX_PHASE_AHUB,        // main loop from the feed: ahubus_run
    BLACKBOX_PHASE_BAMBUBUS,    // bambubus_run
    BLACKBOX_PHASE_SEND,        // send_package, the host-link decision and the SYS LED
    BLACKBOX_PHASE_NVM,         // ams_nvm_save_run
    BLACKBOX_PHASE_MOTION,      // Motion_control_run (also the recalibration hold and its reboot)
    BLACKBOX_PHASE_RGB,         // RGB_update
} blackbox_phase;

typedef struct
{
    bool     valid;       // the header checks
    uint8_t  reset_flags; // RCC_RSTSCKR >> BLACKBOX_RSTF_SHIFT of the reset that started the run
    uint8_t  resets;      // resets since the last power-on the record saw, up to BLACKBOX_RESETS_MAX
    uint8_t  phase;       // blackbox_phase when the run ended
    uint32_t pass_max;    // longest main-loop pass, SysTick ticks
    bool     trapped;     // the run ended in failsafe_stop (valid only)
    uint8_t  mcause;      // blackbox_mcause_code of the trap
    uint32_t mepc;        // the trap's mepc
} blackbox_t;

// mcause in one byte: the exception code (bits 0-6) and the interrupt bit (bit 31) as bit 7, so an
// NMI (interrupt 2) reads 0x82 and an illegal instruction 0x02.
static inline __attribute__((always_inline)) uint16_t blackbox_mcause_code(uint32_t mcause)
{
    return (uint16_t)((mcause & 0x7Fu) | ((mcause >> 24) & 0x80u));
}

static inline __attribute__((always_inline)) uint16_t blackbox_trap_check(uint16_t mepc_lo, uint16_t mepc_hi,
                                                                         uint16_t mcause_code)
{
    return (uint16_t)(BLACKBOX_TRAP_MAGIC ^ mepc_lo ^ mepc_hi ^ mcause_code);
}

// Whether failsafe_stop can write the record: both clocks on (PWR_CTLR reads only then), then DBP.
// Register values, not a RAM flag: a trap in the startup code comes before .bss is zeroed.
static inline __attribute__((always_inline)) bool blackbox_bkp_clocked(uint32_t apb1pcenr)
{
    const uint32_t both = BLACKBOX_RCC_PWREN | BLACKBOX_RCC_BKPEN;
    return (apb1pcenr & both) == both;
}

static inline __attribute__((always_inline)) bool blackbox_bkp_writable(uint32_t pwr_ctlr)
{
    return (pwr_ctlr & BLACKBOX_PWR_DBP) != 0u;
}

// The reset word's complement: a reset between the boot's writes of the two leaves them unmatched.
static inline uint16_t blackbox_header_check(uint16_t reset)
{
    return (uint16_t)~reset;
}

static inline void blackbox_decode(const uint16_t w[BLACKBOX_WORDS], blackbox_t *r)
{
    r->valid = (w[BLACKBOX_W_MAGIC] == BLACKBOX_MAGIC) &&
               (w[BLACKBOX_W_CHECK] == blackbox_header_check(w[BLACKBOX_W_RESET]));
    r->reset_flags = (uint8_t)w[BLACKBOX_W_RESET];
    r->resets = (uint8_t)(w[BLACKBOX_W_RESET] >> 8);
    r->phase = (uint8_t)w[BLACKBOX_W_PHASE];
    r->pass_max = (uint32_t)w[BLACKBOX_W_PASS_LO] | ((uint32_t)w[BLACKBOX_W_PASS_HI] << 16);
    r->trapped = r->valid && (w[BLACKBOX_W_TRAP_CHECK] ==
                              blackbox_trap_check(w[BLACKBOX_W_MEPC_LO], w[BLACKBOX_W_MEPC_HI], w[BLACKBOX_W_MCAUSE]));
    r->mcause = (uint8_t)w[BLACKBOX_W_MCAUSE];
    r->mepc = (uint32_t)w[BLACKBOX_W_MEPC_LO] | ((uint32_t)w[BLACKBOX_W_MEPC_HI] << 16);
}

// The words of r; the trap words are all 0 (no trap) unless r->trapped.
static inline void blackbox_encode(const blackbox_t *r, uint16_t w[BLACKBOX_WORDS])
{
    const uint16_t reset = (uint16_t)(r->reset_flags | ((uint16_t)r->resets << 8));
    w[BLACKBOX_W_MAGIC] = BLACKBOX_MAGIC;
    w[BLACKBOX_W_RESET] = reset;
    w[BLACKBOX_W_CHECK] = blackbox_header_check(reset);
    w[BLACKBOX_W_PHASE] = r->phase;
    w[BLACKBOX_W_PASS_LO] = (uint16_t)r->pass_max;
    w[BLACKBOX_W_PASS_HI] = (uint16_t)(r->pass_max >> 16);
    if (r->trapped)
    {
        w[BLACKBOX_W_MEPC_LO] = (uint16_t)r->mepc;
        w[BLACKBOX_W_MEPC_HI] = (uint16_t)(r->mepc >> 16);
        w[BLACKBOX_W_MCAUSE] = r->mcause;
        w[BLACKBOX_W_TRAP_CHECK] = blackbox_trap_check(w[BLACKBOX_W_MEPC_LO], w[BLACKBOX_W_MEPC_HI], r->mcause);
    }
    else
    {
        w[BLACKBOX_W_MEPC_LO] = 0u;
        w[BLACKBOX_W_MEPC_HI] = 0u;
        w[BLACKBOX_W_MCAUSE] = 0u;
        w[BLACKBOX_W_TRAP_CHECK] = 0u;
    }
}

// The record of the run that starts with a reset whose RCC_RSTSCKR flags are rstsckr, after the run
// recorded in last: the new flags, the count restarted at 0 by a power-on (and at 1 after any other
// reset with no record: the record starts there), phase BOOT, no pass and no trap yet.
static inline blackbox_t blackbox_next(const blackbox_t *last, uint32_t rstsckr)
{
    blackbox_t r;
    r.valid = true;
    r.reset_flags = (uint8_t)(rstsckr >> BLACKBOX_RSTF_SHIFT);
    if (rstsckr & WDG_RSTF_POR)
        r.resets = 0u;
    else if (!last->valid)
        r.resets = 1u;
    else
        r.resets = (uint8_t)((last->resets < BLACKBOX_RESETS_MAX) ? last->resets + 1u : BLACKBOX_RESETS_MAX);
    r.phase = BLACKBOX_PHASE_BOOT;
    r.pass_max = 0u;
    r.trapped = false;
    r.mcause = 0u;
    r.mepc = 0u;
    return r;
}

// At boot, on the words read from the backup registers: *last gets the run that just ended, and w
// the new run's record, to be written back.
static inline void blackbox_boot_words(uint16_t w[BLACKBOX_WORDS], uint32_t rstsckr, blackbox_t *last)
{
    blackbox_decode(w, last);
    const blackbox_t next = blackbox_next(last, rstsckr);
    blackbox_encode(&next, w);
}

// blackbox_boot's register writes of the new record w, in order: store k (0 to
// BLACKBOX_BOOT_STORES - 1) writes blackbox_boot_store_value(k, w) to word
// blackbox_boot_store_word(k). The magic goes to 0 first and gets its value last, so after a reset
// between two stores the header does not check: the next boot reads no record. Written in word
// order alone, a reset after the new header and before the trap words left run N + 1's checked
// header over run N's trap, read at the next boot as a trap of run N + 1.
#define BLACKBOX_BOOT_STORES (BLACKBOX_WORDS + 1u)

static inline uint32_t blackbox_boot_store_word(uint32_t k)
{
    return (k < BLACKBOX_WORDS) ? k : BLACKBOX_W_MAGIC;
}

static inline uint16_t blackbox_boot_store_value(uint32_t k, const uint16_t w[BLACKBOX_WORDS])
{
    return (k == 0u) ? 0u : w[blackbox_boot_store_word(k)];
}

// The SYS LED boot code (main.cpp, timing in watchdog_cfg.h): the watchdog flash after an IWDG reset,
// as before, and after a trap (which ends in an IWDG reset) the blue flash after it.
typedef enum
{
    BLACKBOX_FLASH_NONE = 0,
    BLACKBOX_FLASH_WATCHDOG, // magenta x3
    BLACKBOX_FLASH_TRAP,     // magenta x3, then blue
} blackbox_flash;

static inline blackbox_flash blackbox_boot_flash(const blackbox_t *last, uint32_t rstsckr)
{
    if (wdg_reset_cause_decode(rstsckr) != WDG_RESET_IWDG) return BLACKBOX_FLASH_NONE;
    return last->trapped ? BLACKBOX_FLASH_TRAP : BLACKBOX_FLASH_WATCHDOG;
}

// Longest main-loop pass: blackbox_pass_begin before the loop, blackbox_pass_end at the top of every
// pass. Wrap-safe for passes under 2^32 ticks (238 s).
typedef struct
{
    uint32_t start;
    uint32_t max;
} blackbox_pass_t;

static inline void blackbox_pass_begin(blackbox_pass_t *p, uint32_t now)
{
    p->start = now;
    p->max = 0u;
}

// The pass since the previous call ends at now; true if it is the longest so far (p->max).
static inline bool blackbox_pass_end(blackbox_pass_t *p, uint32_t now)
{
    const uint32_t dt = now - p->start;
    p->start = now;
    if (dt <= p->max) return false;
    p->max = dt;
    return true;
}
