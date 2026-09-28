#pragma once
// Independent watchdog, reset cause, the fault record and the fail-safe trap handlers (watchdog.cpp).
// Timing and the list of what feeds it: watchdog_cfg.h; the record's layout: blackbox.h.
#include "ch32v20x.h"
#include "watchdog_cfg.h"
#include "blackbox.h"

// Starts the IWDG with WDG_TIMEOUT_MS (1 s nominal, 0.67-1.6 s over the LSI range). Only a reset
// stops it again.
void watchdog_start(void);

extern uint8_t g_watchdog_on; // set by watchdog_start()

// Feed: one register write (reload key), and only once watchdog_start() has run. WCH does not
// document whether a reload key can start a stopped IWDG (on STM32F1 it cannot), and the ADC wait
// feeds during boot, before the start, so no key is written until then.
static inline __attribute__((always_inline)) void watchdog_feed(void)
{
    if (g_watchdog_on) IWDG->CTLR = 0xAAAAu;
}

// RCC_RSTSCKR of the last reset (the WDG_RSTF_* flags; wdg_reset_cause_decode). Clears the flags, so
// the next boot sees only its own reset.
uint32_t watchdog_reset_flags_take(void);

// ===== fault record (blackbox.h) =====
// Turns the backup registers on (PWR and BKP clocks, PWR_CTLR.DBP), decodes the record of the run
// that just ended into g_blackbox_last and writes this run's, from this boot's RCC_RSTSCKR. main
// calls it right after the clock and SysTick setup and the reset-flag read (SystemInit,
// SystemCoreClockUpdate, time_hw_init, watchdog_reset_flags_take), well before watchdog_start, so
// that failsafe_stop records a trap from then on; a trap before it leaves no record.
void blackbox_boot(uint32_t rstsckr);

extern blackbox_t g_blackbox_last;      // the run before this boot (also for a debugger)
extern blackbox_pass_t g_blackbox_pass; // this run's longest main-loop pass

// Word i of the record: BKP_DATAR(i + 1), 4 bytes apart (watchdog.cpp checks the SDK's layout).
static inline __attribute__((always_inline)) volatile uint16_t *blackbox_bkp(uint32_t i)
{
    return (volatile uint16_t *)(BKP_BASE + 4u + 4u * i);
}

// One 16-bit store.
static inline __attribute__((always_inline)) void blackbox_phase_set(blackbox_phase p)
{
    *blackbox_bkp(BLACKBOX_W_PHASE) = (uint16_t)p;
}

// At the top of every main-loop pass; the two words are written only when the longest pass grows.
static inline __attribute__((always_inline)) void blackbox_pass_mark(uint32_t now)
{
    if (blackbox_pass_end(&g_blackbox_pass, now))
    {
        *blackbox_bkp(BLACKBOX_W_PASS_LO) = (uint16_t)g_blackbox_pass.max;
        *blackbox_bkp(BLACKBOX_W_PASS_HI) = (uint16_t)(g_blackbox_pass.max >> 16);
    }
}
