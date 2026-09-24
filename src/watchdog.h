#pragma once
// Independent watchdog, reset cause and the fail-safe trap handlers (watchdog.cpp). Timing and
// the list of what feeds it: watchdog_cfg.h.
#include "ch32v20x.h"
#include "watchdog_cfg.h"

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

// The cause of the last reset, from RCC_RSTSCKR. Clears the flags, so the next boot sees only its
// own reset.
wdg_reset_cause watchdog_reset_cause_take(void);
