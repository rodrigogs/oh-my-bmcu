#pragma once
// Independent watchdog (IWDG) timing, reset-cause decoding and the fail-safe motor state for
// src/watchdog.cpp. Hardware-free, so all three are tested on the host (test/test_watchdog_cfg).
//
// The IWDG is a 12-bit down counter clocked by the LSI RC oscillator through a prescaler (PSCR code
// 0..6 divides by 4 << code). A feed (key 0xAAAA) loads the counter with RLDR, and the chip resets
// when it reaches 0. A feed does not restart the prescaler, so the first step after it comes 0 to 1
// prescaler periods later: the reset follows RLDR - 1 to RLDR periods after the last feed.
//
// The LSI is imprecise. CH32V203 datasheet V3.0, table 4-14: 25 kHz min, 39 kHz typ, 60 kHz max
// (the V203RBT6 is 25/32/45 kHz, inside the same bounds). The reference manual and WCH's IWDG
// example (/32, reload 4000 = "3.2 s") use 40 kHz, so the nominal timeout is computed at 40 kHz;
// the bounds are what the watchdog is checked against.
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
#define WDG_CONSTEXPR constexpr
#else
#define WDG_CONSTEXPR
#endif

#define WDG_LSI_MIN_HZ 25000u
#define WDG_LSI_NOM_HZ 40000u
#define WDG_LSI_MAX_HZ 60000u

// Nominal timeout, at WDG_LSI_NOM_HZ.
#define WDG_TIMEOUT_MS 1000u

// Longest time between two feeds once the watchdog runs (watchdog_start() in main, right after the
// buffer calibration), with about 2.5x margin over the ~40 ms estimate below. Page/sector erase
// 16 ms and page program 2 ms are the datasheet's typical values (table 4-17 gives no maximum).
// - Main-loop pass (fed at its top): under 1 ms. The worst pass runs one NVM job: the loaded-channel
//   record can take 2 page erases and 2 word programs (Flash_AMS_state_write), about 36 ms.
// - Rest of the boot: Motion_control_init to the loop. ADC_DMA_wait_full (up to 2 s if the ADC
//   does not fill) feeds in its loop. From its last feed, 16 x 2 ms ADC samples and the AS5600 and
//   PWM init to the first MOTOR_get_dir feed, or to the loop if no direction test runs: about 40 ms.
//   MOTOR_get_dir (up to 201 x 10 ms at 1000 PWM) feeds once per step, and after it
//   Motion_control_save (erase + program, about 18 ms) and the bus init run.
// - Recalibration (the 5 s buffer hold is timed by the loop, not a wait): blink_all_blue_3s feeds
//   every 20 ms, then Flash_NVM_full_clear (4 KB erase, 16 ms) and NVIC_SystemReset.
// - Not covered because the watchdog is not running yet: the first-boot calibration (up to 30 s
//   per step, interactive), its blink and averaging loops, the fallback flashes and the ADC init
//   waits. No motor is driven and the bus is not set up until after them.
#define WDG_FEED_GAP_MAX_MS 100u

// The shortest timeout (LSI at WDG_LSI_MAX_HZ) must be at least WDG_FEED_MARGIN times
// WDG_FEED_GAP_MAX_MS, and the longest (LSI at WDG_LSI_MIN_HZ) at most WDG_TIMEOUT_MAX_MS: that is
// how long a hang, or the trap handler, may wait for the reset.
#define WDG_FEED_MARGIN 5u
#define WDG_TIMEOUT_MAX_MS 2000u

#define WDG_PR_MAX 6u       // /256
#define WDG_RLR_MAX 0x0FFFu // 12-bit reload

typedef struct
{
    uint8_t  pr;  // PSCR code: divide by 4 << pr
    uint16_t rlr; // RLDR
} wdg_iwdg_cfg;

static inline WDG_CONSTEXPR uint32_t wdg_div(uint32_t pr)
{
    return 4u << pr;
}

// PSCR/RLDR for timeout_ms at lsi_hz: the smallest prescaler whose reload fits 12 bits (the finest
// steps), the reload rounded to the nearest step and at least 2, so the shortest timeout is not 0.
// Past the longest timeout (/256 and 0xFFF, 26.2 s at 40 kHz), that longest one.
static inline WDG_CONSTEXPR wdg_iwdg_cfg wdg_iwdg_config(uint32_t timeout_ms, uint32_t lsi_hz)
{
    const uint64_t cycles = (uint64_t)timeout_ms * lsi_hz / 1000u;
    for (uint32_t pr = 0u; pr <= WDG_PR_MAX; pr++)
    {
        const uint64_t div = wdg_div(pr);
        uint64_t rlr = (cycles + div / 2u) / div;
        if (rlr <= WDG_RLR_MAX)
        {
            if (rlr < 2u) rlr = 2u;
            const wdg_iwdg_cfg c = {(uint8_t)pr, (uint16_t)rlr};
            return c;
        }
    }
    const wdg_iwdg_cfg c = {(uint8_t)WDG_PR_MAX, (uint16_t)WDG_RLR_MAX};
    return c;
}

// Earliest reset after a feed, in us (rounded down): RLDR - 1 prescaler periods at lsi_max_hz.
static inline WDG_CONSTEXPR uint32_t wdg_timeout_min_us(wdg_iwdg_cfg c, uint32_t lsi_max_hz)
{
    return (uint32_t)((uint64_t)(c.rlr - 1u) * wdg_div(c.pr) * 1000000u / lsi_max_hz);
}

// Latest reset after a feed, in us (rounded up): RLDR periods at lsi_min_hz, plus one more for the
// few LSI cycles the feed takes to reach the counter.
static inline WDG_CONSTEXPR uint32_t wdg_timeout_max_us(wdg_iwdg_cfg c, uint32_t lsi_min_hz)
{
    return (uint32_t)(((uint64_t)(c.rlr + 1u) * wdg_div(c.pr) * 1000000u + lsi_min_hz - 1u) / lsi_min_hz);
}

// Nominal timeout, in us (rounded to nearest): RLDR periods at lsi_hz.
static inline WDG_CONSTEXPR uint32_t wdg_timeout_nom_us(wdg_iwdg_cfg c, uint32_t lsi_hz)
{
    return (uint32_t)(((uint64_t)c.rlr * wdg_div(c.pr) * 1000000u + lsi_hz / 2u) / lsi_hz);
}

// The configuration is safe over the whole LSI range (see WDG_FEED_MARGIN).
static inline WDG_CONSTEXPR bool wdg_cfg_safe(wdg_iwdg_cfg c, uint32_t lsi_min_hz, uint32_t lsi_max_hz)
{
    return (wdg_timeout_min_us(c, lsi_max_hz) >= WDG_FEED_MARGIN * WDG_FEED_GAP_MAX_MS * 1000u) &&
           (wdg_timeout_max_us(c, lsi_min_hz) <= WDG_TIMEOUT_MAX_MS * 1000u);
}

// ===== reset cause =====
// RCC_RSTSCKR reset flags (the RCC_*RSTF bits of ch32v20x.h; watchdog.cpp checks they match).
#define WDG_RSTF_PIN  0x04000000u // NRST pin
#define WDG_RSTF_POR  0x08000000u // power-on / power-down
#define WDG_RSTF_SFT  0x10000000u // software (NVIC_SystemReset)
#define WDG_RSTF_IWDG 0x20000000u // independent watchdog
#define WDG_RSTF_WWDG 0x40000000u // window watchdog
#define WDG_RSTF_LPWR 0x80000000u // low-power

typedef enum
{
    WDG_RESET_NONE = 0,  // no flag set
    WDG_RESET_PIN,       // NRST pin (a WCH-Link, for instance)
    WDG_RESET_POWER_ON,  // power-on
    WDG_RESET_SOFTWARE,  // NVIC_SystemReset (the recalibration reboot)
    WDG_RESET_LOW_POWER, // low-power reset (this firmware never enters Standby or Stop)
    WDG_RESET_WWDG,      // window watchdog (disabled by main)
    WDG_RESET_IWDG,      // independent watchdog: a hang, or a trap (see watchdog.cpp)
} wdg_reset_cause;

// The cause of the last reset. The flags add up until they are cleared (RMVF, which main does at
// every boot) or the power is cycled, which clears all but POR (and PIN). So any other flag is newer
// than POR. An internal reset can set PINRSTF too (on the STM32F1, whose RCC this copies, it drives
// NRST low), so PIN counts only on its own. The watchdog, the one the firmware reports, comes first
// if the flags of several resets add up (boots before this firmware cleared them).
static inline WDG_CONSTEXPR wdg_reset_cause wdg_reset_cause_decode(uint32_t rstsckr)
{
    return (rstsckr & WDG_RSTF_IWDG) ? WDG_RESET_IWDG
         : (rstsckr & WDG_RSTF_WWDG) ? WDG_RESET_WWDG
         : (rstsckr & WDG_RSTF_LPWR) ? WDG_RESET_LOW_POWER
         : (rstsckr & WDG_RSTF_SFT)  ? WDG_RESET_SOFTWARE
         : (rstsckr & WDG_RSTF_POR)  ? WDG_RESET_POWER_ON
         : (rstsckr & WDG_RSTF_PIN)  ? WDG_RESET_PIN
         : WDG_RESET_NONE;
}

// After a watchdog reset the SYS LED flashes magenta WDG_BOOT_FLASHES times, WDG_BOOT_FLASH_MS on
// and WDG_BOOT_FLASH_MS off, before the boot goes on (main.cpp): 420 ms, under the 0.5 s allowed.
#define WDG_BOOT_FLASHES 3u
#define WDG_BOOT_FLASH_MS 70u
#define WDG_BOOT_FLASH_MAX_MS 500u

// ===== fail-safe =====
// Timer compare the trap handler writes to both channels of every motor: what
// Motion_control_set_PWM(ch, 0) writes for a stopped motor. It is above the period (TIM_Period 999
// in MC_PWM_init), so PWM mode 1 keeps both bridge inputs high: the motor brakes.
#define WDG_FAILSAFE_PWM_COMPARE 1000u
