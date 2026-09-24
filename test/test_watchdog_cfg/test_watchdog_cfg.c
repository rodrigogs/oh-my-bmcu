// Host tests for src/watchdog_cfg.h: the IWDG prescaler/reload that watchdog_start() programs, the
// shortest and longest timeout over the datasheet's LSI range (25-60 kHz) against the feed gap and
// the fail-safe budget, the reset-cause decoder main.cpp runs at boot, the length of the watchdog
// flash, and that the trap handler's motor state is the one the firmware uses for a stopped motor.
//
// watchdog.cpp and Motion_control.cpp need the CH32 SDK and are not built on the host. The motor
// stop, the timer setup it relies on and RGB_update's throttle are verbatim copies
// (scripts/check_test_copies.py keeps them in sync with src/). The RCC_RSTSCKR flag values below
// are the RCC_*RSTF values of ch32v20x.h; watchdog.cpp also checks the header against them at
// build time.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unity.h>

#include "watchdog_cfg.h"

// ch32v20x.h, RCC_RSTSCKR
#define SDK_LSION    0x00000001u
#define SDK_LSIRDY   0x00000002u
#define SDK_RMVF     0x01000000u
#define SDK_PINRSTF  0x04000000u
#define SDK_PORRSTF  0x08000000u
#define SDK_SFTRSTF  0x10000000u
#define SDK_IWDGRSTF 0x20000000u
#define SDK_WWDGRSTF 0x40000000u
#define SDK_LPWRRSTF 0x80000000u

void setUp(void) {}
void tearDown(void) {}

// ---- timing ----

static uint64_t cycles_of(uint32_t timeout_ms, uint32_t lsi_hz)
{
    return (uint64_t)timeout_ms * lsi_hz / 1000u;
}

static void test_one_second_is_div16_reload_2500(void)
{
    const wdg_iwdg_cfg c = wdg_iwdg_config(WDG_TIMEOUT_MS, WDG_LSI_NOM_HZ);

    TEST_ASSERT_EQUAL_UINT32(1000u, WDG_TIMEOUT_MS);
    TEST_ASSERT_EQUAL_UINT8(2u, c.pr); // /16: /4 and /8 cannot reach 1 s in 12 bits
    TEST_ASSERT_EQUAL_UINT32(16u, wdg_div(c.pr));
    TEST_ASSERT_EQUAL_UINT16(2500u, c.rlr);
    TEST_ASSERT_EQUAL_UINT32(1000000u, wdg_timeout_nom_us(c, WDG_LSI_NOM_HZ));
    TEST_ASSERT_EQUAL_UINT32(1025641u, wdg_timeout_nom_us(c, 39000u)); // datasheet typ
}

static void test_datasheet_lsi_range(void)
{
    // CH32V203 datasheet V3.0, table 4-14 (V203C8T6): 25 / 39 / 60 kHz. 40 kHz: RM and WCH example.
    TEST_ASSERT_EQUAL_UINT32(25000u, WDG_LSI_MIN_HZ);
    TEST_ASSERT_EQUAL_UINT32(40000u, WDG_LSI_NOM_HZ);
    TEST_ASSERT_EQUAL_UINT32(60000u, WDG_LSI_MAX_HZ);
    TEST_ASSERT_TRUE(WDG_LSI_MIN_HZ < 39000u && 39000u < WDG_LSI_NOM_HZ && WDG_LSI_NOM_HZ < WDG_LSI_MAX_HZ);
    // V203RBT6: 25 / 32 / 45 kHz, inside the same bounds
    TEST_ASSERT_TRUE(WDG_LSI_MIN_HZ <= 25000u && 45000u <= WDG_LSI_MAX_HZ);
}

static void test_shortest_and_longest_timeout_of_the_firmware(void)
{
    const wdg_iwdg_cfg c = wdg_iwdg_config(WDG_TIMEOUT_MS, WDG_LSI_NOM_HZ);
    const uint32_t tmin = wdg_timeout_min_us(c, WDG_LSI_MAX_HZ);
    const uint32_t tmax = wdg_timeout_max_us(c, WDG_LSI_MIN_HZ);

    // 2499 x 16 / 60 kHz and 2501 x 16 / 25 kHz
    TEST_ASSERT_EQUAL_UINT32(666400u, tmin);
    TEST_ASSERT_EQUAL_UINT32(1600640u, tmax);

    TEST_ASSERT_TRUE(tmin >= WDG_FEED_MARGIN * WDG_FEED_GAP_MAX_MS * 1000u);
    TEST_ASSERT_TRUE(tmin >= 6u * WDG_FEED_GAP_MAX_MS * 1000u); // 6.6x the 100 ms gap budget
    TEST_ASSERT_TRUE(tmax <= WDG_TIMEOUT_MAX_MS * 1000u);
    TEST_ASSERT_TRUE(wdg_cfg_safe(c, WDG_LSI_MIN_HZ, WDG_LSI_MAX_HZ));

    // Every LSI frequency in the range gives a timeout inside [tmin, tmax].
    for (uint32_t hz = WDG_LSI_MIN_HZ; hz <= WDG_LSI_MAX_HZ; hz += 100u)
    {
        TEST_ASSERT_TRUE(wdg_timeout_min_us(c, hz) >= tmin);
        TEST_ASSERT_TRUE(wdg_timeout_max_us(c, hz) <= tmax);
        TEST_ASSERT_TRUE(wdg_timeout_min_us(c, hz) < wdg_timeout_nom_us(c, hz));
        TEST_ASSERT_TRUE(wdg_timeout_nom_us(c, hz) < wdg_timeout_max_us(c, hz));
    }
}

static void test_bounds_count_the_prescaler_phase_and_the_feed_sync(void)
{
    // /4, reload 10, at 40 kHz: 10 periods of 100 us nominal; 9 at the earliest (the first step
    // can follow the feed at once), 11 at the latest (one more for the feed reaching the counter).
    const wdg_iwdg_cfg c = {0u, 10u};
    TEST_ASSERT_EQUAL_UINT32(1000u, wdg_timeout_nom_us(c, 40000u));
    TEST_ASSERT_EQUAL_UINT32(900u, wdg_timeout_min_us(c, 40000u));
    TEST_ASSERT_EQUAL_UINT32(1100u, wdg_timeout_max_us(c, 40000u));

    // Rounding: the earliest rounds down, the latest up. /4, reload 4 at 60 kHz: 3 x 66.67 us and
    // 5 x 66.67 us.
    const wdg_iwdg_cfg d = {0u, 4u};
    TEST_ASSERT_EQUAL_UINT32(200u, wdg_timeout_min_us(d, 60000u));
    TEST_ASSERT_EQUAL_UINT32(334u, wdg_timeout_max_us(d, 60000u));

    // The longest configuration does not overflow: 4096 x 256 / 25 kHz = 41.9 s.
    const wdg_iwdg_cfg e = {WDG_PR_MAX, WDG_RLR_MAX};
    TEST_ASSERT_EQUAL_UINT32(41943040u, wdg_timeout_max_us(e, 25000u));
    TEST_ASSERT_EQUAL_UINT32(17467733u, wdg_timeout_min_us(e, 60000u));
}

static void test_config_takes_the_smallest_prescaler_and_the_nearest_reload(void)
{
    const uint32_t lsis[3] = {WDG_LSI_MIN_HZ, WDG_LSI_NOM_HZ, WDG_LSI_MAX_HZ};

    for (uint32_t l = 0u; l < 3u; l++)
    {
        const uint32_t hz = lsis[l];
        const uint64_t longest = (uint64_t)WDG_RLR_MAX * wdg_div(WDG_PR_MAX) * 1000u / hz; // ms

        for (uint32_t ms = 1u; ms <= (uint32_t)longest; ms++)
        {
            const wdg_iwdg_cfg c = wdg_iwdg_config(ms, hz);
            const uint64_t cyc = cycles_of(ms, hz);
            const uint64_t div = wdg_div(c.pr);

            TEST_ASSERT_TRUE(c.pr <= WDG_PR_MAX);
            TEST_ASSERT_TRUE(c.rlr >= 2u && c.rlr <= WDG_RLR_MAX);

            // No finer prescaler fits: its nearest reload is over 12 bits.
            if (c.pr > 0u)
            {
                const uint64_t fdiv = wdg_div(c.pr - 1u);
                TEST_ASSERT_TRUE((cyc + fdiv / 2u) / fdiv > WDG_RLR_MAX);
            }

            // The nearest reload: within half a step of the timeout, unless raised to 2.
            const uint64_t got = (uint64_t)c.rlr * div;
            const uint64_t err = (got > cyc) ? (got - cyc) : (cyc - got);
            if (c.rlr > 2u)
                TEST_ASSERT_TRUE(2u * err <= div);
            else
                TEST_ASSERT_TRUE(cyc <= 2u * div + div / 2u);
        }
    }
}

static void test_config_limits(void)
{
    wdg_iwdg_cfg c = wdg_iwdg_config(0u, WDG_LSI_NOM_HZ);
    TEST_ASSERT_EQUAL_UINT8(0u, c.pr);
    TEST_ASSERT_EQUAL_UINT16(2u, c.rlr); // never 0 or 1: the shortest timeout stays above 0
    TEST_ASSERT_TRUE(wdg_timeout_min_us(c, WDG_LSI_MAX_HZ) > 0u);

    c = wdg_iwdg_config(1u, WDG_LSI_NOM_HZ); // 40 cycles: /4, 10
    TEST_ASSERT_EQUAL_UINT8(0u, c.pr);
    TEST_ASSERT_EQUAL_UINT16(10u, c.rlr);

    c = wdg_iwdg_config(409u, WDG_LSI_NOM_HZ); // 16360 cycles: /4, 4090
    TEST_ASSERT_EQUAL_UINT8(0u, c.pr);
    TEST_ASSERT_EQUAL_UINT16(4090u, c.rlr);

    c = wdg_iwdg_config(410u, WDG_LSI_NOM_HZ); // 16400 cycles: 4100 > 4095, so /8, 2050
    TEST_ASSERT_EQUAL_UINT8(1u, c.pr);
    TEST_ASSERT_EQUAL_UINT16(2050u, c.rlr);

    c = wdg_iwdg_config(26208u, WDG_LSI_NOM_HZ); // 1048320 cycles = 4095 x 256
    TEST_ASSERT_EQUAL_UINT8(6u, c.pr);
    TEST_ASSERT_EQUAL_UINT16(4095u, c.rlr);

    c = wdg_iwdg_config(26207u, WDG_LSI_NOM_HZ); // 1048280 cycles: 4094.8 x 256 -> 4095
    TEST_ASSERT_EQUAL_UINT8(6u, c.pr);
    TEST_ASSERT_EQUAL_UINT16(4095u, c.rlr);

    c = wdg_iwdg_config(60000u, WDG_LSI_NOM_HZ); // past the longest: the longest
    TEST_ASSERT_EQUAL_UINT8(WDG_PR_MAX, c.pr);
    TEST_ASSERT_EQUAL_UINT16(WDG_RLR_MAX, c.rlr);

    c = wdg_iwdg_config(0xFFFFFFFFu, WDG_LSI_MAX_HZ); // no overflow in the cycle count
    TEST_ASSERT_EQUAL_UINT8(WDG_PR_MAX, c.pr);
    TEST_ASSERT_EQUAL_UINT16(WDG_RLR_MAX, c.rlr);
}

static void test_safe_window_needs_the_lsi_tolerance(void)
{
    // Over 25-60 kHz only nominal timeouts of about 0.75-1.25 s pass: the shortest must stay
    // >= 500 ms (5 x 100 ms) at 60 kHz, the longest <= 2 s at 25 kHz. 1 s is in the middle.
    uint32_t lo = 0u, hi = 0u;
    for (uint32_t ms = 100u; ms <= 5000u; ms++)
    {
        if (!wdg_cfg_safe(wdg_iwdg_config(ms, WDG_LSI_NOM_HZ), WDG_LSI_MIN_HZ, WDG_LSI_MAX_HZ)) continue;
        if (!lo) lo = ms;
        hi = ms;
    }
    for (uint32_t ms = lo; ms <= hi; ms++) // one window, no holes
        TEST_ASSERT_TRUE(wdg_cfg_safe(wdg_iwdg_config(ms, WDG_LSI_NOM_HZ), WDG_LSI_MIN_HZ, WDG_LSI_MAX_HZ));

    char msg[96];
    snprintf(msg, sizeof(msg), "safe nominal timeouts: %u..%u ms", (unsigned)lo, (unsigned)hi);
    TEST_MESSAGE(msg);

    TEST_ASSERT_EQUAL_UINT32(751u, lo);
    TEST_ASSERT_EQUAL_UINT32(1249u, hi);
    TEST_ASSERT_TRUE(lo < WDG_TIMEOUT_MS && WDG_TIMEOUT_MS < hi);

    // Checked at the nominal LSI alone, 700 ms and 1.5 s would pass. Over the range, 700 ms can
    // reset after 466 ms (60 kHz) and 1.5 s only after 2.4 s (25 kHz).
    TEST_ASSERT_TRUE(wdg_cfg_safe(wdg_iwdg_config(700u, WDG_LSI_NOM_HZ), WDG_LSI_NOM_HZ, WDG_LSI_NOM_HZ));
    TEST_ASSERT_FALSE(wdg_cfg_safe(wdg_iwdg_config(700u, WDG_LSI_NOM_HZ), WDG_LSI_MIN_HZ, WDG_LSI_MAX_HZ));
    TEST_ASSERT_TRUE(wdg_cfg_safe(wdg_iwdg_config(1500u, WDG_LSI_NOM_HZ), WDG_LSI_NOM_HZ, WDG_LSI_NOM_HZ));
    TEST_ASSERT_FALSE(wdg_cfg_safe(wdg_iwdg_config(1500u, WDG_LSI_NOM_HZ), WDG_LSI_MIN_HZ, WDG_LSI_MAX_HZ));

    // The IWDG reset values (/4, 0xFFF), which the trap handler starts with if the trap comes before
    // watchdog_start, and which the first period may still use: 0.27-0.66 s.
    const wdg_iwdg_cfg r = {0u, 0x0FFFu};
    TEST_ASSERT_EQUAL_UINT32(272933u, wdg_timeout_min_us(r, WDG_LSI_MAX_HZ));
    TEST_ASSERT_EQUAL_UINT32(655360u, wdg_timeout_max_us(r, WDG_LSI_MIN_HZ));
    TEST_ASSERT_TRUE(wdg_timeout_min_us(r, WDG_LSI_MAX_HZ) > 2u * WDG_FEED_GAP_MAX_MS * 1000u);
}

// ---- reset cause ----

static void test_reset_flags_are_rcc_rstsckr(void)
{
    TEST_ASSERT_EQUAL_HEX32(SDK_PINRSTF, WDG_RSTF_PIN);
    TEST_ASSERT_EQUAL_HEX32(SDK_PORRSTF, WDG_RSTF_POR);
    TEST_ASSERT_EQUAL_HEX32(SDK_SFTRSTF, WDG_RSTF_SFT);
    TEST_ASSERT_EQUAL_HEX32(SDK_IWDGRSTF, WDG_RSTF_IWDG);
    TEST_ASSERT_EQUAL_HEX32(SDK_WWDGRSTF, WDG_RSTF_WWDG);
    TEST_ASSERT_EQUAL_HEX32(SDK_LPWRRSTF, WDG_RSTF_LPWR);
}

static void test_each_reset_flag_alone(void)
{
    TEST_ASSERT_EQUAL_INT(WDG_RESET_NONE, wdg_reset_cause_decode(0u));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_PIN, wdg_reset_cause_decode(SDK_PINRSTF));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_POWER_ON, wdg_reset_cause_decode(SDK_PORRSTF));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_SOFTWARE, wdg_reset_cause_decode(SDK_SFTRSTF));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_IWDG, wdg_reset_cause_decode(SDK_IWDGRSTF));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_WWDG, wdg_reset_cause_decode(SDK_WWDGRSTF));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_LOW_POWER, wdg_reset_cause_decode(SDK_LPWRRSTF));
}

static void test_pin_flag_comes_with_every_reset(void)
{
    // An internal reset can set PINRSTF too; only on its own is it the NRST pin.
    TEST_ASSERT_EQUAL_INT(WDG_RESET_POWER_ON, wdg_reset_cause_decode(SDK_PORRSTF | SDK_PINRSTF)); // RSTSCKR reset value
    TEST_ASSERT_EQUAL_INT(WDG_RESET_SOFTWARE, wdg_reset_cause_decode(SDK_SFTRSTF | SDK_PINRSTF));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_IWDG, wdg_reset_cause_decode(SDK_IWDGRSTF | SDK_PINRSTF));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_WWDG, wdg_reset_cause_decode(SDK_WWDGRSTF | SDK_PINRSTF));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_LOW_POWER, wdg_reset_cause_decode(SDK_LPWRRSTF | SDK_PINRSTF));
}

static void test_flags_of_several_resets(void)
{
    // Not cleared since power-on (the boots before this firmware): the newer reset wins over POR,
    // and the watchdog over everything.
    TEST_ASSERT_EQUAL_INT(WDG_RESET_SOFTWARE, wdg_reset_cause_decode(SDK_PORRSTF | SDK_PINRSTF | SDK_SFTRSTF));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_IWDG, wdg_reset_cause_decode(SDK_PORRSTF | SDK_PINRSTF | SDK_IWDGRSTF));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_IWDG, wdg_reset_cause_decode(SDK_SFTRSTF | SDK_IWDGRSTF));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_IWDG, wdg_reset_cause_decode(SDK_WWDGRSTF | SDK_IWDGRSTF | SDK_LPWRRSTF));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_IWDG, wdg_reset_cause_decode(0xFC000000u));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_WWDG, wdg_reset_cause_decode(SDK_WWDGRSTF | SDK_LPWRRSTF | SDK_SFTRSTF));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_LOW_POWER, wdg_reset_cause_decode(SDK_LPWRRSTF | SDK_SFTRSTF | SDK_PORRSTF));
}

static void test_other_rstsckr_bits_are_ignored(void)
{
    // LSION/LSIRDY (the IWDG turns the LSI on) and the write-only RMVF are not reset causes.
    TEST_ASSERT_EQUAL_INT(WDG_RESET_NONE, wdg_reset_cause_decode(SDK_LSION | SDK_LSIRDY));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_NONE, wdg_reset_cause_decode(SDK_RMVF));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_NONE, wdg_reset_cause_decode(0x03FFFFFFu));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_IWDG, wdg_reset_cause_decode(SDK_IWDGRSTF | SDK_PINRSTF | SDK_LSION | SDK_LSIRDY));
    TEST_ASSERT_EQUAL_INT(WDG_RESET_PIN, wdg_reset_cause_decode(SDK_PINRSTF | 0x03FFFFFFu));
}

static void test_only_the_watchdog_flag_shows_the_flash(void)
{
    // main.cpp flashes for WDG_RESET_IWDG only: every flag combination with IWDGRSTF, and none
    // without it.
    for (uint32_t m = 0u; m < 64u; m++)
    {
        const uint32_t f = m << 26;
        const bool flash = (wdg_reset_cause_decode(f) == WDG_RESET_IWDG);
        TEST_ASSERT_EQUAL((f & SDK_IWDGRSTF) != 0u, flash);
    }
}

// ---- watchdog flash ----

static uint32_t time_hw_tpms = 1u; // ticks per ms: the gap below comes out in ms

static uint32_t rgb_update_min_gap_ms(void)
{
// ---- main.cpp at this commit: RGB_update's throttle gap, verbatim ----
    uint32_t min_gap = time_hw_tpms * 10u;
    if (!min_gap) min_gap = 1u;
// ---- end of the main.cpp copy ----
    return min_gap;
}

static void test_watchdog_flash_is_short_and_visible(void)
{
    const uint32_t total = 2u * WDG_BOOT_FLASHES * WDG_BOOT_FLASH_MS;
    TEST_ASSERT_EQUAL_UINT32(420u, total);
    TEST_ASSERT_TRUE(total <= WDG_BOOT_FLASH_MAX_MS);
    TEST_ASSERT_EQUAL_UINT32(500u, WDG_BOOT_FLASH_MAX_MS);
    TEST_ASSERT_TRUE(WDG_BOOT_FLASHES >= 2u); // a pattern, not a single blip
    // RGB_update skips a frame less than its throttle gap after the previous one.
    TEST_ASSERT_TRUE(WDG_BOOT_FLASH_MS >= rgb_update_min_gap_ms());
    TEST_ASSERT_TRUE(WDG_BOOT_FLASH_MS >= 50u); // long enough to see
}

// ---- fail-safe motor state ----

typedef struct
{
    uint16_t TIM_Period;
    uint16_t TIM_Prescaler;
    uint16_t TIM_ClockDivision;
    uint16_t TIM_CounterMode;
} tim_base_t;

typedef struct
{
    uint16_t TIM_OCMode;
    uint16_t TIM_OutputState;
    uint16_t TIM_Pulse;
    uint16_t TIM_OCPolarity;
} tim_oc_t;

// ch32v20x_tim.h
#define TIM_CounterMode_Up     ((uint16_t)0x0000)
#define TIM_OCMode_PWM1        ((uint16_t)0x0060)
#define TIM_OutputState_Enable ((uint16_t)0x0001)
#define TIM_OCPolarity_High    ((uint16_t)0x0000)

static tim_base_t mc_pwm_init_base(void)
{
    tim_base_t TIM_TimeBaseStructure;
// ---- Motion_control.cpp at this commit: MC_PWM_init time base, verbatim ----
    TIM_TimeBaseStructure.TIM_Period        = 999;
    TIM_TimeBaseStructure.TIM_Prescaler     = 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
// ---- end of the Motion_control.cpp copy ----
    return TIM_TimeBaseStructure;
}

static tim_oc_t mc_pwm_init_oc(void)
{
    tim_oc_t TIM_OCInitStructure;
// ---- Motion_control.cpp at this commit: MC_PWM_init output compare, verbatim ----
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
// ---- end of the Motion_control.cpp copy ----
    return TIM_OCInitStructure;
}

// The two compares Motion_control_set_PWM writes for one channel.
static void set_pwm_compares(int PWM, uint16_t *o1, uint16_t *o2)
{
// ---- Motion_control.cpp at this commit: Motion_control_set_PWM compares, verbatim ----
    uint16_t set1 = 0, set2 = 0;

    if (PWM > 0)       set1 = (uint16_t)PWM;
    else if (PWM < 0)  set2 = (uint16_t)(-PWM);
    else { set1 = 1000; set2 = 1000; }
// ---- end of the Motion_control.cpp copy ----
    *o1 = set1;
    *o2 = set2;
}

// PWM mode 1, counting up, active high: the output is high while CNT < CCR.
static uint32_t high_counts(uint16_t ccr, const tim_base_t *b)
{
    uint32_t n = 0u;
    for (uint32_t cnt = 0u; cnt <= b->TIM_Period; cnt++)
        if (cnt < ccr) n++;
    return n;
}

static void test_failsafe_compare_is_the_firmwares_stop(void)
{
    uint16_t s1 = 0u, s2 = 0u;
    set_pwm_compares(0, &s1, &s2);
    TEST_ASSERT_EQUAL_UINT16(WDG_FAILSAFE_PWM_COMPARE, s1);
    TEST_ASSERT_EQUAL_UINT16(WDG_FAILSAFE_PWM_COMPARE, s2);

    // Driving values differ from it: one side at the duty, the other low.
    set_pwm_compares(1000, &s1, &s2);
    TEST_ASSERT_TRUE(s1 == 1000u && s2 == 0u);
    set_pwm_compares(-900, &s1, &s2);
    TEST_ASSERT_TRUE(s1 == 0u && s2 == 900u);
}

static void test_failsafe_compare_keeps_both_inputs_high(void)
{
    const tim_base_t b = mc_pwm_init_base();
    const tim_oc_t oc = mc_pwm_init_oc();

    TEST_ASSERT_EQUAL_UINT16(TIM_CounterMode_Up, b.TIM_CounterMode);
    TEST_ASSERT_EQUAL_UINT16(TIM_OCMode_PWM1, oc.TIM_OCMode);
    TEST_ASSERT_EQUAL_UINT16(TIM_OCPolarity_High, oc.TIM_OCPolarity);
    TEST_ASSERT_TRUE(WDG_FAILSAFE_PWM_COMPARE > b.TIM_Period);

    // High for the whole period on both inputs of the bridge: brake, as a stopped motor.
    TEST_ASSERT_EQUAL_UINT32((uint32_t)b.TIM_Period + 1u, high_counts(WDG_FAILSAFE_PWM_COMPARE, &b));
    // Not 0 (both low: coast, which the firmware never uses for a stop) nor a duty.
    TEST_ASSERT_EQUAL_UINT32(0u, high_counts(0u, &b));
    TEST_ASSERT_TRUE(high_counts((uint16_t)b.TIM_Period, &b) < (uint32_t)b.TIM_Period + 1u);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_one_second_is_div16_reload_2500);
    RUN_TEST(test_datasheet_lsi_range);
    RUN_TEST(test_shortest_and_longest_timeout_of_the_firmware);
    RUN_TEST(test_bounds_count_the_prescaler_phase_and_the_feed_sync);
    RUN_TEST(test_config_takes_the_smallest_prescaler_and_the_nearest_reload);
    RUN_TEST(test_config_limits);
    RUN_TEST(test_safe_window_needs_the_lsi_tolerance);
    RUN_TEST(test_reset_flags_are_rcc_rstsckr);
    RUN_TEST(test_each_reset_flag_alone);
    RUN_TEST(test_pin_flag_comes_with_every_reset);
    RUN_TEST(test_flags_of_several_resets);
    RUN_TEST(test_other_rstsckr_bits_are_ignored);
    RUN_TEST(test_only_the_watchdog_flag_shows_the_flash);
    RUN_TEST(test_watchdog_flash_is_short_and_visible);
    RUN_TEST(test_failsafe_compare_is_the_firmwares_stop);
    RUN_TEST(test_failsafe_compare_keeps_both_inputs_high);
    return UNITY_END();
}
