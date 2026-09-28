#include "watchdog.h"

#include "ch32v20x_gpio.h"
#include "ch32v20x_rcc.h"
#include <stddef.h>

static_assert(WDG_RSTF_PIN == RCC_PINRSTF && WDG_RSTF_POR == RCC_PORRSTF && WDG_RSTF_SFT == RCC_SFTRSTF &&
                  WDG_RSTF_IWDG == RCC_IWDGRSTF && WDG_RSTF_WWDG == RCC_WWDGRSTF && WDG_RSTF_LPWR == RCC_LPWRRSTF,
              "watchdog_cfg.h reset flags differ from RCC_RSTSCKR");
static_assert(BLACKBOX_RCC_PWREN == RCC_PWREN && BLACKBOX_RCC_BKPEN == RCC_BKPEN && BLACKBOX_PWR_DBP == PWR_CTLR_DBP,
              "blackbox.h backup-domain bits differ from RCC_APB1PCENR and PWR_CTLR");

// /16 and 2500 at 40 kHz: 1.000 s nominal, 0.666 s at 60 kHz, 1.601 s at 25 kHz.
static constexpr wdg_iwdg_cfg kIwdg = wdg_iwdg_config(WDG_TIMEOUT_MS, WDG_LSI_NOM_HZ);
static_assert(wdg_cfg_safe(kIwdg, WDG_LSI_MIN_HZ, WDG_LSI_MAX_HZ),
              "IWDG timeout too short for the longest feed gap, or too long for the fail-safe");

uint8_t g_watchdog_on = 0u;

void watchdog_start(void)
{
    // WCH's IWDG_Feed_Init sequence. The SDK's IWDG_Enable then waits for LSIRDY with no limit;
    // that is left out. Until the LSI runs (datasheet table 4-14: 5 ms start-up with the LSE off),
    // the counter does not count and the PSCR/RLDR writes wait for it (PVU/RVU). If the first
    // reload still used the reset values (/4, 0xFFF: at least 0.27 s), that is longer than any feed
    // gap, and the next feed loads 2500.
    IWDG->CTLR = 0x5555u; // unlock PSCR/RLDR
    IWDG->PSCR = kIwdg.pr;
    IWDG->RLDR = kIwdg.rlr;
    IWDG->CTLR = 0xAAAAu; // reload
    IWDG->CTLR = 0xCCCCu; // start (also turns the LSI on)
    g_watchdog_on = 1u;   // from now on watchdog_feed() writes the reload key
}

uint32_t watchdog_reset_flags_take(void)
{
    const uint32_t f = RCC->RSTSCKR;
    RCC->RSTSCKR |= RCC_RMVF; // as RCC_ClearFlag()
    return f;
}

// ===== fault record =====
static_assert(offsetof(BKP_TypeDef, DATAR1) == 4u && offsetof(BKP_TypeDef, DATAR10) == 4u + 4u * (BLACKBOX_WORDS - 1u),
              "blackbox_bkp() does not address BKP_DATAR1-10");

blackbox_t g_blackbox_last;
blackbox_pass_t g_blackbox_pass;

void blackbox_boot(uint32_t rstsckr)
{
    // CH32FV2x_V3xRM 4.2: the PWR and BKP clocks, then DBP to allow writes. Nothing turns them off.
    RCC->APB1PCENR |= RCC_PWREN | RCC_BKPEN; // as RCC_APB1PeriphClockCmd
    PWR->CTLR |= PWR_CTLR_DBP;               // as PWR_BackupAccessCmd

    uint16_t w[BLACKBOX_WORDS];
    for (uint32_t i = 0u; i < BLACKBOX_WORDS; i++) w[i] = *blackbox_bkp(i);
    blackbox_boot_words(w, rstsckr, &g_blackbox_last);
    for (uint32_t i = 0u; i < BLACKBOX_WORDS; i++) *blackbox_bkp(i) = w[i];
}

// ===== fail-safe trap handlers =====
// Without them a trap ran the SDK's weak default, an endless loop: TIM2/3/4 kept driving the motors
// at their last PWM, and a trap during a reply left DE high, the transceiver driving the bus, until
// the printer was power-cycled. Now the handler stops everything, records the trap (blackbox.h),
// then the IWDG resets the chip, so the next boot shows the watchdog and trap flashes (main.cpp).
//
// HardFault takes the fault exceptions (illegal instruction, misaligned or faulting load/store or
// fetch). NMI (only the HSE clock-security system, unused with HSI) and the breakpoint exception
// (GCC compiles __builtin_trap(), for instance on an isolated NULL dereference, to ebreak; none is
// in the image now) get the same handler, since their default loops too. ecall is never executed.
//
// Register accesses only, no call and no stack: the trap may come from a bad stack pointer.
// "WCH-Interrupt-fast" like WCH's own HardFault/NMI handlers (EVT ch32v20x_it.c): the hardware
// prologue (HPE, enabled by the startup code) saves the registers, so GCC emits no software
// save to the stack, and the handler never returns.
static inline __attribute__((always_inline)) void failsafe_stop(void)
{
    // IRQs off (MIE/MPIE, as __disable_irq), so the USART1 ISR cannot start another TX.
    __asm volatile("csrc 0x800, %0" ::"r"(0x88u) : "memory");

    // First the IWDG, which needs no clock enable: if a trap during boot came before watchdog_start
    // it starts now with its reset values (/4, 0xFFF: 0.27-0.66 s); if it runs this does nothing and
    // it resets 0.67-1.6 s after the last feed. Even if a write below faulted again, it resets.
    IWDG->CTLR = 0xCCCCu;

    // Every motor to the firmware's stop, both compares above the period (Motion_control_set_PWM).
    // CCR preload is on: UG loads the compares now instead of at the next update (14 us).
    TIM4->CH3CVR = WDG_FAILSAFE_PWM_COMPARE; // ch 0
    TIM4->CH4CVR = WDG_FAILSAFE_PWM_COMPARE;
    TIM4->CH1CVR = WDG_FAILSAFE_PWM_COMPARE; // ch 1
    TIM4->CH2CVR = WDG_FAILSAFE_PWM_COMPARE;
    TIM3->CH1CVR = WDG_FAILSAFE_PWM_COMPARE; // ch 2
    TIM3->CH2CVR = WDG_FAILSAFE_PWM_COMPARE;
    TIM2->CH1CVR = WDG_FAILSAFE_PWM_COMPARE; // ch 3
    TIM2->CH2CVR = WDG_FAILSAFE_PWM_COMPARE;
    TIM2->SWEVGR = TIM_UG;
    TIM3->SWEVGR = TIM_UG;
    TIM4->SWEVGR = TIM_UG;

    // DE = RX, as the TC interrupt does after a reply: the transceiver stops driving the bus.
    GPIOA->BCR = GPIO_Pin_12;

    // The fault record, after the stop: mepc and mcause, their check last, so a reset between the
    // writes reads as no trap. The phase and the longest pass stay as the run left them. Before
    // blackbox_boot turns the backup domain on (startup code, SystemInit, time_hw_init), the record
    // is skipped: two register reads, PWR_CTLR only once its clock is on.
    uint32_t mepc, mcause;
    __asm volatile("csrr %0, mepc" : "=r"(mepc));
    __asm volatile("csrr %0, mcause" : "=r"(mcause));
    const uint16_t pc_lo = (uint16_t)mepc;
    const uint16_t pc_hi = (uint16_t)(mepc >> 16);
    const uint16_t code = blackbox_mcause_code(mcause);
    if (blackbox_bkp_clocked(RCC->APB1PCENR) && blackbox_bkp_writable(PWR->CTLR))
    {
        *blackbox_bkp(BLACKBOX_W_MEPC_LO) = pc_lo;
        *blackbox_bkp(BLACKBOX_W_MEPC_HI) = pc_hi;
        *blackbox_bkp(BLACKBOX_W_MCAUSE) = code;
        *blackbox_bkp(BLACKBOX_W_TRAP_CHECK) = blackbox_trap_check(pc_lo, pc_hi, code);
    }

    for (;;)
        __asm volatile("" ::: "memory");
}

extern "C" void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
extern "C" void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
extern "C" void Break_Point_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void HardFault_Handler(void)
{
    failsafe_stop();
}

void NMI_Handler(void)
{
    failsafe_stop();
}

void Break_Point_Handler(void)
{
    failsafe_stop();
}
