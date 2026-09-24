#pragma once
// What the buffer calibration (MC_PULL_calibration.cpp) stores for each channel, how the run ends
// and what later boots show. Hardware-free, so it is tested on the host (test/test_mc_pull_cal_range).
//
// Each inserted buffer is calibrated in two steps: move it to one end and back (this step also
// learns the Hall polarity), then to the other end and back. A step counts only once the reading
// has left the idle centre by CAL_PRESS_DELTA_V (100 mV) and settled back; it gives up after 30 s.
// All values are normalised: MC_PULL_V_OFFSET puts the idle reading at 1.65 V and the polarity
// puts the first step's end below it.
//
// A step that timed out used to be stored as centre -/+ 50 mV (+/- 100 mV if both timed out) and
// the run still ended with the green blink. pull_v_to_percent_f() (Motion_control.cpp) then put
// 40 % (on_use jam latch) 10 mV below centre, inside the 20 mV the calibration itself accepts as
// "back at rest" (CAL_CENTER_EPS_V): random tangle pauses, and with every slot empty a 20 mV drift
// held for 5 s met the recalibration gesture (v <= MC_PULL_V_MIN + 0.03 V) and wiped the whole NVM.
// A missed second step put the load stops (85/90/95 %) 35-45 mV above centre instead.
//
// pull_v_to_percent_f() maps a side linearly: p % below 50 sits (50 - p) / 50 of the span below
// centre, p % above 50 sits (p - 50) / 50 of the span above it. A side that was not measured, or
// is too shallow to keep, gets a fallback that errs the safe way for what that side decides:
//  - Low side (buffer pulled: jam latch < 40 %; recalibration gesture <= 15 %, 100 mV below centre
//    or 30 mV above MC_PULL_V_MIN, held 5 s).
//    A captured end is kept only if it spans at least MC_PULL_CAL_MIN_LOW_SPAN_V (150 mV). A step
//    already accepts 100 mV, which puts the jam latch 20 mV below centre, on the edge of the rest
//    scatter, and lets the gesture's 15 % rule wipe the NVM 69 mV below centre, before its own
//    absolute 100 mV rule. From 150 mV the latch sits at least 30 mV (1.5x the scatter) below
//    centre and the 15 % rule (pct rounds, so < 15.5 %: 0.69 x span) needs at least 103 mV; 145 mV
//    is the least span that keeps it at 100 mV.
//    Otherwise (timed out or shallower) the low side is 1.00 V, the MC_PULL_V_MIN default (the
//    range held before a calibration is loaded) in every source release, V6..V10.5: a 650 mV span,
//    so falling back never narrows a side. The jam latch then needs a 130 mV pull (0.2 * 650 mV):
//    6.5x the 20 mV rest scatter and more than the 100 mV the firmware treats as a deliberate move.
//    The gesture is left to its absolute rule (v <= 1.65 - 0.10 V), since MC_PULL_V_MIN + 0.03 V
//    is then 1.03 V. A too wide low side only makes these need a longer pull; it never trips them
//    on noise. (The DM gesture load, < 10 %, then needs 520 mV and may be out of reach until
//    recalibrated.)
//  - High side (buffer pushed: load stops). A timed-out second step gives 1.75 V, i.e.
//    CAL_PRESS_DELTA_V above centre, the range pull_v_to_percent_f() and the boot already use for
//    an unknown or absent buffer. Every buffer the calibration can measure moves at least that
//    far, so the 95 % hard stop (+90 mV; P1S 97 %: +94 mV) still trips within its travel. The
//    MC_PULL_V_MAX default (2.00 V) would put it at +315 mV, which a short buffer may never reach:
//    loading would keep pushing into the end stop. A too narrow high side only stops the push
//    earlier (less force), never later, and its other triggers stay clear of the 20 mV rest
//    scatter (idle deadband 70 %: +40 mV, auto-unload and empty-slot pull 80 %: +60 mV).
//    Since that safe span is also the least a step captures, a captured high side is always kept:
//    replacing a shallow push by the fallback would only narrow it further.
//  - Polarity comes from the first step alone; a shallow one still moved at least 100 mV, 5x the
//    rest scatter. If it timed out (alone or with the second step) the magnet orientation was
//    never measured and stays +1, the stock orientation every release before V10.3 assumed. A
//    buffer with a reversed magnet then runs its pressure loop inverted until it is recalibrated;
//    its fallback flag makes it flash red at every boot.
//
// A channel with a fallback side is still saved, so later boots do not wait again for up to 60 s
// per buffer before the bus starts. Its bit is kept in the calibration record (rsv bits 4..7), the
// run ends with the fast red flash on it instead of the green blink, and every later boot repeats
// that flash briefly until the user recalibrates (the usual 5 s buffer hold, all slots empty).

#include <stdbool.h>
#include <stdint.h>

#define MC_PULL_CAL_PRESS_DELTA_V  0.10f  // smallest move a calibration step accepts
#define MC_PULL_CAL_MIN_LOW_SPAN_V 0.15f  // narrowest captured low side that is kept
#define MC_PULL_CAL_FALLBACK_V_MIN 1.00f  // low side of a buffer whose first step failed or was shallow
#define MC_PULL_CAL_FALLBACK_V_MAX 1.75f  // high side of a buffer whose second step timed out

typedef struct
{
    float  vmin;     // -> MC_PULL_V_MIN
    float  vmax;     // -> MC_PULL_V_MAX
    int8_t pol;      // -> MC_PULL_POLARITY
    bool   fallback; // a side is a fallback: calibration record, red end and boot flash
} mc_pull_cal_range_t;

// The first step's end is kept: the step did not time out and its end spans the low-side floor.
static inline bool mc_pull_cal_low_ok(float center_v, bool ok_min, float cap_min)
{
    return ok_min && ((center_v - cap_min) >= MC_PULL_CAL_MIN_LOW_SPAN_V);
}

// center_v: the channel's normalised idle reading. ok_min / cap_min / pol: result of the first
// step; ok_max / cap_max: result of the second step (capture_*_wait_release in the .cpp).
static inline mc_pull_cal_range_t mc_pull_cal_range(float center_v, bool ok_min, float cap_min, int8_t pol,
                                                    bool ok_max, float cap_max)
{
    const bool low_ok = mc_pull_cal_low_ok(center_v, ok_min, cap_min);

    mc_pull_cal_range_t r;
    r.vmin = low_ok ? cap_min : MC_PULL_CAL_FALLBACK_V_MIN;
    r.vmax = ok_max ? cap_max : MC_PULL_CAL_FALLBACK_V_MAX;
    // Only the first step measures the polarity. If it timed out (always so when both steps did)
    // the orientation is unknown and stays +1, what every release before V10.3 assumed; a reversed
    // magnet then needs a recalibration (see above; a hardware validation item).
    r.pol  = (ok_min && pol < 0) ? (int8_t)-1 : (int8_t)1;
    r.fallback = !(low_ok && ok_max);

    // Unchanged guards. A kept end is at least CAL_PRESS_DELTA_V from centre and the fallbacks
    // are wider than 50 mV, so they no longer change anything; they only keep the range sane.
    if (r.vmin > (center_v - 0.050f)) r.vmin = (center_v - 0.050f);
    if (r.vmax < (center_v + 0.050f)) r.vmax = (center_v + 0.050f);
    if (r.vmax <= r.vmin + 0.10f) { r.vmin = center_v - 0.10f; r.vmax = center_v + 0.10f; }

    return r;
}

// Channels of one calibration run, or of the record a later boot loads.
typedef struct
{
    uint8_t inserted_mask; // a buffer is connected
    uint8_t fallback_mask; // its saved range has a fallback side (rsv bits 4..7)
} mc_pull_cal_run_t;

// Everything the run keeps from channel ch's two steps: the range to store, and ch's bits in *run.
static inline mc_pull_cal_range_t mc_pull_cal_run_channel(mc_pull_cal_run_t *run, uint8_t ch, float center_v,
                                                          bool ok_min, float cap_min, int8_t pol,
                                                          bool ok_max, float cap_max)
{
    const uint8_t bit = (uint8_t)(1u << ch);
    const mc_pull_cal_range_t r = mc_pull_cal_range(center_v, ok_min, cap_min, pol, ok_max, cap_max);

    run->inserted_mask |= bit;
    if (r.fallback) run->fallback_mask |= bit;
    return r;
}

// A boot that loads the record: the buffers connected now and the saved fallback bits.
static inline mc_pull_cal_run_t mc_pull_cal_run_loaded(const bool inserted[4], uint8_t fallback_mask)
{
    mc_pull_cal_run_t run;
    run.inserted_mask = 0u;
    for (uint8_t ch = 0u; ch < 4u; ch++)
    {
        if (inserted[ch]) run.inserted_mask |= (uint8_t)(1u << ch);
    }
    run.fallback_mask = (uint8_t)(fallback_mask & 0x0Fu);
    return run;
}

// How a run ends, after the three diagnostic steps.
typedef enum
{
    MC_PULL_CAL_END_NONE = 0,    // only the fallback flash follows
    MC_PULL_CAL_END_GREEN,       // calibrated and saved: every flash write and every buffer succeeded
    MC_PULL_CAL_END_FLASH_ERROR, // a flash write failed: all red 260/260 ms, then any fallback flash
} mc_pull_cal_end_t;

static inline mc_pull_cal_end_t mc_pull_cal_end(bool flash_ok, const mc_pull_cal_run_t *run)
{
    if (!flash_ok) return MC_PULL_CAL_END_FLASH_ERROR;
    return (run->fallback_mask == 0u) ? MC_PULL_CAL_END_GREEN : MC_PULL_CAL_END_NONE;
}

// The fast red flash (60 ms on, 60 ms off) marks the buffers whose saved range is a fallback. It
// ends the run that stored them (20 flashes, 2.4 s; the calibrated buffers stay green) and repeats
// at every later boot that loads them (6 flashes, 0.72 s, only those buffers lit) before
// Motion_control_init() and the bus start, until they are recalibrated. No other pattern looks
// like it: the step prompts blink at 200/200 ms, the flash-error blink is all red at 260/260 ms,
// the recalibration wipe all blue at 150/150 ms, and in use the LEDs do not flash.
#define MC_PULL_CAL_FLASH_MS     60u
#define MC_PULL_CAL_END_FLASHES  20u
#define MC_PULL_CAL_BOOT_FLASHES 6u

// Number of flashes to show; 0 (no delay at all) when no connected buffer has a fallback range.
static inline uint32_t mc_pull_cal_flashes(const mc_pull_cal_run_t *run, bool at_boot)
{
    if ((run->fallback_mask & run->inserted_mask) == 0u) return 0u;
    return at_boot ? MC_PULL_CAL_BOOT_FLASHES : MC_PULL_CAL_END_FLASHES;
}

typedef enum
{
    MC_PULL_CAL_LED_OFF = 0,
    MC_PULL_CAL_LED_GREEN,
    MC_PULL_CAL_LED_RED,
} mc_pull_cal_led_t;

// Channel ch's LED in one phase of the flash: a fallback buffer flashes red; at the end of a run a
// calibrated one stays green, at boot it stays dark; a channel without a buffer stays dark.
static inline mc_pull_cal_led_t mc_pull_cal_flash_led(const mc_pull_cal_run_t *run, uint8_t ch, bool phase_on,
                                                      bool at_boot)
{
    const uint8_t bit = (uint8_t)(1u << ch);
    if (!(run->inserted_mask & bit)) return MC_PULL_CAL_LED_OFF;
    if (run->fallback_mask & bit) return phase_on ? MC_PULL_CAL_LED_RED : MC_PULL_CAL_LED_OFF;
    return at_boot ? MC_PULL_CAL_LED_OFF : MC_PULL_CAL_LED_GREEN;
}

// The calibration record's header word rsv (Flash_saves.cpp, 'CAL2' version 1). Bits 0..3:
// channel 0..3 has polarity -1 (V10.3+). Bits 4..7: channel 0..3 has a fallback range. Every source
// release (V6..V10.5) wrote the bits above 3 as 0 and none reads them, so the layout is unchanged:
// an older record reads as "no fallback", and an older firmware loads a newer record as before.
#define MC_PULL_CAL_RSV_FALLBACK_SHIFT 4u

static inline uint32_t mc_pull_cal_rsv_pack(const int8_t pol[4], uint8_t fallback_mask)
{
    uint32_t rsv = 0u;
    for (uint8_t ch = 0u; ch < 4u; ch++)
    {
        if (pol && pol[ch] < 0) rsv |= (1u << ch);
    }
    return rsv | ((uint32_t)(fallback_mask & 0x0Fu) << MC_PULL_CAL_RSV_FALLBACK_SHIFT);
}

static inline int8_t mc_pull_cal_rsv_pol(uint32_t rsv, uint8_t ch)
{
    return (rsv & (1u << ch)) ? (int8_t)-1 : (int8_t)1;
}

static inline uint8_t mc_pull_cal_rsv_fallback(uint32_t rsv)
{
    return (uint8_t)((rsv >> MC_PULL_CAL_RSV_FALLBACK_SHIFT) & 0x0Fu);
}
