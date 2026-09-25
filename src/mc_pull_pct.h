#pragma once
// The buffer position in percent of its calibrated range (Motion_control.cpp: pull_v_to_percent_f,
// i.e. MC_PULL_pct_f), and the recalibration gesture that reads it. Hardware-free, so the
// calibration's safety tests (test/test_mc_pull_cal_range) run the firmware's own mapping and
// gesture rule on the ranges a calibration can store.
#include <stdbool.h>
#include <stdint.h>

// clampf(x, 0, 1) of Motion_control.cpp, in the same form.
static inline float mc_pull_clamp01(float x)
{
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

// v: the channel's normalised reading (MC_PULL_stu_raw: MC_PULL_V_OFFSET puts the idle reading at
// 1.65 V, the polarity puts a pulled buffer below it). vmin/vmax: its calibrated range
// (MC_PULL_V_MIN/MAX). Each side maps linearly, vmin to 0 %, 1.65 V to 50 %, vmax to 100 %, and the
// result is clamped to 0..100 %. A side narrower than 50 mV is widened to 50 mV first, and a range
// that is then 100 mV wide becomes 1.55..1.75 V. MC_PULL_ONLINE_read rounds it to MC_PULL_pct.
static inline float mc_pull_v_to_pct(float vmin, float vmax, float v)
{
    const float c = 1.65f;

    if (vmin > 1.60f) vmin = 1.60f;
    if (vmax < 1.70f) vmax = 1.70f;
    if (vmax <= (vmin + 0.10f)) { vmin = 1.55f; vmax = 1.75f; }

    float pos01;
    if (v <= c)
    {
        float den = c - vmin;
        if (den < 0.05f) den = 0.05f;
        pos01 = 0.5f * (v - vmin) / den;
    }
    else
    {
        float den = vmax - c;
        if (den < 0.05f) den = 0.05f;
        pos01 = 0.5f + 0.5f * (v - c) / den;
    }

    return mc_pull_clamp01(pos01) * 100.0f;
}

// ---- Recalibration gesture ----
// With no filament at any channel's switches, one connected buffer held pulled for
// MC_PULL_CAL_RESET_HOLD_MS makes Motion_control_run blink blue for 3 s, wipe the whole NVM and
// reboot, so the next boot calibrates the buffers again, online or not: main.cpp passes it an
// error of 0 (link up) or -1 (offline, such as on USB power alone), so its 'error <= 0' always
// holds. Pulled: its MC_PULL_pct is at most MC_PULL_CAL_RESET_PCT_THRESH, or its reading is
// MC_PULL_CAL_RESET_V_DELTA or more below the 1.65 V centre, or within MC_PULL_CAL_RESET_NEAR_MIN
// of its calibrated low end.
#define MC_PULL_CAL_RESET_HOLD_MS    5000u
#define MC_PULL_CAL_RESET_PCT_THRESH 15
#define MC_PULL_CAL_RESET_V_DELTA    0.10f
#define MC_PULL_CAL_RESET_NEAR_MIN   0.03f

// pct: the channel's MC_PULL_pct (as int); v: its MC_PULL_stu_raw; vmin: its MC_PULL_V_MIN. A
// macro, not an inline function: that way Motion_control_run compiles to exactly the code it had
// with the expression written out in it (as a function, GCC laid it out differently).
#define MC_PULL_CAL_RESET_PRESSED(pct, v, vmin)             \
    (((pct) <= MC_PULL_CAL_RESET_PCT_THRESH) ||             \
     ((v) <= (1.65f - MC_PULL_CAL_RESET_V_DELTA)) ||        \
     ((v) <= ((vmin) + MC_PULL_CAL_RESET_NEAR_MIN)))
