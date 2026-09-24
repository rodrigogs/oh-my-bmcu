#include "MC_PULL_calibration.h"
#include "mc_pull_cal_range.h"
#include "Motion_control.h"
#include "ADC_DMA.h"
#include "Flash_saves.h"
#include "hal/time_hw.h"
#include "Debug_log.h"
#include "app_api.h"
#include <math.h>

extern void RGB_update();

static inline float adc_pull_raw_ch(int ch, const float *v8)
{
    switch (ch)
    {
    case 0: return (float)v8[6];
    case 1: return (float)v8[4];
    case 2: return (float)v8[2];
    default:return (float)v8[0];
    }
}

static inline float adc_key_raw_ch(int ch, const float *v8)
{
    switch (ch)
    {
    case 0: return (float)v8[7];
    case 1: return (float)v8[5];
    case 2: return (float)v8[3];
    default:return (float)v8[1];
    }
}

static inline uint8_t dm_key_round_up_to_centi(float v)
{
    if (v <= 0.0f) return 0u;

    float x = v * 100.0f - 0.0001f;
    int iv = (int)x;
    if ((float)iv < x) iv++;

    if (iv < 0) iv = 0;
    if (iv > 255) iv = 255;
    return (uint8_t)iv;
}

static inline float dm_key_none_threshold_from_idle(float key_value)
{
    const uint8_t key_cv = dm_key_round_up_to_centi(key_value);

    uint8_t thr_cv = (uint8_t)(key_cv + 10u);
    if (thr_cv < 60u) thr_cv = 60u;
    if (thr_cv > 139u) thr_cv = 139u;

    return 0.01f * (float)thr_cv;
}

static inline float adc_pull_v_centered(int ch)
{
    const float *d = ADC_DMA_get_value();
    return adc_pull_raw_ch(ch, d) + MC_PULL_V_OFFSET[ch];
}

static inline float cal_apply_polarity(float v, int8_t pol)
{
    return (pol < 0) ? (3.30f - v) : v;
}

static const float    CAL_PRESS_DELTA_V = MC_PULL_CAL_PRESS_DELTA_V;
static const float    CAL_CENTER_EPS_V  = 0.02f;
static const uint32_t CAL_STABLE_MS     = 200;
static const uint32_t CAL_TIMEOUT_MS    = 30000;

static void blink_all(uint8_t r, uint8_t g, uint8_t b, int times = 4, int on_ms = 60, int off_ms = 60)
{
    for (int k = 0; k < times; k++)
    {
        for (int ch = 0; ch < 4; ch++) MC_PULL_ONLINE_RGB_set(ch, r, g, b);
        RGB_update(); delay(on_ms);
        for (int ch = 0; ch < 4; ch++) MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
        RGB_update(); delay(off_ms);
    }
}

static void blink_one(int ch, uint8_t r, uint8_t g, uint8_t b, int times = 3, int on_ms = 70, int off_ms = 70)
{
    for (int k = 0; k < times; k++)
    {
        MC_PULL_ONLINE_RGB_set(ch, r, g, b);
        RGB_update(); delay(on_ms);
        MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
        RGB_update(); delay(off_ms);
    }
}

static void show_diag_step(uint8_t r, uint8_t g, uint8_t b, int on_ms = 360, int off_ms = 180)
{
    for (int ch = 0; ch < 4; ch++) MC_PULL_ONLINE_RGB_set(ch, r, g, b);
    RGB_update();
    delay(on_ms);
    for (int ch = 0; ch < 4; ch++) MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
    RGB_update();
    delay(off_ms);
}

static bool capture_first_extreme_wait_release(int ch, float center_v, float &out_best_norm, int8_t &out_pol)
{
    const uint32_t tpm = time_hw_ticks_per_ms();
    const uint32_t t0  = time_ticks32();
    const uint32_t dt  = (uint32_t)CAL_TIMEOUT_MS * tpm;

    bool pressed = false;
    float raw_min = center_v;
    float raw_max = center_v;
    uint32_t stable_t0 = 0u;

    out_best_norm = center_v;
    out_pol = 1;

    for (;;)
    {
        const uint32_t now_t = time_ticks32();
        if ((uint32_t)(now_t - t0) >= dt) break;

        const float v = adc_pull_v_centered(ch);

        const uint32_t elapsed_ms = (uint32_t)((now_t - t0) / tpm);
        if (((elapsed_ms / 200u) & 1u) == 0u) MC_PULL_ONLINE_RGB_set(ch, 0x00, 0x00, 0x10);
        else                                  MC_PULL_ONLINE_RGB_set(ch, 0x00, 0x00, 0x00);
        RGB_update();

        if (!pressed)
        {
            if (fabsf(v - center_v) >= CAL_PRESS_DELTA_V)
            {
                pressed = true;
                raw_min = v;
                raw_max = v;
            }
        }
        else
        {
            if (v < raw_min) raw_min = v;
            if (v > raw_max) raw_max = v;

            if (fabsf(v - center_v) <= CAL_CENTER_EPS_V)
            {
                if (stable_t0 == 0u) stable_t0 = now_t;
                if ((uint32_t)(now_t - stable_t0) >= (uint32_t)CAL_STABLE_MS * tpm)
                {
                    const float dmin = center_v - raw_min;
                    const float dmax = raw_max - center_v;

                    if (dmax > dmin)
                    {
                        out_pol = -1;
                        out_best_norm = cal_apply_polarity(raw_max, out_pol);
                    }
                    else
                    {
                        out_pol = 1;
                        out_best_norm = cal_apply_polarity(raw_min, out_pol);
                    }

                    MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
                    RGB_update();
                    return true;
                }
            }
            else
            {
                stable_t0 = 0u;
            }
        }

        delay(15);
    }

    out_best_norm = center_v;
    out_pol = 1;
    MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
    RGB_update();
    return false;
}

static bool capture_second_extreme_wait_release(int ch, float center_v, int8_t pol, float &out_best_norm)
{
    const uint32_t tpm = time_hw_ticks_per_ms();
    const uint32_t t0  = time_ticks32();
    const uint32_t dt  = (uint32_t)CAL_TIMEOUT_MS * tpm;

    const bool want_raw_min = (pol < 0);

    bool pressed = false;
    float best_raw = center_v;
    uint32_t stable_t0 = 0u;

    for (;;)
    {
        const uint32_t now_t = time_ticks32();
        if ((uint32_t)(now_t - t0) >= dt) break;

        const float v = adc_pull_v_centered(ch);

        const uint32_t elapsed_ms = (uint32_t)((now_t - t0) / tpm);
        if (((elapsed_ms / 200u) & 1u) == 0u) MC_PULL_ONLINE_RGB_set(ch, 0x10, 0x00, 0x00);
        else                                  MC_PULL_ONLINE_RGB_set(ch, 0x00, 0x00, 0x00);
        RGB_update();

        if (!pressed)
        {
            if (want_raw_min)
            {
                if (v < (center_v - CAL_PRESS_DELTA_V))
                {
                    pressed = true;
                    best_raw = v;
                }
            }
            else
            {
                if (v > (center_v + CAL_PRESS_DELTA_V))
                {
                    pressed = true;
                    best_raw = v;
                }
            }
        }
        else
        {
            if (want_raw_min)
            {
                if (v < best_raw) best_raw = v;
            }
            else
            {
                if (v > best_raw) best_raw = v;
            }

            if (fabsf(v - center_v) <= CAL_CENTER_EPS_V)
            {
                if (stable_t0 == 0u) stable_t0 = now_t;
                if ((uint32_t)(now_t - stable_t0) >= (uint32_t)CAL_STABLE_MS * tpm)
                {
                    out_best_norm = cal_apply_polarity(best_raw, pol);
                    MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
                    RGB_update();
                    return true;
                }
            }
            else
            {
                stable_t0 = 0u;
            }
        }

        delay(15);
    }

    out_best_norm = center_v;
    MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
    RGB_update();
    return false;
}

// Both steps on channel ch. The range to store comes from mc_pull_cal_run_channel(), which also
// marks ch in run (and as a fallback if a side is one; see mc_pull_cal_range.h). A first step whose
// end is too shallow to keep gets no yellow ack, like one that timed out.
static void capture_minmax_one_ch_event(int ch, float center_v, mc_pull_cal_run_t &run,
                                        float &out_min, float &out_max, int8_t &out_pol)
{
    float vmin = center_v;
    float vmax = center_v;
    int8_t pol = 1;

    bool ok_min = capture_first_extreme_wait_release(ch, center_v, vmin, pol);
    if (mc_pull_cal_low_ok(center_v, ok_min, vmin)) blink_one(ch, 0x10, 0x10, 0x00, 2, 60, 60);

    bool ok_max = capture_second_extreme_wait_release(ch, center_v, pol, vmax);
    if (ok_max) blink_one(ch, 0x10, 0x10, 0x00, 2, 60, 60);

    const mc_pull_cal_range_t r =
        mc_pull_cal_run_channel(&run, (uint8_t)ch, center_v, ok_min, vmin, pol, ok_max, vmax);

    out_min = r.vmin;
    out_max = r.vmax;
    out_pol = r.pol;
}

// Fast red flash on the buffers whose saved range is a fallback: at the end of the run that stored
// it (calibrated buffers stay green) and briefly at every boot that loads it, before the bus
// starts (see mc_pull_cal_range.h). Returns at once if there is nothing to show.
static void blink_cal_fallback(const mc_pull_cal_run_t &run, bool at_boot)
{
    const uint32_t phases = 2u * mc_pull_cal_flashes(&run, at_boot);
    if (phases == 0u) return;

    for (uint32_t k = 0u; k < phases; k++)
    {
        const bool on = ((k & 1u) == 0u);
        for (int ch = 0; ch < 4; ch++)
        {
            switch (mc_pull_cal_flash_led(&run, (uint8_t)ch, on, at_boot))
            {
            case MC_PULL_CAL_LED_RED:   MC_PULL_ONLINE_RGB_set(ch, 0x10, 0x00, 0x00); break;
            case MC_PULL_CAL_LED_GREEN: MC_PULL_ONLINE_RGB_set(ch, 0x00, 0x10, 0x00); break;
            default:                    MC_PULL_ONLINE_RGB_set(ch, 0x00, 0x00, 0x00); break;
            }
        }
        RGB_update();
        delay(MC_PULL_CAL_FLASH_MS);
    }

    for (int ch = 0; ch < 4; ch++) MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
    RGB_update();
}

void MC_PULL_calibration_clear()
{
    Flash_NVM_full_clear();
}

void MC_PULL_calibration_boot()
{
    for (int i = 0; i < 6; i++) { ADC_DMA_poll(); delay(20); }

    MC_PULL_detect_channels_inserted();

    float offs[4], vmin[4], vmax[4];
    int8_t pol[4];
    uint8_t fallback_mask = 0u;
    if (Flash_MC_PULL_cal_read(offs, vmin, vmax, pol, &fallback_mask))
    {
        for (int ch = 0; ch < 4; ch++)
        {
            MC_PULL_V_OFFSET[ch] = offs[ch];
            MC_PULL_V_MIN[ch]    = vmin[ch];
            MC_PULL_V_MAX[ch]    = vmax[ch];
            MC_PULL_POLARITY[ch] = (pol[ch] < 0) ? -1 : 1;
        }

        blink_cal_fallback(mc_pull_cal_run_loaded(filament_channel_inserted, fallback_mask), true);
        return;
    }

    const bool ok_wipe = Flash_NVM_full_clear();

    double sum_raw[4] = {0, 0, 0, 0};
    double sum_key[4] = {0, 0, 0, 0};
    const int N = 90;
    const uint32_t tpm = time_hw_ticks_per_ms();
    const uint32_t t0  = time_ticks32();

    for (int k = 0; k < N; k++)
    {
        const float *v = ADC_DMA_get_value();

        for (int ch = 0; ch < 4; ch++)
        {
            if (!filament_channel_inserted[ch]) continue;
            sum_raw[ch] += adc_pull_raw_ch(ch, v);
            sum_key[ch] += adc_key_raw_ch(ch, v);
        }

        const uint32_t now_t = time_ticks32();
        const uint32_t elapsed_ms = (uint32_t)((now_t - t0) / tpm);
        bool on = (((elapsed_ms / 200u) & 1u) == 0u);
        for (int ch = 0; ch < 4; ch++)
            MC_PULL_ONLINE_RGB_set(ch, on ? 0x10 : 0, on ? 0x10 : 0, 0x00);
        RGB_update();
        delay(15);
    }

    float center_raw[4] = {1.65f, 1.65f, 1.65f, 1.65f};
    for (int ch = 0; ch < 4; ch++)
    {
        if (!filament_channel_inserted[ch]) continue;
        center_raw[ch] = (float)(sum_raw[ch] / (double)N);
    }

    for (int ch = 0; ch < 4; ch++)
    {
        if (!filament_channel_inserted[ch])
        {
            MC_PULL_V_OFFSET[ch] = 0.0f;
            MC_PULL_POLARITY[ch] = 1;
            MC_DM_KEY_NONE_THRESH[ch] = 0.60f;
            continue;
        }

        MC_PULL_V_OFFSET[ch] = 1.65f - center_raw[ch];
        MC_PULL_POLARITY[ch] = 1;
        MC_DM_KEY_NONE_THRESH[ch] =
            dm_key_none_threshold_from_idle((float)(sum_key[ch] / (double)N));
    }

    float center_v_ref[4] = {1.65f, 1.65f, 1.65f, 1.65f};
    for (int ch = 0; ch < 4; ch++)
    {
        if (!filament_channel_inserted[ch]) continue;
        center_v_ref[ch] = center_raw[ch] + MC_PULL_V_OFFSET[ch];
    }

    blink_all(0x10, 0x10, 0x00, 3, 60, 60);

    mc_pull_cal_run_t run = {0u, 0u};

    for (int ch = 0; ch < 4; ch++)
    {
        if (!filament_channel_inserted[ch])
        {
            MC_PULL_V_MIN[ch] = 1.55f;
            MC_PULL_V_MAX[ch] = 1.75f;
            MC_PULL_POLARITY[ch] = 1;
            continue;
        }

        float mn, mx;
        int8_t p;
        capture_minmax_one_ch_event(ch, center_v_ref[ch], run, mn, mx, p);

        MC_PULL_V_MIN[ch] = mn;
        MC_PULL_V_MAX[ch] = mx;
        MC_PULL_POLARITY[ch] = (p < 0) ? -1 : 1;

        MC_PULL_ONLINE_RGB_set(ch, 0, 0, 0);
        RGB_update();
        delay(80);
    }

    const bool ok_cal = Flash_MC_PULL_cal_write_all(MC_PULL_V_OFFSET, MC_PULL_V_MIN, MC_PULL_V_MAX, MC_PULL_POLARITY,
                                                    run.fallback_mask);
    const bool ok_mot = Motion_control_save_dm_key_none_thresholds();
    const bool ok = ok_wipe && ok_cal && ok_mot;

    if (ok_wipe) show_diag_step(0x00, 0x10, 0x10);
    else         show_diag_step(0x10, 0x00, 0x00, 460, 220);

    if (ok_cal) show_diag_step(0x10, 0x10, 0x00);
    else        show_diag_step(0x10, 0x00, 0x10, 460, 220);

    if (ok_mot) show_diag_step(0x00, 0x00, 0x10);
    else        show_diag_step(0x10, 0x10, 0x10, 460, 220);

    switch (mc_pull_cal_end(ok, &run))
    {
    case MC_PULL_CAL_END_GREEN:       blink_all(0x00, 0x10, 0x00, 2, 220, 220); break;
    case MC_PULL_CAL_END_FLASH_ERROR: blink_all(0x10, 0x00, 0x00, 2, 260, 260); break;
    default:                          break;
    }

    blink_cal_fallback(run, false);

    delay(200);
}