#pragma once
// Limits for the motor states that otherwise end only on a sensor event (Motion_control.cpp): the
// unload pull back, the redetect push after it, and the DM autoload Stage-2 push/retract stages.
// Hardware-free, so the decisions are tested on the host (test/test_motion_limits).
//
// Without limits, a held filament or an emptied channel kept the motor at 900-1000 PWM for good.
// While a channel is in pull back or redetect, motor_motion_switch does not run, so the printer's
// commands for every channel were ignored until filament was inserted or the BMCU power-cycled.
// A state that hits a limit ends as if its sensor event had come, and the motor stops.
#include <stdbool.h>
#include <stdint.h>

// Gear travel is measured in AS5600 counts, not with the float odometer filament[].meters. That
// float gets one small step per AS5600 read (about 1 ms), and once it is large the steps start to
// get lost (slow moves after about 128 m net since boot, 60 mm/s after 1-2 km), so a turning gear
// would look stalled. The position fed to the guard is a running uint32_t sum of the same per-read
// angle steps that meters gets (as5600_count in Motion_control.cpp). Every distance is a difference
// to the position at the start of the state, taken modulo 2^32, so it starts at 0 for each state
// and is exact at any odometer value. One count is pi * 7.5 mm / 4096 = 5.75 um of gear travel.
#define ML_MM_PER_CNT (3.14159265358979323846f * 7.5f / 4096.0f)

// Each time budget is the move's distance at ML_V_MIN_MM_S, plus ML_MARGIN_MS. 12 mm/s is the pull
// back's end speed (PULL_V_END), the slowest speed any of these moves is commanded at. The pull
// runs at 60 mm/s otherwise. The open-loop moves use 900 PWM, more than twice the 400 PWM floor
// (PULL_PWM_MIN) the pull uses for its 12 mm/s end. A move that is slower than 12 mm/s over its
// whole distance is not following its command.
#define ML_V_MIN_MM_S 12.0f

// Covers the pull's 400 ms soft start, about 0.7 s for its speed PID (P 2, I 20, 60 mm/s error) to
// wind up to full PWM against drag, and spin-up/reversal of the open-loop moves.
#define ML_MARGIN_MS 2000u

// Stall: at least ML_STALL_PWM, with the gear moving less than ML_STALL_MOVE_CNT (1 mm) in
// ML_STALL_MS, i.e. slower than 1 mm/s. 800 is the on_use "high push" level, 1.6x the 500 PWM
// the pull uses as its floor. At the slowest commanded speed (12 mm/s) the gear covers 1 mm in
// 83 ms. 174 counts (1.0009 mm) is far above sensor jitter. The on_use anti-stall already treats
// 0.8 s without motion at 450 PWM or more as a stall, and this limit waits longer. A pull that
// starts against a held filament reaches 800 PWM after about 0.6 s, so it stops about 1.6 s in.
// The window restarts each time the gear has moved 1 mm, so a gear that stops dead is caught
// ML_STALL_MS after its last full 1 mm: at 60 mm/s up to 17 ms before block + ML_STALL_MS.
#define ML_STALL_PWM 800.0f
#define ML_STALL_MOVE_CNT 174u
#define ML_STALL_MS 1000u

// A gap between two checks that is longer than motor_motion_run's time-step cap (200 ms) means
// the state was not being driven. For example, the bus was offline: the motors stopped and the
// state machine was frozen. Such a gap uses none of the budget, and the stall window restarts.
#define ML_STEP_MAX_MS 200u

typedef enum
{
    ML_OK = 0, // keep going
    ML_TIME,   // limit: time budget used up
    ML_DIST,   // limit: moved the maximum distance
    ML_STALL,  // limit: high PWM, gear not moving
    ML_END,    // the state's own end (target reached, switch event), not a limit
} ml_result;

static inline bool ml_is_limit(ml_result r) { return (r == ML_TIME) || (r == ML_DIST) || (r == ML_STALL); }

typedef struct
{
    uint64_t last_ms;   // time of the previous check (start, then each check)
    uint32_t run_ms;    // time the state has been driven, gaps excluded
    uint32_t max_ms;    // time budget
    uint32_t stall_ms;  // how long the gear has been held at high PWM without moving
    uint32_t start_cnt; // gear position (AS5600 counts) at the start
    uint32_t stall_cnt; // gear position when the current stall window started
    uint32_t max_cnt;   // maximum distance from start_cnt, 0 = none
} motion_guard;

// Time budget for moving dist_m: at ML_V_MIN_MM_S, plus ML_MARGIN_MS.
static inline uint32_t ml_time_budget_ms(float dist_m)
{
    float ms = (dist_m > 0.0f) ? (dist_m * 1000000.0f / ML_V_MIN_MM_S) : 0.0f; // m / (mm/s) -> ms
    if (ms > 4.0e9f) ms = 4.0e9f; // keeps the conversion (and + margin) inside uint32_t
    return (uint32_t)(ms + 0.5f) + ML_MARGIN_MS;
}

// Distance dist_m in whole AS5600 counts (nearest), 0 for none.
static inline uint32_t ml_m_to_cnt(float dist_m)
{
    float c = (dist_m > 0.0f) ? (dist_m * 1000.0f / ML_MM_PER_CNT) : 0.0f;
    if (c > 2.0e9f) c = 2.0e9f;
    return (uint32_t)(c + 0.5f);
}

// |a - b| of two positions on the wrapping uint32_t count, without signed overflow.
static inline uint32_t ml_cnt_dist(uint32_t a, uint32_t b)
{
    const uint32_t d = a - b;
    return (d > 0x80000000u) ? (0u - d) : d;
}

// Distance cnt (whole AS5600 counts) in m. The count is exact, so the result has only float's
// relative error (about 1e-7), however large the odometer is.
static inline float ml_cnt_to_m(uint32_t cnt)
{
    return (float)cnt * (ML_MM_PER_CNT * 0.001f);
}

// Gear travel |pos_cnt - start_cnt| in m. Every distance a motor state decides on (pull back
// target, send length cap, DM Stage-2 countdown) comes from here, never from filament[].meters.
static inline float ml_travel_m(uint32_t pos_cnt, uint32_t start_cnt)
{
    return ml_cnt_to_m(ml_cnt_dist(pos_cnt, start_cnt));
}

static inline float ml_absf(float x) { return (x < 0.0f) ? -x : x; }

static inline void motion_guard_start(motion_guard *g, uint64_t now_ms, uint32_t pos_cnt, uint32_t max_ms, uint32_t max_cnt)
{
    g->last_ms   = now_ms;
    g->run_ms    = 0u;
    g->max_ms    = max_ms;
    g->stall_ms  = 0u;
    g->start_cnt = pos_cnt;
    g->stall_cnt = pos_cnt;
    g->max_cnt   = max_cnt;
}

// One check per main-loop pass while the state runs. pwm is the PWM the channel is driven with
// (sign ignored), pos_cnt the gear position in AS5600 counts. Returns ML_OK or a limit.
static inline ml_result motion_guard_check(motion_guard *g, uint64_t now_ms, uint32_t pos_cnt, float pwm)
{
    uint64_t dt64 = now_ms - g->last_ms;
    g->last_ms = now_ms;

    if (dt64 > ML_STEP_MAX_MS)
    {
        dt64 = 0u;
        g->stall_ms  = 0u;
        g->stall_cnt = pos_cnt;
    }
    const uint32_t dt = (uint32_t)dt64;

    g->run_ms = (g->run_ms > UINT32_MAX - dt) ? UINT32_MAX : (g->run_ms + dt);

    if ((ml_absf(pwm) < ML_STALL_PWM) || (ml_cnt_dist(pos_cnt, g->stall_cnt) >= ML_STALL_MOVE_CNT))
    {
        // Low PWM or the gear moved: a new window starts here.
        g->stall_ms  = 0u;
        g->stall_cnt = pos_cnt;
    }
    else
    {
        g->stall_ms += dt;
    }

    if ((g->max_cnt > 0u) && (ml_cnt_dist(pos_cnt, g->start_cnt) >= g->max_cnt)) return ML_DIST;
    if (g->stall_ms >= ML_STALL_MS) return ML_STALL;
    if (g->run_ms >= g->max_ms) return ML_TIME;
    return ML_OK;
}

// ---- Pull back (filament_pulling_back) ----
// Budget: the target at ML_V_MIN_MM_S (95 mm SOLO: 9.9 s; a normal pull takes about 1.9 s).
static inline void ml_pull_back_start(motion_guard *g, uint64_t now_ms, uint32_t pos_cnt, float target_m)
{
    motion_guard_start(g, now_ms, pos_cnt, ml_time_budget_ms(target_m), 0u);
}

// Not ML_OK when the pull back ends, and the channel then goes to redetect: ML_END when the target
// is reached or no switch sees filament (as before), else the limit that was hit. ks is
// MC_ONLINE_key_stu, pulled_m the distance pulled so far as the pull itself measures it.
static inline ml_result ml_pull_back_check(motion_guard *g, uint64_t now_ms, uint32_t pos_cnt, float pwm,
                                           float target_m, float pulled_m, uint8_t ks)
{
    if ((target_m <= 0.0f) || (pulled_m >= target_m)) return ML_END;
    if (ks == 0u) return ML_END;
    return motion_guard_check(g, now_ms, pos_cnt, pwm);
}

// ---- Redetect (filament_redetect) ----
// It pushes while no switch sees filament. Pushing the configured retract length undoes the whole
// pull back, which started with filament at the switches. If the switches still see nothing after
// that, the gear is not moving filament (it was pulled out of the BMCU), so the push stops. The
// budget is the same distance at ML_V_MIN_MM_S.
static inline void ml_redetect_start(motion_guard *g, uint64_t now_ms, uint32_t pos_cnt, float retract_m)
{
    motion_guard_start(g, now_ms, pos_cnt, ml_time_budget_ms(retract_m), ml_m_to_cnt(retract_m));
}

// Not ML_OK when the redetect ends and the channel goes idle: ML_END when a switch sees filament
// (as before), else the limit that was hit.
static inline ml_result ml_redetect_check(motion_guard *g, uint64_t now_ms, uint32_t pos_cnt, float pwm, uint8_t ks)
{
    if (ks != 0u) return ML_END;
    return motion_guard_check(g, now_ms, pos_cnt, pwm);
}

// ---- DM autoload Stage-2 push / retract / fail retract ----
// Each stage moves at most the Stage-2 length (DM_AUTO_S2_TARGET_M, 120 mm): the push counts it
// down, the retracts undo at most what was pushed before the tip is back at the inner switch.
// Budget: that length at ML_V_MIN_MM_S (12 s), plus the stall check.
static inline void ml_dm_s2_start(motion_guard *g, uint64_t now_ms, uint32_t pos_cnt, float s2_len_m)
{
    motion_guard_start(g, now_ms, pos_cnt, ml_time_budget_ms(s2_len_m), 0u);
}

// ---- Unfinished unload (status LED) ----
// A pull back stopped by a limit did not finish the unload, but the printer is still told it did:
// the bus has no reply for it, and no printer-facing byte changes. The channel's status LED blinks
// red instead (Motion_control.cpp). The fault is set by the pull's end (ml_is_limit), and it stays
// until no switch sees filament (the user pulled it out) or the printer starts another move on the
// channel (load, print or unload: channel_busy). The redetect and idle after the pull keep it.
static inline bool ml_unload_fault_kept(bool fault, uint8_t ks, bool channel_busy)
{
    return fault && (ks != 0u) && !channel_busy;
}
