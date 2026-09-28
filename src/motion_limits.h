#pragma once
// Limits for the motor states that otherwise end only on a sensor event or a printer command
// (Motion_control.cpp): the unload pull back, the redetect push after it, the DM autoload Stage-2
// push/retract stages, the send (load), the buffer-lift auto-unload (auto_unload.h) and the idle
// control's push.
// Hardware-free, so the decisions are tested on the host (test/test_motion_limits,
// test/test_auto_unload).
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
    uint32_t max_ms;    // time budget, 0 = none
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
// target, DM Stage-2 countdown) comes from here or from the guard's count (redetect and send
// caps), never from filament[].meters.
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
// (sign ignored), pos_cnt the gear position in AS5600 counts, stall_pwm the lowest PWM at which a
// gear that does not move is stalled. Returns ML_OK or a limit.
static inline ml_result motion_guard_check_at(motion_guard *g, uint64_t now_ms, uint32_t pos_cnt, float pwm,
                                              float stall_pwm)
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

    if ((ml_absf(pwm) < stall_pwm) || (ml_cnt_dist(pos_cnt, g->stall_cnt) >= ML_STALL_MOVE_CNT))
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
    if ((g->max_ms > 0u) && (g->run_ms >= g->max_ms)) return ML_TIME;
    return ML_OK;
}

// The check with the stall level of every open-loop or speed-controlled move, ML_STALL_PWM.
static inline ml_result motion_guard_check(motion_guard *g, uint64_t now_ms, uint32_t pos_cnt, float pwm)
{
    return motion_guard_check_at(g, now_ms, pos_cnt, pwm, ML_STALL_PWM);
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

// ---- Send (filament_motion_send) ----
// The printer's load: it commands send_out until its tool head sees the filament. The send feeds at
// 60 mm/s (speed PID, P 2, I 20, 500 PWM floor, clamped at 1000) until the tip at the extruder has
// pushed the buffer to MC_LOAD_S1_FAST_PCT (85 % on A1), then holds it (hold_load, with the on_use
// anti-stall), and brakes at MC_LOAD_S1_HARD_STOP_PCT (95 %). A gear that does not turn (a tangle
// on the spool) moves no filament, so the buffer never reaches either: the PID sat at 1000 PWM for
// as long as the printer kept sending send_out. The stall check stops it: about 0.7 s to 800 PWM,
// then ML_STALL_MS. A normal load never trips it: the gear moves while it feeds and while it
// pushes the buffer up. Nor does the hold. On A1 (latch 85 %, hold target 90 %, or the pct it
// latched at if higher) it drives 800 PWM or more only when it pushes with the buffer below 83.7 %,
// under the 85 % it latched at, when it retracts above about 92.0 % (800-850 PWM,
// retract_mag_from_err(err, 850), 2 points over the target), or in the anti-stall's 850 PWM kick.
// The guard counts the retract too (ml_absf), and in all three the anti-stall rests 0.5 s after
// 0.8 s without motion, under ML_STALL_MS, so a buffer that stops between 83.7 % and 92 % is not a
// stall. With BMCU_SOFT_LOAD (latch and target 75 %, so the target is the pct it latched at) the
// push is 800 PWM or more below 62.6 % and the retract 2 points over that pct (about 77.0 % for a
// latch at 75 %). On BMCU_P1S (latch 88 %, target 95 %) the push is 1000 PWM at the latch point
// and 800 PWM or more up to 90.67 %, so the hold drives above 800 PWM at and just above the latch,
// bounded by the same rest; its retract reaches 800 PWM only above about 97.0 %, where the send has
// braked (MC_LOAD_S1_HARD_STOP_PCT).
// No time budget: how long the printer sends is its own decision, and its load timeout reports a
// send that stopped. ML_SEND_MAX_M of gear travel is upstream's cap (a gear that turns without
// feeding, e.g. slipping on the filament).
#define ML_SEND_MAX_M 10.0f

static inline void ml_send_start(motion_guard *g, uint64_t now_ms, uint32_t pos_cnt)
{
    motion_guard_start(g, now_ms, pos_cnt, 0u, ml_m_to_cnt(ML_SEND_MAX_M));
}

// ---- Auto-unload (auto_unload.h) ----
// The buffer-lift gesture's retract, open loop at 850 PWM (AUTO_UNLOAD_PWM_PULL in Motion_control.cpp)
// from its first pass. It ends on its own when the buffer drops below 35% (the filament pulled
// tight), 1.5 s after the key left 'both' (unloaded) or after 15 s. A gear that does not turn
// (filament caught ahead of the gear or behind it) moves the filament neither out of the switches
// nor tight, so it retracted for the whole 15 s. The stall check stops it ML_STALL_MS after the gear
// stopped. A normal auto-unload never trips it: the gear moves the filament while it retracts, and
// once the tail has left the gear it spins free.
// No time budget (the 15 s end stays its own) and no distance limit: the gesture pulls the filament
// from wherever it is parked out past the switches, which can be the whole tube to the printer, far
// more than the retract length.
static inline void ml_auto_unload_start(motion_guard *g, uint64_t now_ms, uint32_t pos_cnt)
{
    motion_guard_start(g, now_ms, pos_cnt, 0u, 0u);
}

// ---- Idle control push (filament_motion_pressure_ctrl_idle) ----
// A channel with filament at the switches that the printer is not using runs the idle control: with
// the buffer below 30 % (MC_PULL_DEADBAND_PCT_LOW) its pressure PID (P 25) pushes towards 50 %, at
// 512 PWM at the deadband's edge up to its 800 PWM clamp at 18 %, until the buffer is back at 30 %.
// It is not on_use_like, so neither the on_use anti-stall nor the 20 s push limit applies: a filament
// held tight (a tangle on the spool, the end fixed to an empty spool's core), or a buffer that reads
// low, had it push for good. A normal push only takes up the slack the buffer shows (after the
// BMCU's pull back, or a pull by the printer) and ends back at 30 %: the printer draws no filament
// from an idle channel. Each push (from the first pass that pushes to the first that does not) gets
// a guard: the stall check at ML_IDLE_STALL_PWM, the 420 PWM hold floor, so it sees every push the
// idle control makes, and ML_IDLE_PUSH_MAX_MS of push time, 120 mm at ML_V_MIN_MM_S. Retracts
// (above 70 %) are not guarded: they pull filament back to the spool, and a hand that holds the
// buffer up (the jam-latch release, the auto-unload's lift) is what stalls them.
// A limit hit with the rounded buffer at ML_IDLE_EDGE_PCT or above (28-29 %, the PID's last 512-562
// PWM, where a gear under heavy drag may stop short of its breakaway, as the on_use anti-stall leaves
// a gear within 2 % of its target) only brakes: the channel is held as if in the deadband, with no
// fault shown. Deeper, it brakes with the status LED red. The edge brake has one point of
// hysteresis: it holds down to a rounded 27 % (ML_IDLE_EDGE_EXIT_PCT), so a gear stopped at 28 %
// is not pushed again, and then shown red, by ADC noise across 27.5 %.
#define ML_IDLE_STALL_PWM     420.0f
#define ML_IDLE_PUSH_MAX_MS   10000u
#define ML_IDLE_EDGE_PCT      28u
#define ML_IDLE_EDGE_EXIT_PCT (ML_IDLE_EDGE_PCT - 1u)

typedef enum
{
    ML_IDLE_PUSH_RUN = 0, // drive as the PID asks
    ML_IDLE_PUSH_BRAKE,   // brake, no fault shown: a limit at the deadband's edge
    ML_IDLE_PUSH_FAULT,   // brake, status LED red
} ml_idle_push_act;

typedef struct
{
    motion_guard guard;   // the current push, started on its first pass
    uint8_t      pushing; // a push is under way
    uint8_t      fault;   // ML_IDLE_PUSH_BRAKE or _FAULT: a limit stopped it, held until the push ends
} ml_idle_push_t;

static inline void ml_idle_push_reset(ml_idle_push_t *s)
{
    s->pushing = 0u;
    s->fault   = ML_IDLE_PUSH_RUN;
}

// One pass of the idle control with filament at the switches. push: its PID pushes on this pass;
// pct: the rounded buffer reading (MC_PULL_pct); pwm: the PWM applied since the previous pass
// (x_prev). From the pass a limit is hit the channel is braked for as long as the control still
// pushes. Any pass on which it does not (the buffer back in the deadband, or above it) ends the push
// and clears the limit; ml_idle_push_reset() does when the channel leaves the idle control or its
// filament the switches. An edge brake also ends when the buffer falls below ML_IDLE_EDGE_EXIT_PCT
// (rounded 26 % or less: more filament drawn): a new push, with the higher PWM, its whole budget and
// a new stall window. A one-point dip to 27 % keeps the brake.
static inline ml_idle_push_act ml_idle_push_pass(ml_idle_push_t *s, bool push, uint8_t pct, uint64_t now_ms,
                                                 uint32_t pos_cnt, float pwm)
{
    if (!push)
    {
        ml_idle_push_reset(s);
        return ML_IDLE_PUSH_RUN;
    }
    if ((s->fault == ML_IDLE_PUSH_BRAKE) && (pct < ML_IDLE_EDGE_EXIT_PCT)) ml_idle_push_reset(s);
    if (s->fault != ML_IDLE_PUSH_RUN) return (ml_idle_push_act)s->fault;
    if (!s->pushing)
    {
        s->pushing = 1u;
        motion_guard_start(&s->guard, now_ms, pos_cnt, ML_IDLE_PUSH_MAX_MS, 0u);
        return ML_IDLE_PUSH_RUN;
    }
    if (!ml_is_limit(motion_guard_check_at(&s->guard, now_ms, pos_cnt, pwm, ML_IDLE_STALL_PWM)))
        return ML_IDLE_PUSH_RUN;
    s->fault = (pct >= ML_IDLE_EDGE_PCT) ? ML_IDLE_PUSH_BRAKE : ML_IDLE_PUSH_FAULT;
    return (ml_idle_push_act)s->fault;
}

// ---- Unfinished unload (status LED) ----
// A pull back stopped by a limit did not finish the unload, but the printer is still told it did:
// the bus has no reply for it, and no printer-facing byte changes. The channel's status LED blinks
// red instead (Motion_control.cpp). The fault is set by the pull's end (ml_is_limit), and it stays
// until no switch sees filament (the user pulled it out) or the printer starts another move on the
// channel (load, print or unload: channel_busy). The redetect and idle after the pull keep it. An
// auto-unload stopped by its stall check sets the same fault: the filament is still in the BMCU.
static inline bool ml_unload_fault_kept(bool fault, uint8_t ks, bool channel_busy)
{
    return fault && (ks != 0u) && !channel_busy;
}
