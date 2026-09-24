#pragma once
// DM autoload (BMCU_DM_TWO_MICROSWITCH) of Motion_control.cpp: what Stage-2 keeps when the key
// leaves 'both' while it runs. Hardware-free, so the decision is tested on the host
// (test/test_dm_stage2).
//
// Stage-2 pushes the filament 120 mm (DM_AUTO_S2_TARGET_M) open-loop at 900 PWM once the key
// reads 'both' on a channel that is not loaded. It counts the length down from the gear travel
// (dm_auto_remain_m; its retract after a buffer abort adds back what it undoes), counts the buffer
// aborts (dm_auto_try; the third sets the fail latch) and bounds each stage with a time and stall
// guard (dm_s2_guard, motion_limits.h). One pass with the key away from 'both' used to start all
// of it over at the next 'both': during S2_PUSH (external only -> S1_DEBOUNCE -> IDLE, or the other
// state -> IDLE) and during S2_RETRACT (external only). So key dips (noise, supply ripple, a
// vibrating lever) reset the 120 mm countdown, the time and stall budget and the three-abort fail
// latch while the motor pushed at 900 PWM, and a buffer that rose on every push could loop for good:
// abort, retract until the tip left the inner switch, Stage-1 back to 'both', a fresh Stage-2.
//
// Now a Stage-2 run lasts from the first 'both' of a channel that is not loaded until it ends by
// itself (the length is done and the channel is loaded, or it fails: fail latch), the filament
// leaves both switches (key 'none'), or the channel's loaded state changes (dm_rearm.h). Only then
// does the next 'both' start a new run with 120 mm and no aborts. The key leaving 'both' interrupts
// the run's stage (dm_s2_leave); the run keeps its remaining length and abort count, and at the
// next 'both' (dm_s2_enter: in IDLE, or in S1_PUSH once Stage-1 has pushed the tip back there):
// - an interrupted push resumes with its guard, however long the key was away;
// - an interrupted retract resumes with its guard if the key is back within DM_S2_RESUME_MS (a dip).
//   After a longer excursion (the retract took the tip back behind the inner switch, and Stage-1
//   pushed it to 'both' again) the run goes on with a push, a new stage, as the retract would at
//   its end; the abort count still bounds how often that can happen.
// The gear travel from the run's last update to the next 'both' counts as well when it went forward:
// the last push pass before the key left, and what Stage-1 pushed while the key read 'external
// only'. (Filament pulled back by hand meanwhile does not lengthen the push.) However often a run
// is interrupted, Stage-2 therefore pushes only while the run is short of 120 mm net forward (pushes
// minus the retracts it adds back) from where it started, and three buffer aborts still fail it. The
// run ends there, or short of it if the gear was turned back meanwhile other than by its retracts (by
// hand, or by an auto-unload, during a push or an excursion). Only Stage-1 can take it further: it
// pushes while the key reads 'external only' (after its 100 ms debounce) until the key reads 'both',
// so a key that misreads a tip past the inner switch as 'external only' adds that push (the run's
// guard still bounds it).
//
// The run is guarded as a whole (dm_s2_guard, motion_limits.h). Each of its stages gets its own
// time budget (12 s: 120 mm at ML_V_MIN_MM_S, plus the margin), but the stall window is the run's:
// it goes on across the stages, so a gear that has not moved does not get another ML_STALL_MS with
// every buffer abort (the retract after it, the push after that, or the push an interrupted retract
// goes on with). Only the distance the gear must move to restart the window is measured from each
// stage's start, as the stage may drive the other way. The guard also goes on while the key keeps
// the run interrupted: dm_s2_guard_pass() runs on every pass of the run but its first, the key's
// excursions included (IDLE, S1_DEBOUNCE, S1_PUSH). 'external only' sends the run to S1_DEBOUNCE
// and Stage-1, whose 900 PWM push has a 5 s timeout (DM_AUTO_S1_TIMEOUT_MS) that starts again on
// every such round. With the guard only checked in S2_PUSH / S2_RETRACT, a key that read 'external
// only' with a short 'both' now and then kept Stage-1 pushing into a blocked gear for good: the
// 'both' pass that resumed the push drove nothing and so restarted the stall window, and every gap
// over ML_STEP_MAX_MS left the time budget where it was. Now a stage's time budget counts every
// pass from the stage's start, the key's excursions included, and the stall window every pass on
// which the autoload drives (push or retract), Stage-1's push included; a pass that drives nothing
// neither grows nor restarts it. An auto-unload (the buffer lifted to 80% and let back, which
// motor_motion_run allows in the idle control) drives the channel instead of run(), so the DM block
// does not run while it retracts: its passes go to the run's guard as passes that drive nothing
// (dm_s2_auto_unload_pass in Motion_control.cpp). Left out, each such gesture was a gap that restarted
// the window and kept its time out of the budget, and with key blips restarting Stage-1's 5 s it gave
// a gear that does not turn another second of 900 PWM. Only a gap of more than ML_STEP_MAX_MS (200 ms)
// between two passes is left out, and restarts the window: the channel was not run at all, for example
// offline, with its AS5600 failed, or with the printer holding it outside the idle control, or its DM
// block was not, because a jam latch brakes the idle control before it (jam_latch_brakes(); the latch
// needs on_use in the same insertion, which the bound below excludes). A run the
// key interrupted also goes on through Stage-1 from IDLE ('external only' after the other state),
// which dm_autoload_gate otherwise allows once per insertion only: without that, such a run could only
// wait in IDLE until its budget ran out.
//
// For one insertion (from key 'none' to the next 'none', which ends any run), with the link up, the
// channel's AS5600 read and the printer leaving the channel in its idle control, whatever the key, the
// buffer and the auto-unloads do in between:
// - a gear that does not turn: Stage-1 before the run pushes for at most DM_AUTO_S1_TIMEOUT_MS (it
//   starts from IDLE only once, dm_autoload_gate). There is one run (it cannot finish, and a new one
//   needs key 'none' or a change of the loaded state), and it drives until its stall window has
//   counted ML_STALL_MS: with 1 ms passes at most 5.998 s of 900 PWM in all (4.999 s of Stage-1,
//   then 0.999 s of push or retract; test/test_dm_stage2), however often the buffer aborts. The fail
//   latch then keeps the motor off;
// - any gear: each stage ends at most 12 s after its start (its time budget counts every pass, an
//   auto-unload's included), and a run has at most five stages (three pushes, two retracts): the
//   third buffer abort fails it. A run that stays interrupted for the rest of its stage's budget
//   fails too (red, motor off, until key 'none' or a finished printer load): for example with the key
//   at the other state for 12 s.
// The auto-unload's own 850 PWM retract is not the autoload's drive and is not in these figures: each
// one takes a gesture and ends by its own limits (the buffer below 35%, 1.5 s after the key left
// 'both', 15 s).
// The stall window charges each pass that drives with the time since the pass before it. That is
// the time the motor drives when the passes are evenly spaced (a main-loop pass takes under 1 ms,
// about 36 ms at worst: watchdog_cfg.h); a pass more than ML_STEP_MAX_MS after the one before it is
// a gap and is left out. Key 'none' on a single pass ends the insertion, as before: a key that
// glitches to 'none' every few seconds gets Stage-1's 5 s push each time (not changed here).
#include <stdbool.h>
#include <stdint.h>

#include "dm_rearm.h"
#include "motion_limits.h"

// The Stage-2 stages a run can be interrupted in (S2_PUSH, S2_RETRACT).
#define DM_S2_STAGE_NONE    0u  // no run interrupted: the next 'both' starts a new one
#define DM_S2_STAGE_PUSH    1u
#define DM_S2_STAGE_RETRACT 2u

// A key excursion shorter than the re-arm debounce (dm_rearm.h) is a dip, not a filament move: an
// interrupted retract resumes as if it had not left. After a longer one the run goes on with a
// push. S1_DEBOUNCE itself takes DM_AUTO_S1_DEBOUNCE_MS (100 ms) before Stage-1 pushes, so a run
// that Stage-1 brings back to 'both' always goes on with a push.
#define DM_S2_RESUME_MS DM_REARM_AWAY_MS

// What dm_s2_enter() does with the run's guard (dm_s2_guard) for the stage it returns.
#define DM_S2_GUARD_NEW   0u  // a new run: its guard starts (ml_dm_s2_start)
#define DM_S2_GUARD_STAGE 1u  // the run goes on with a new stage: dm_s2_guard_pass(), new_stage
#define DM_S2_GUARD_KEEP  2u  // the interrupted stage resumes: dm_s2_guard_pass()

typedef struct
{
    uint8_t  stage;  // DM_S2_STAGE_*: the stage the current excursion interrupted
    uint64_t t0_ms;  // the pass the key left 'both' on
} dm_s2_run_t;

// The run is over or cannot go on: finished, failed, key 'none', the loaded state changed, the
// channel is not wired. The next 'both' starts a new run.
static inline void dm_s2_end(dm_s2_run_t *r)
{
    r->stage = DM_S2_STAGE_NONE;
    r->t0_ms = 0u;
}

// The key left 'both' in Stage-2 stage `stage` (DM_S2_STAGE_PUSH or _RETRACT): the run is
// interrupted, its remaining length and abort count stay as they are.
static inline void dm_s2_leave(dm_s2_run_t *r, uint8_t stage, uint64_t now_ms)
{
    r->stage = stage;
    r->t0_ms = now_ms;
}

// The key reads 'both' where Stage-2 starts (IDLE, or S1_PUSH). remain_m / tries / last_cnt:
// dm_auto_remain_m, dm_auto_try and dm_auto_last_cnt; a new run gets s2_len_m and 0, an interrupted
// one keeps them, less the forward gear travel since last_cnt. *last_cnt becomes pos_cnt
// (as5600_count, which falls while the gear pushes). Returns the stage to run (DM_S2_STAGE_PUSH or
// _RETRACT); *guard: DM_S2_GUARD_* for it.
static inline uint8_t dm_s2_enter(dm_s2_run_t *r, float *remain_m, uint8_t *tries, uint32_t *last_cnt,
                                  float s2_len_m, uint32_t pos_cnt, uint64_t now_ms, uint8_t *guard)
{
    const uint8_t  stage = r->stage;
    const uint64_t t0_ms = r->t0_ms;
    const int32_t  fwd   = (int32_t)(*last_cnt - pos_cnt);
    dm_s2_end(r);
    *last_cnt = pos_cnt;

    if (stage == DM_S2_STAGE_NONE)
    {
        *guard    = DM_S2_GUARD_NEW;
        *remain_m = s2_len_m;
        *tries    = 0u;
        return DM_S2_STAGE_PUSH;
    }
    if (fwd > 0)
    {
        const float rem = *remain_m - ml_cnt_to_m((uint32_t)fwd);
        *remain_m = (rem > 0.0f) ? rem : 0.0f;
    }
    if ((stage == DM_S2_STAGE_PUSH) || ((now_ms - t0_ms) < DM_S2_RESUME_MS))
    {
        *guard = DM_S2_GUARD_KEEP;
        return stage;
    }
    *guard = DM_S2_GUARD_STAGE;
    return DM_S2_STAGE_PUSH;
}

// One pass for the run's guard (dm_s2_guard): every pass of the run after the one that started the
// guard, i.e. every pass of S2_PUSH / S2_RETRACT and every pass while the key keeps the run
// interrupted (IDLE, S1_DEBOUNCE, S1_PUSH). pwm: what the autoload drives on the pass (0 when it
// drives nothing). new_stage: a later stage of the run starts on this pass (the retract after a
// buffer abort, the push after it, or the push an interrupted retract goes on with). Its time budget
// starts again here; the stall window goes on, and the 1 mm the gear must move to restart it is
// measured from here.
// A pass that drives at least ML_STALL_PWM (the stage's own 900 PWM, or Stage-1's push) is a plain
// motion_guard_check(). Any other pass (the key away from 'both' with Stage-1 not pushing, or a
// pass that resumes or starts a stage without driving) only adds its time to the budget: the stall
// window neither grows nor restarts on it, unless it follows a gap over ML_STEP_MAX_MS. Returns
// ML_OK or a limit.
static inline ml_result dm_s2_guard_pass(motion_guard *g, uint64_t now_ms, uint32_t pos_cnt, float pwm,
                                         bool new_stage)
{
    if (new_stage)
    {
        g->run_ms    = 0u;
        g->start_cnt = pos_cnt;
        g->stall_cnt = pos_cnt;
    }
    if (ml_absf(pwm) >= ML_STALL_PWM) return motion_guard_check(g, now_ms, pos_cnt, pwm);

    const bool     gap       = (now_ms - g->last_ms) > ML_STEP_MAX_MS;
    const uint32_t stall_ms  = g->stall_ms;
    const uint32_t stall_cnt = g->stall_cnt;
    const ml_result r = motion_guard_check(g, now_ms, pos_cnt, 0.0f);
    if (!gap)
    {
        g->stall_ms  = stall_ms;
        g->stall_cnt = stall_cnt;
    }
    return r;
}
