// Host tests for src/dm_stage2.h and the DM autoload (BMCU_DM_TWO_MICROSWITCH) of Motion_control.cpp
// that uses it. The state machine cannot be linked on the host, so this test carries verbatim copies
// of its globals and Stage-2 helpers, of run()'s DM block, of motor_motion_run's DM pass, of
// Motion_control_init's DM reset and of the filament position enum (scripts/check_test_copies.py
// checks that they still match src/), and drives them in 1 ms passes for one channel in the idle
// control, with a simulated gear, filament tip, key and buffer. The auto-unload that a buffer-lift
// gesture starts, and that motor_motion_run then runs instead of run(), is the firmware's own
// src/auto_unload.h, called as motor_motion_run calls it.
//
// Stage-2 pushes 120 mm at 900 PWM once the key reads 'both'. Key dips away from 'both' (one pass
// to 'external only' or the other state, up to 99 ms, or longer ones that Stage-1 ends) must not
// start it over: the 120 mm countdown, the three-abort fail latch, the push stage's time budget and
// the run's stall window must go on, so the push still ends after 120 mm, a blocked or slow gear
// still stops (also when the buffer aborts: the stall window goes on across the run's stages), and a
// buffer that rises on every push still fails after three aborts, also when each retract takes the
// tip back behind the inner switch. A dip during a retract resumes the retract. Only the filament
// leaving both switches, the end of the run, or a printer load start a new run. Nor may auto-unloads
// the buffer-lift gesture starts meanwhile give a blocked gear more push, or a stage more time: their
// passes count as time for the run's guard.

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <unity.h>

#include "ams.h"
#include "auto_unload.h"
#include "dm_rearm.h"
#include "dm_stage2.h"
#include "motion_limits.h"

// ---- What the copied firmware code uses (Motion_control.cpp names) ----
static constexpr uint8_t kChCount = 4;
_ams ams[ams_max_number];
#define motion_control_ams_num 0

// ---- Motion_control.cpp at this commit: filament_now_position_enum, verbatim ----
enum filament_now_position_enum
{
    filament_idle,
    filament_sending_out,
    filament_using,
    filament_before_pull_back,
    filament_pulling_back,
    filament_redetect,
};
// ---- end of the Motion_control.cpp copy ----

static filament_now_position_enum filament_now_position[4];
static bool     filament_channel_inserted[4];
static uint8_t  MC_ONLINE_key_stu[4];
static float    MC_PULL_pct_f[4];
static uint32_t as5600_count[4];

static uint8_t led_r, led_g; // red and green of channel 0's last status LED colour
static void MC_STU_RGB_set(uint8_t ch, uint8_t r, uint8_t g, uint8_t b)
{
    (void)b;
    if (ch == 0u)
    {
        led_r = r;
        led_g = g;
    }
}

// ---- Motion_control.cpp at this commit: DM autoload constants, state, Stage-2 entry and guard, verbatim ----
// ---- DM autoload (two microswitch) ----
static constexpr uint64_t DM_AUTO_S1_DEBOUNCE_MS       = 100ull;   // 0.1s
static constexpr uint64_t DM_AUTO_S1_TIMEOUT_MS        = 5000ull;  // 5s
static constexpr uint64_t DM_AUTO_S1_FAIL_RETRACT_MS   = 1500ull;  // 1.5s

static constexpr float    DM_AUTO_S2_TARGET_M          = 0.120f;   // 120mm
static constexpr float    DM_AUTO_BUF_ABORT_PCT        = 75.0f;    // abort push
static constexpr float    DM_AUTO_BUF_RECOVER_PCT      = 50.2f;    // retract-to (try 1/2)
static constexpr uint64_t DM_AUTO_FAIL_EXTRA_MS        = 1500ull;  // extra retract after fail
static constexpr float    DM_AUTO_PWM_PUSH             = 900.0f;   // push strength
static constexpr float    DM_AUTO_PWM_PULL             = 900.0f;   // retract strength
static constexpr float    DM_AUTO_IDLE_LIM             = 950.0f;   // clamp only during autoload

enum : uint8_t
{
    DM_AUTO_IDLE = 0,
    DM_AUTO_S1_DEBOUNCE,
    DM_AUTO_S1_PUSH,
    DM_AUTO_S1_FAIL_RETRACT,
    DM_AUTO_S2_PUSH,
    DM_AUTO_S2_RETRACT,
    DM_AUTO_S2_FAIL_RETRACT,
    DM_AUTO_S2_FAIL_EXTRA,
};

static uint8_t  dm_loaded[4]            = {1,1,1,1};   // 1=loaded (after stage2 success)
static uint8_t  dm_fail_latch[4]        = {0,0,0,0};   // latch until ks==0 (<0.6V)
static uint8_t  dm_auto_state[4]        = {0,0,0,0};
static uint8_t  dm_autoload_gate[4]     = {0,0,0,0}; // 0=allow Stage1, 1=block Stage1 until idle+ks==0
static uint8_t  dm_auto_try[4]          = {0,0,0,0};   // abort count (stage2)
static uint64_t dm_auto_t0_ms[4]        = {0ull,0ull,0ull,0ull};
static float    dm_auto_remain_m[4]     = {0,0,0,0};
static uint32_t dm_auto_last_cnt[4]     = {0,0,0,0};   // as5600_count at the last Stage-2 update

// A loaded channel's key away from 'both' (dm_rearm.h): first pass (0 = none), gear position then.
static uint64_t dm_loaded_drop_t0_ms[4] = {0ull,0ull,0ull,0ull};
static uint32_t dm_loaded_drop_cnt[4]   = {0u,0u,0u,0u};

// Time and stall limits of S2_PUSH / S2_RETRACT / S2_FAIL_RETRACT (motion_limits.h).
static motion_guard dm_s2_guard[4];
static_assert(DM_AUTO_PWM_PUSH >= ML_STALL_PWM && DM_AUTO_PWM_PULL >= ML_STALL_PWM,
              "DM Stage-2 PWM must be covered by the stall check");

// The Stage-2 run a key excursion away from 'both' interrupted (dm_stage2.h).
static dm_s2_run_t dm_s2_run[4];
static_assert(DM_AUTO_S1_DEBOUNCE_MS >= DM_S2_RESUME_MS, "a retract Stage-1 brought back to 'both' must go on as a push");

// The key reads 'both' where Stage-2 starts (IDLE, S1_PUSH): the Stage-2 state to run, with a new
// run's length and abort count or the interrupted run's. *resumed: that state when it keeps its
// guard, else DM_AUTO_IDLE. *new_run: a new run starts (a new guard), else the run goes on.
static inline uint8_t dm_s2_enter_state(uint8_t ch, uint32_t cur_cnt, uint64_t now_ms, uint8_t *resumed,
                                        bool *new_run)
{
    uint8_t guard = DM_S2_GUARD_NEW;
    const uint8_t stage = dm_s2_enter(&dm_s2_run[ch], &dm_auto_remain_m[ch], &dm_auto_try[ch], &dm_auto_last_cnt[ch],
                                      DM_AUTO_S2_TARGET_M, cur_cnt, now_ms, &guard);
    const uint8_t st = (stage == DM_S2_STAGE_RETRACT) ? DM_AUTO_S2_RETRACT : DM_AUTO_S2_PUSH;
    *resumed = (guard == DM_S2_GUARD_KEEP) ? st : (uint8_t)DM_AUTO_IDLE;
    *new_run = (guard == DM_S2_GUARD_NEW);
    return st;
}

// A limit of the run's guard fails the autoload the way three buffer aborts do: the run is over, and
// the fail latch keeps the motor off and the LED red until key 'none' or a finished printer load.
static inline void dm_s2_guard_fail(uint8_t ch)
{
    dm_s2_end(&dm_s2_run[ch]);
    dm_fail_latch[ch]    = 1u;
    dm_auto_state[ch]    = DM_AUTO_IDLE;
    dm_auto_try[ch]      = 0u;
    dm_auto_remain_m[ch] = 0.0f;
    dm_auto_t0_ms[ch]    = 0ull;
}

// A pass on which the auto-unload (the buffer lifted by hand, motor_motion_run) drives channel ch
// instead of run(), so that the DM block does not run; idle_ctrl: the channel runs its idle control,
// where the DM block would have run. Left out, such passes are a gap for the guard of a run that is in
// a Stage-2 stage or interrupted (more than ML_STEP_MAX_MS after its last pass), which restarts the
// stall window and leaves the time out of the stage's budget: with key blips restarting Stage-1's 5 s,
// every buffer-lift gesture gave a gear that does not turn up to another second of 900 PWM. So each of
// them is a pass of the run's guard that drives nothing (dm_s2_guard_pass, dm_stage2.h): its time
// counts towards the stage's budget, and the stall window neither grows nor restarts on it. The
// auto-unload's 850 PWM retract is the person's, with its own limits, not the autoload's drive, and a
// gear it moves 1 mm is not stalled: the window restarts at the autoload's next drive, as after a pull
// by hand. A limit fails the run as in the DM block.
static inline void dm_s2_auto_unload_pass(uint8_t ch, bool idle_ctrl, uint64_t now_ms)
{
    if (!idle_ctrl || (dm_loaded[ch] != 0u) || dm_fail_latch[ch]) return;

    const uint8_t st = dm_auto_state[ch];
    const bool s2_stage = (st == DM_AUTO_S2_PUSH) || (st == DM_AUTO_S2_RETRACT) || (st == DM_AUTO_S2_FAIL_RETRACT);
    if (!s2_stage && (dm_s2_run[ch].stage == DM_S2_STAGE_NONE)) return;

    if (dm_s2_guard_pass(&dm_s2_guard[ch], now_ms, as5600_count[ch], 0.0f, false) != ML_OK) dm_s2_guard_fail(ch);
}
// ---- end of the Motion_control.cpp copy ----

// run() clamps what the autoload drives to DM_AUTO_IDLE_LIM (outside the copied block), so the
// simulated gear takes the Stage-1/2 PWM as it is.
static_assert((DM_AUTO_IDLE_LIM >= DM_AUTO_PWM_PUSH) && (DM_AUTO_IDLE_LIM >= DM_AUTO_PWM_PULL),
              "run() would clamp the autoload PWM");

// run() for channel CHx in the idle control: its DM block. Returns what it drives (0 = the idle
// control runs instead; Stage-1/2 drive 0 or +-900).
static float dm_run(int CHx, uint64_t now_ms)
{
    const float dir = 1.0f; // MOTOR_CONTROL[CHx].dir: a retract is +, a push -
    bool  dm_autoload_active = false;
    float dm_autoload_x      = 0.0f;

// ---- Motion_control.cpp at this commit: run()'s DM autoload block, verbatim ----
                    if (filament_channel_inserted[CHx] && (dm_loaded[CHx] == 0u))
                    {
                        const uint8_t ks = MC_ONLINE_key_stu[CHx];
                        const uint32_t cur_cnt = as5600_count[CHx];

                        if (dm_fail_latch[CHx])
                        {
                            dm_autoload_active = true;
                            dm_autoload_x = 0.0f;
                            MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);
                        }
                        else
                        {
                            const uint8_t dm_state_at_entry = dm_auto_state[CHx];
                            uint8_t dm_s2_resumed = DM_AUTO_IDLE; // Stage-2 state resumed with its guard
                            bool    dm_s2_new_run = false;        // a new Stage-2 run starts on this pass

                            if (dm_auto_state[CHx] == DM_AUTO_IDLE)
                            {
                                if (ks == 2u)
                                {
                                    // Stage-1 once per insertion, and for a run the key interrupted (dm_stage2.h)
                                    if ((dm_autoload_gate[CHx] == 0u) || (dm_s2_run[CHx].stage != DM_S2_STAGE_NONE))
                                    {
                                        dm_autoload_gate[CHx] = 1u;
                                        dm_auto_state[CHx] = DM_AUTO_S1_DEBOUNCE;
                                        dm_auto_t0_ms[CHx] = now_ms;
                                    }
                                }
                                else if (ks == 1u)
                                {
                                    dm_auto_state[CHx] =
                                        dm_s2_enter_state(CHx, cur_cnt, now_ms, &dm_s2_resumed, &dm_s2_new_run);
                                }
                            }

                            switch (dm_auto_state[CHx])
                            {
                            case DM_AUTO_S1_DEBOUNCE:
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0xFF, 0x00);

                                if (ks != 2u)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_IDLE;
                                    dm_auto_t0_ms[CHx] = 0ull;
                                }
                                else if ((now_ms - dm_auto_t0_ms[CHx]) >= DM_AUTO_S1_DEBOUNCE_MS)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_S1_PUSH;
                                    dm_auto_t0_ms[CHx] = now_ms;
                                }
                                break;

                            case DM_AUTO_S1_PUSH:
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0xFF, 0x00);

                                if (ks == 0u)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_IDLE;
                                    dm_auto_t0_ms[CHx] = 0ull;
                                }
                                else if (ks == 1u)
                                {
                                    dm_auto_state[CHx] =
                                        dm_s2_enter_state(CHx, cur_cnt, now_ms, &dm_s2_resumed, &dm_s2_new_run);
                                }
                                else if ((now_ms - dm_auto_t0_ms[CHx]) >= DM_AUTO_S1_TIMEOUT_MS)
                                {
                                    dm_fail_latch[CHx] = 1u;
                                    dm_auto_state[CHx] = DM_AUTO_S1_FAIL_RETRACT;
                                    dm_auto_t0_ms[CHx] = now_ms;
                                }
                                else
                                {
                                    dm_autoload_x = -dir * DM_AUTO_PWM_PUSH;
                                }
                                break;

                            case DM_AUTO_S1_FAIL_RETRACT:
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);

                                if (ks == 0u)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_IDLE;
                                    dm_auto_t0_ms[CHx] = 0ull;
                                }
                                else if ((now_ms - dm_auto_t0_ms[CHx]) >= DM_AUTO_S1_FAIL_RETRACT_MS)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_IDLE;
                                    dm_auto_t0_ms[CHx] = 0ull;
                                }
                                else
                                {
                                    dm_autoload_x = dir * DM_AUTO_PWM_PULL;
                                }
                                break;

                            case DM_AUTO_S2_PUSH:
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0xFF, 0x00);

                                if (ks != 1u)
                                {
                                    // The run is interrupted, not over: length and aborts are kept.
                                    dm_s2_leave(&dm_s2_run[CHx], DM_S2_STAGE_PUSH, now_ms);

                                    if (ks == 2u)
                                    {
                                        dm_auto_state[CHx] = DM_AUTO_S1_DEBOUNCE;
                                        dm_auto_t0_ms[CHx] = now_ms;
                                    }
                                    else
                                    {
                                        dm_auto_state[CHx] = DM_AUTO_IDLE;
                                        dm_auto_t0_ms[CHx] = 0ull;
                                    }
                                    break;
                                }

                                // remain -= moved
                                {
                                    const float moved = ml_travel_m(cur_cnt, dm_auto_last_cnt[CHx]);
                                    dm_auto_last_cnt[CHx] = cur_cnt;

                                    float r = dm_auto_remain_m[CHx] - moved;
                                    if (r < 0.0f) r = 0.0f;
                                    dm_auto_remain_m[CHx] = r;
                                }

                                if (MC_PULL_pct_f[CHx] > DM_AUTO_BUF_ABORT_PCT)
                                {
                                    uint8_t t = dm_auto_try[CHx];
                                    if (t < 255u) t++;
                                    dm_auto_try[CHx] = t;

                                    dm_auto_last_cnt[CHx] = cur_cnt;

                                    if (t >= 3u)
                                    {
                                        dm_fail_latch[CHx] = 1u;
                                        dm_auto_state[CHx] = DM_AUTO_S2_FAIL_RETRACT;
                                    }
                                    else
                                    {
                                        dm_auto_state[CHx] = DM_AUTO_S2_RETRACT;
                                    }

                                    MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);
                                }
                                else if (dm_auto_remain_m[CHx] <= 0.0f)
                                {
                                    dm_loaded[CHx] = 1u;

                                    dm_auto_state[CHx]    = DM_AUTO_IDLE;
                                    dm_auto_try[CHx]      = 0u;
                                    dm_auto_remain_m[CHx] = 0.0f;
                                    dm_auto_t0_ms[CHx]    = 0ull;

                                    MC_STU_RGB_set(CHx, 0x38, 0x35, 0x32);
                                    dm_autoload_x = 0.0f;
                                }
                                else
                                {
                                    dm_autoload_x = -dir * DM_AUTO_PWM_PUSH;
                                }
                                break;

                            case DM_AUTO_S2_RETRACT:
                                dm_autoload_active = true;

                                if (ks == 0u)
                                {
                                    dm_auto_state[CHx]    = DM_AUTO_IDLE;
                                    dm_auto_try[CHx]      = 0u;
                                    dm_auto_remain_m[CHx] = 0.0f;
                                    dm_auto_t0_ms[CHx]    = 0ull;
                                    break;
                                }

                                // remain += moved
                                {
                                    const float moved = ml_travel_m(cur_cnt, dm_auto_last_cnt[CHx]);
                                    dm_auto_last_cnt[CHx] = cur_cnt;

                                    float r = dm_auto_remain_m[CHx] + moved;
                                    if (r > DM_AUTO_S2_TARGET_M) r = DM_AUTO_S2_TARGET_M;
                                    dm_auto_remain_m[CHx] = r;
                                }

                                MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);

                                if ((MC_PULL_pct_f[CHx] <= DM_AUTO_BUF_RECOVER_PCT) || (ks == 2u))
                                {
                                    dm_auto_last_cnt[CHx] = cur_cnt;

                                    if (ks == 1u)
                                    {
                                        dm_auto_state[CHx] = DM_AUTO_S2_PUSH;
                                    }
                                    else if (ks == 2u)
                                    {
                                        dm_s2_leave(&dm_s2_run[CHx], DM_S2_STAGE_RETRACT, now_ms);
                                        dm_auto_state[CHx] = DM_AUTO_S1_DEBOUNCE;
                                        dm_auto_t0_ms[CHx] = now_ms;
                                    }
                                    else
                                    {
                                        dm_s2_leave(&dm_s2_run[CHx], DM_S2_STAGE_RETRACT, now_ms);
                                        dm_auto_state[CHx] = DM_AUTO_IDLE;
                                        dm_auto_t0_ms[CHx] = 0ull;
                                    }
                                    dm_autoload_x = 0.0f;
                                }
                                else
                                {
                                    dm_autoload_x = dir * DM_AUTO_PWM_PULL;
                                }
                                break;

                            case DM_AUTO_S2_FAIL_RETRACT:
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);

                                if (ks == 0u)
                                {
                                    dm_auto_state[CHx]    = DM_AUTO_IDLE;
                                    dm_auto_try[CHx]      = 0u;
                                    dm_auto_remain_m[CHx] = 0.0f;
                                    dm_auto_t0_ms[CHx]    = 0ull;
                                }
                                else if (ks == 2u)
                                {
                                    dm_auto_state[CHx] = DM_AUTO_S2_FAIL_EXTRA;
                                    dm_auto_t0_ms[CHx] = now_ms;
                                }
                                else
                                {
                                    dm_autoload_x = dir * DM_AUTO_PWM_PULL;
                                }
                                break;

                            case DM_AUTO_S2_FAIL_EXTRA:
                                dm_autoload_active = true;
                                MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);

                                if (ks == 0u)
                                {
                                    dm_auto_state[CHx]    = DM_AUTO_IDLE;
                                    dm_auto_try[CHx]      = 0u;
                                    dm_auto_remain_m[CHx] = 0.0f;
                                    dm_auto_t0_ms[CHx]    = 0ull;
                                }
                                else if ((now_ms - dm_auto_t0_ms[CHx]) >= DM_AUTO_FAIL_EXTRA_MS)
                                {
                                    dm_auto_state[CHx]    = DM_AUTO_IDLE;
                                    dm_auto_try[CHx]      = 0u;
                                    dm_auto_remain_m[CHx] = 0.0f;
                                    dm_auto_t0_ms[CHx]    = 0ull;
                                }
                                else
                                {
                                    dm_autoload_x = dir * DM_AUTO_PWM_PULL;
                                }
                                break;

                            case DM_AUTO_IDLE:
                                // Before Stage-1 or Stage-2, or a Stage-2 run the key interrupted: its
                                // length and aborts are kept for the next 'both' (dm_stage2.h).
                                break;

                            default:
                                dm_s2_end(&dm_s2_run[CHx]);
                                dm_auto_state[CHx]    = DM_AUTO_IDLE;
                                dm_auto_try[CHx]      = 0u;
                                dm_auto_remain_m[CHx] = 0.0f;
                                dm_auto_t0_ms[CHx]    = 0ull;
                                break;
                            }

                            // Stage-2 stages end only on a switch or buffer event (the push also after 120 mm
                            // of gear travel), so a blocked gear or a buffer that never relaxes kept 900 PWM on
                            // for good. These states are only entered inside this block, so one that differs
                            // from the state at the start of the pass has just been entered, unless it is the
                            // stage a key excursion interrupted, resumed on this pass with its guard. A new
                            // run's first stage starts the run's guard; a later stage (after a buffer abort, or
                            // the push an interrupted retract goes on with) gets a new time budget, and the
                            // run's stall window goes on. While the key keeps a run interrupted, its guard goes
                            // on on every pass as well (IDLE, S1_DEBOUNCE, and Stage-1's push back to 'both'),
                            // so no round of Stage-1 restarts it (dm_stage2.h), and so it does on the passes an
                            // auto-unload drives instead of run() (dm_s2_auto_unload_pass). A limit fails the
                            // autoload the way three buffer aborts do (dm_s2_guard_fail): motor off, red, until
                            // ks == 0 or a finished printer load. (S2_FAIL_RETRACT does not run today: the fail
                            // latch set with it skips this state machine.)
                            const uint8_t st = dm_auto_state[CHx];
                            const bool s2_stage =
                                (st == DM_AUTO_S2_PUSH) || (st == DM_AUTO_S2_RETRACT) || (st == DM_AUTO_S2_FAIL_RETRACT);
                            const bool s2_interrupted = (dm_s2_run[CHx].stage != DM_S2_STAGE_NONE);
                            const bool s2_entered = s2_stage && (st != dm_state_at_entry) && (st != dm_s2_resumed);
                            if (s2_entered && dm_s2_new_run)
                            {
                                ml_dm_s2_start(&dm_s2_guard[CHx], now_ms, as5600_count[CHx], DM_AUTO_S2_TARGET_M);
                            }
                            else if ((s2_stage || s2_interrupted) &&
                                     (dm_s2_guard_pass(&dm_s2_guard[CHx], now_ms, as5600_count[CHx], dm_autoload_x,
                                                       s2_entered) != ML_OK))
                            {
                                dm_s2_guard_fail(CHx);
                                dm_autoload_x = 0.0f;
                                MC_STU_RGB_set(CHx, 0xFF, 0x00, 0x00);
                            }
                        }
                    }
// ---- end of the Motion_control.cpp copy ----

    return dm_autoload_active ? dm_autoload_x : 0.0f;
}

static void dm_motor_motion_run(uint64_t time_now)
{
// ---- Motion_control.cpp at this commit: motor_motion_run's DM autoload pass, verbatim ----
    const auto &A = ams[motion_control_ams_num];

    for (uint8_t ch = 0; ch < kChCount; ch++)
    {
        if (!filament_channel_inserted[ch])
        {
            dm_loaded[ch]            = 1u;
            dm_fail_latch[ch]        = 0u;
            dm_auto_state[ch]        = DM_AUTO_IDLE;
            dm_auto_try[ch]          = 0u;
            dm_auto_t0_ms[ch]        = 0ull;
            dm_auto_remain_m[ch]     = 0.0f;
            dm_auto_last_cnt[ch]     = 0u;
            dm_loaded_drop_t0_ms[ch] = 0ull;
            dm_autoload_gate[ch]     = 0u;
            dm_s2_end(&dm_s2_run[ch]);
            continue;
        }

        const uint8_t ks = MC_ONLINE_key_stu[ch];

        // dm_loaded (dm_rearm.h): a loaded channel is unloaded, which arms Stage-2 again, only once
        // the filament left both switches or the gear retracted it out of 'both'; a finished printer
        // load marks it loaded.
        const filament_now_position_enum pos = filament_now_position[ch];
        const dm_host_t host = dm_host_from_motion(A.now_filament_num == ch, A.filament[ch].motion,
                                                   (pos == filament_pulling_back) || (pos == filament_redetect));
        const dm_rearm_event ev = dm_rearm_pass(&dm_loaded[ch], &dm_loaded_drop_t0_ms[ch], &dm_loaded_drop_cnt[ch],
                                                ks, host, time_now, as5600_count[ch]);

        if (ks == 0u)
        {
            if (filament_now_position[ch] == filament_idle)
                dm_autoload_gate[ch] = 0u;

            dm_fail_latch[ch]        = 0u;
            dm_auto_state[ch]        = DM_AUTO_IDLE;
            dm_auto_try[ch]          = 0u;
            dm_auto_t0_ms[ch]        = 0ull;
            dm_auto_remain_m[ch]     = 0.0f;
            dm_auto_last_cnt[ch]     = 0u;
            dm_s2_end(&dm_s2_run[ch]); // the filament is out: the next 'both' starts a new run
            continue;
        }

        // A printer load that finished proved the path, so the Stage-2 failure latch has no reason to
        // keep the LED red (the channel runs the normal idle control once loaded).
        if (ev == DM_REARM_LOADED)
            dm_fail_latch[ch] = 0u;

        if (ev != DM_REARM_NONE) // loaded changed: the autoload starts over from IDLE
        {
            dm_auto_state[ch]    = DM_AUTO_IDLE;
            dm_auto_try[ch]      = 0u;
            dm_auto_t0_ms[ch]    = 0ull;
            dm_auto_remain_m[ch] = 0.0f;
            dm_auto_last_cnt[ch] = 0u;
            dm_s2_end(&dm_s2_run[ch]);
        }
    }
// ---- end of the Motion_control.cpp copy ----
}

static void dm_init(void)
{
// ---- Motion_control.cpp at this commit: Motion_control_init's DM autoload reset, verbatim ----
        for (uint8_t ch = 0; ch < kChCount; ch++)
        {
            if (!filament_channel_inserted[ch])
            {
                dm_loaded[ch]            = 1u;
                dm_fail_latch[ch]        = 0u;
                dm_auto_state[ch]        = DM_AUTO_IDLE;
                dm_auto_try[ch]          = 0u;
                dm_auto_t0_ms[ch]        = 0ull;
                dm_auto_remain_m[ch]     = 0.0f;
                dm_auto_last_cnt[ch]     = 0u;
                dm_loaded_drop_t0_ms[ch] = 0ull;
                dm_autoload_gate[ch]     = 0u;
                dm_s2_end(&dm_s2_run[ch]);
                continue;
            }

            const uint8_t ks = MC_ONLINE_key_stu[ch];

            dm_autoload_gate[ch] = (ks != 0u) ? 1u : 0u;
            dm_loaded[ch] = (ks == 1u) ? 1u : 0u;

            dm_fail_latch[ch]        = 0u;
            dm_auto_state[ch]        = DM_AUTO_IDLE;
            dm_auto_try[ch]          = 0u;
            dm_auto_t0_ms[ch]        = 0ull;
            dm_auto_remain_m[ch]     = 0.0f;
            dm_auto_last_cnt[ch]     = 0u;
            dm_loaded_drop_t0_ms[ch] = 0ull;
            dm_s2_end(&dm_s2_run[ch]);
        }
// ---- end of the Motion_control.cpp copy ----
}

// ---- Motion_control.cpp at this commit: the auto-unload and manual pull strengths, verbatim ----
static constexpr float    AUTO_UNLOAD_PWM_PULL       = 850.0f;
static constexpr float    MANUAL_EMPTY_PULL_PWM      = 700.0f;
// ---- end of the Motion_control.cpp copy ----

// ---- adapted from Motion_control.cpp: motor_motion_run's auto-unload call and drive choice ----
// What motor_motion_run does for a channel after its DM pass and the status LED's baseline, with the
// link up and the AS5600 read: auto_unload_pass() decides whether the auto-unload (a retract at
// AUTO_UNLOAD_PWM_PULL, then the Stage-2 guard's pass dm_s2_auto_unload_pass) or the manual empty
// pull (a retract at 700 PWM) drives the channel, or, for AU_DRIVE_NONE, run() with the DM block.
// printer_idle stands for the channel's MOTOR_CONTROL motion being the idle control.
static bool printer_idle;   // the printer commands idle: the channel runs its idle control

static auto_unload_t g_auto_unload[4];

static au_drive_t au_pass(uint8_t i, uint64_t time_now)
{
        au_in_t au;
        au.online    = true;
        au.inserted  = filament_channel_inserted[i];
        au.idle_ctrl = printer_idle;
        au.pct       = MC_PULL_pct_f[i];
        au.ks        = MC_ONLINE_key_stu[i];
        au.now_ms    = time_now;
        const au_drive_t drive = auto_unload_pass(&g_auto_unload[i], &au);

        if (drive == AU_DRIVE_UNLOAD) dm_s2_auto_unload_pass(i, printer_idle, time_now);
        return drive;
}

// ---- Simulation ----
#define KS_NONE  0u
#define KS_BOTH  1u
#define KS_EXT   2u // external switch only
#define KS_OTHER 3u

#define S2_LEN_MM 120.0
#define ASSERT_MM_WITHIN(delta, expected, actual) \
    TEST_ASSERT_FLOAT_WITHIN((float)(delta), (float)(expected), (float)(actual))
#define EXT_LEN_MM 30.0 // tip positions from -30 mm to the inner switch read 'external only'

static uint64_t now;        // ms
static double tip_mm;       // filament tip, mm past the inner switch (< 0: behind it)
static double gear_mm;      // gear travel, + = retract (as5600_count rises on a retract)
static uint32_t cnt0;       // as5600_count at gear_mm = 0
static double v_mm_s;       // gear speed at 900 PWM (0: gear blocked)
static double block_mm;     // the path ahead is blocked where the tip reaches this
static double backlash_mm;  // buffer slider friction: it follows a retract only this much later
static double slider_mm;    // buffer compression the slider shows
static int noise_ks;        // the key reads this instead during noise (-1: never)
static uint64_t noise_t0;   // noise: from noise_t0, noise_len ms of every noise_period ms
static uint32_t noise_period, noise_len;
static int forced_ks;       // the key reads this, whatever the tip and the noise (-1: not forced)
static float forced_pct;    // the buffer reads this, whatever the tip (< 0: not forced)

static double pushed_mm;    // gear travel pushed, any state
static double pushed_s2_mm; // of it in S2_PUSH
static double retracted_mm;
static int aborts;          // S2_PUSH -> S2_RETRACT or S2_FAIL_RETRACT (buffer above 75%)
static uint8_t last_s2;     // last Stage-2 state run (S2_PUSH / S2_RETRACT), 0 = none yet
static uint32_t push_ms;    // passes (1 ms each) on which the autoload pushed, whatever the gear did
static uint32_t drive_ms;   // passes on which it drove the motor at all (push or retract)
static uint64_t run_t0;     // first pass of the current Stage-2 run (0: no run live)
static uint64_t stage_t0;   // first pass of the run's current stage (S2_PUSH / S2_RETRACT)
static uint32_t run_stages; // stages the run has had
static uint64_t fail_t0;    // first pass with the fail latch set (0: none since the last 'none')
static double run_pushed0;  // pushed_mm when that run began
static uint32_t runs;       // Stage-2 runs that began since the last 'none' (a new run, not a resumed one)
static double run_gear0;    // gear_mm when the latest of them began
static bool run_live;       // it has not ended yet (loaded, failed, or key 'none')
static double run_fwd_max;  // the most any run moved the filament net forward from where it began
static uint32_t au_ms;      // passes on which the auto-unload or the manual pull drove (a retract)
static uint32_t au_starts;  // auto-unloads started
static uint64_t au_last;    // last pass the auto-unload drove on (0: none yet)

static uint8_t key_from_tip(void)
{
    if (tip_mm >= 0.0) return KS_BOTH;
    if (tip_mm >= -EXT_LEN_MM) return KS_EXT;
    return KS_NONE;
}

static uint8_t key_now(void)
{
    if (forced_ks >= 0) return (uint8_t)forced_ks;
    if ((noise_ks >= 0) && (now >= noise_t0) && (((now - noise_t0) % noise_period) < noise_len))
        return (uint8_t)noise_ks;
    return key_from_tip();
}

// Buffer: 50% at rest; the filament the gear feeds past a block compresses it, 2.5% per mm (10 mm:
// 75%, Stage-2's abort level). The slider follows a push at once and a retract only once it has
// undone backlash_mm.
static float buffer_pct(void)
{
    const double c = tip_mm - block_mm;
    if (slider_mm < c) slider_mm = c;
    if (slider_mm > c + backlash_mm) slider_mm = c + backlash_mm;
    return (float)(50.0 + 2.5 * ((slider_mm > 0.0) ? slider_mm : 0.0));
}

static void noise(int ks, uint32_t period_ms, uint32_t len_ms)
{
    noise_ks = ks;
    noise_t0 = now;
    noise_period = period_ms;
    noise_len = len_ms;
}

// One main-loop pass: the switches and the buffer are read, motor_motion_run's DM pass runs, then
// what drives the channel: the auto-unload or the manual pull, or run() with its DM block. The gear
// moves for 1 ms at what it drives (the retracts at 850 or 700 PWM a little slower than at 900).
static void pass(void)
{
    MC_ONLINE_key_stu[0] = key_now();
    const float pct = buffer_pct();
    MC_PULL_pct_f[0] = (forced_pct >= 0.0f) ? forced_pct : pct;
    as5600_count[0] = cnt0 + (uint32_t)(int32_t)llround(gear_mm / (double)ML_MM_PER_CNT);

    const uint8_t st0 = dm_auto_state[0];
    const uint8_t try0 = dm_auto_try[0];
    dm_motor_motion_run(now);
    const uint8_t st_mid = dm_auto_state[0];
    const uint8_t run_stage = dm_s2_run[0].stage;

    // The status LED's baseline (stu_apply_baseline): red for a failed channel, else off; what drives
    // the channel sets its colour after it (the auto-unload's is purple).
    led_r = dm_fail_latch[0] ? 0xFFu : 0x00u;
    led_g = 0x00u;
    const uint8_t au_was = g_auto_unload[0].active;
    const au_drive_t au = au_pass(0u, now);
    float x = 0.0f;
    if (au == AU_DRIVE_UNLOAD)
    {
        led_r = 0xA0u;
        led_g = 0x2Du;
    }
    else if (au == AU_DRIVE_NONE)
    {
        x = printer_idle ? dm_run(0, now) : 0.0f;
    }
    if (!au_was && g_auto_unload[0].active) au_starts++;
    const uint8_t st1 = dm_auto_state[0];

    if ((st0 == DM_AUTO_S2_PUSH) && ((st1 == DM_AUTO_S2_RETRACT) || (st1 == DM_AUTO_S2_FAIL_RETRACT))) aborts++;
    if (MC_ONLINE_key_stu[0] == KS_NONE)
    {
        run_t0 = fail_t0 = 0u;
        runs = 0u;
    }
    const bool s2 = (st1 == DM_AUTO_S2_PUSH) || (st1 == DM_AUTO_S2_RETRACT);
    if ((run_t0 == 0u) && s2)
    {
        run_t0 = stage_t0 = now;
        run_stages = 1u;
        run_pushed0 = pushed_mm;
    }
    else if (s2 && ((st1 != last_s2) || (dm_auto_try[0] > try0)))
    {
        // A resumed stage is the same one; any other change is a new stage (a buffer abort, the push
        // after a retract, or the push an interrupted retract goes on with). A buffer abort raises
        // dm_auto_try, so it counts even when the pass ends in the state it began in: an interrupted
        // retract that goes on as a push which aborts on that pass is two stages (the push, which
        // lasted no pass, and the new retract).
        stage_t0 = now;
        run_stages += (st1 == last_s2) ? 2u : 1u;
    }
    if (s2) last_s2 = st1;
    if ((fail_t0 == 0u) && dm_fail_latch[0]) fail_t0 = now;

    // A new run: Stage-2 entered with no run to go on with (dm_s2_enter).
    const bool mid_s2 =
        (st_mid == DM_AUTO_S2_PUSH) || (st_mid == DM_AUTO_S2_RETRACT) || (st_mid == DM_AUTO_S2_FAIL_RETRACT);
    if (!mid_s2 && (run_stage == DM_S2_STAGE_NONE) && (st1 == DM_AUTO_S2_PUSH))
    {
        runs++;
        run_gear0 = gear_mm;
        run_live = true;
    }
    if (dm_loaded[0] || dm_fail_latch[0] || (MC_ONLINE_key_stu[0] == KS_NONE)) run_live = false;
    // The run record ends with the run (loaded, failed, key 'none' or a dm_rearm change leave no
    // Stage-2 state and no run interrupted), so a later run in the same insertion gets its own.
    if (!s2 && (st1 != DM_AUTO_S2_FAIL_RETRACT) && (dm_s2_run[0].stage == DM_S2_STAGE_NONE)) run_t0 = 0u;

    TEST_ASSERT_TRUE((x == 0.0f) || (x == 900.0f) || (x == -900.0f));
    if (x != 0.0f) drive_ms++;
    const double d = v_mm_s * 0.001;
    if (au != AU_DRIVE_NONE)
    {
        const double da = d * ((au == AU_DRIVE_UNLOAD) ? AUTO_UNLOAD_PWM_PULL : MANUAL_EMPTY_PULL_PWM) / 900.0;
        au_ms++;
        tip_mm -= da;
        gear_mm += da;
        if (au == AU_DRIVE_UNLOAD) au_last = now;
    }
    else if (x < 0.0f)
    {
        push_ms++;
        tip_mm += d;
        gear_mm -= d;
        pushed_mm += d;
        if (st1 == DM_AUTO_S2_PUSH) pushed_s2_mm += d;
    }
    else if (x > 0.0f)
    {
        tip_mm -= d;
        gear_mm += d;
        retracted_mm += d;
    }
    if (run_live && (run_gear0 - gear_mm > run_fwd_max)) run_fwd_max = run_gear0 - gear_mm;
    now++;
}

static void run_for(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) pass();
}

// Passes until the channel is loaded (Stage-2 done) or failed (fail latch), at most max_ms.
// Returns the ms it took, or -1.
static int32_t until_done(uint32_t max_ms)
{
    for (uint32_t i = 0; i < max_ms; i++)
    {
        pass();
        if (dm_loaded[0] || dm_fail_latch[0]) return (int32_t)i + 1;
    }
    return -1;
}

void setUp(void)
{
    now = 10000u;
    tip_mm = -100.0; // no filament
    gear_mm = 0.0;
    cnt0 = 0xFFFF0000u; // the count wraps during the tests
    v_mm_s = 60.0;
    block_mm = 1.0e9;
    backlash_mm = 0.0;
    slider_mm = 0.0;
    noise_ks = -1;
    noise_t0 = 0u;
    noise_period = 1u;
    noise_len = 0u;
    forced_ks = -1;
    forced_pct = -1.0f;
    printer_idle = true;
    pushed_mm = pushed_s2_mm = retracted_mm = 0.0;
    aborts = 0;
    last_s2 = 0u;
    push_ms = drive_ms = 0u;
    run_t0 = fail_t0 = stage_t0 = 0u;
    run_stages = 0u;
    run_pushed0 = 0.0;
    runs = 0u;
    run_gear0 = run_fwd_max = 0.0;
    run_live = false;
    au_ms = au_starts = 0u;
    au_last = 0u;
    led_r = led_g = 0u;
    for (uint8_t ch = 0u; ch < 4u; ch++) auto_unload_reset(&g_auto_unload[ch]);

    for (uint8_t ch = 0; ch < 4u; ch++)
    {
        filament_now_position[ch] = filament_idle;
        filament_channel_inserted[ch] = (ch == 0u);
        MC_ONLINE_key_stu[ch] = 0u;
        MC_PULL_pct_f[ch] = 50.0f;
        as5600_count[ch] = 0u;
        dm_loaded_drop_cnt[ch] = 0u;
    }
    memset(dm_s2_guard, 0, sizeof(dm_s2_guard));
    ams[0].init();

    // Boot with no filament at the channel.
    MC_ONLINE_key_stu[0] = key_from_tip();
    dm_init();
}

void tearDown(void) {}

// The user inserts filament to the outer switch; Stage-1 pushes it to 'both' and Stage-2 starts.
// Returns once S2_PUSH runs, with the counters at 0.
static void insert_and_start_stage2(void)
{
    tip_mm = -10.0;
    for (int i = 0; (i < 2000) && (dm_auto_state[0] != DM_AUTO_S2_PUSH); i++) pass();
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S2_PUSH, dm_auto_state[0]);
    TEST_ASSERT_TRUE(tip_mm >= 0.0);
    pushed_mm = pushed_s2_mm = retracted_mm = 0.0;
    push_ms = drive_ms = 0u;
    aborts = 0;
}

// ---- dm_stage2.h ----

// 10 mm of gear travel in AS5600 counts (as5600_count falls while the gear pushes).
#define CNT_10MM 1739u

static void test_enter_without_an_interrupted_run_starts_a_new_one(void)
{
    dm_s2_run_t r;
    dm_s2_end(&r);
    float remain = 0.03f;
    uint8_t tries = 2u;
    uint32_t last = 5u;
    uint8_t guard = 0xFFu;
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_PUSH, dm_s2_enter(&r, &remain, &tries, &last, 0.120f, 777u, 5000u, &guard));
    TEST_ASSERT_EQUAL_FLOAT(0.120f, remain);
    TEST_ASSERT_EQUAL_UINT8(0u, tries);
    TEST_ASSERT_EQUAL_UINT32(777u, last);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_GUARD_NEW, guard);
}

static void test_an_interrupted_push_goes_on_with_its_length_aborts_and_guard(void)
{
    const uint32_t away[] = {0u, 1u, 99u, 100u, 150u, 5000u, 600000u};
    for (unsigned i = 0; i < sizeof(away) / sizeof(away[0]); i++)
    {
        dm_s2_run_t r;
        dm_s2_end(&r);
        dm_s2_leave(&r, DM_S2_STAGE_PUSH, 7000u);
        float remain = 0.041f;
        uint8_t tries = 2u;
        uint32_t last = 1000u;
        uint8_t guard = 0xFFu;
        TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_PUSH,
                                dm_s2_enter(&r, &remain, &tries, &last, 0.120f, 1000u, 7000u + away[i], &guard));
        TEST_ASSERT_EQUAL_FLOAT(0.041f, remain);
        TEST_ASSERT_EQUAL_UINT8(2u, tries);
        TEST_ASSERT_EQUAL_UINT8(DM_S2_GUARD_KEEP, guard);
    }
}

static void test_an_interrupted_retract_resumes_only_after_a_dip(void)
{
    dm_s2_run_t r;
    float remain = 0.1f;
    uint8_t tries = 1u;
    uint32_t last = 1000u;
    uint8_t guard = 0xFFu;

    dm_s2_end(&r);
    dm_s2_leave(&r, DM_S2_STAGE_RETRACT, 7000u);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_RETRACT,
                            dm_s2_enter(&r, &remain, &tries, &last, 0.120f, 1000u, 7000u + DM_S2_RESUME_MS - 1u, &guard));
    TEST_ASSERT_EQUAL_UINT8(DM_S2_GUARD_KEEP, guard);

    // Away for the re-arm debounce or longer: the run goes on as a push, a new stage of the same run.
    dm_s2_leave(&r, DM_S2_STAGE_RETRACT, 7000u);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_PUSH,
                            dm_s2_enter(&r, &remain, &tries, &last, 0.120f, 1000u, 7000u + DM_S2_RESUME_MS, &guard));
    TEST_ASSERT_EQUAL_UINT8(DM_S2_GUARD_STAGE, guard);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, remain);
    TEST_ASSERT_EQUAL_UINT8(1u, tries);
    TEST_ASSERT_EQUAL_UINT32(100u, DM_S2_RESUME_MS);
}

static void test_forward_travel_during_the_excursion_counts(void)
{
    dm_s2_run_t r;
    uint8_t tries = 0u;
    uint8_t guard = 0xFFu;

    // 10 mm pushed since the last update (the last push pass, Stage-1), across the count wrap.
    float remain = 0.050f;
    uint32_t last = 0x00000100u;
    dm_s2_end(&r);
    dm_s2_leave(&r, DM_S2_STAGE_PUSH, 1000u);
    dm_s2_enter(&r, &remain, &tries, &last, 0.120f, 0x00000100u - CNT_10MM, 1500u, &guard);
    TEST_ASSERT_FLOAT_WITHIN(0.00002f, 0.040f, remain);
    TEST_ASSERT_EQUAL_UINT32(0x00000100u - CNT_10MM, last);

    // More than what was left: nothing left, the push ends on its next pass.
    remain = 0.005f;
    last = 5000u;
    dm_s2_leave(&r, DM_S2_STAGE_PUSH, 2000u);
    dm_s2_enter(&r, &remain, &tries, &last, 0.120f, 5000u - CNT_10MM, 2001u, &guard);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, remain);

    // Pulled back by hand: no change.
    remain = 0.050f;
    last = 5000u;
    dm_s2_leave(&r, DM_S2_STAGE_RETRACT, 3000u);
    dm_s2_enter(&r, &remain, &tries, &last, 0.120f, 5000u + CNT_10MM, 3500u, &guard);
    TEST_ASSERT_EQUAL_FLOAT(0.050f, remain);
    TEST_ASSERT_EQUAL_UINT32(5000u + CNT_10MM, last);
}

static void test_forward_travel_during_a_retract_excursion_counts(void)
{
    // The same for an interrupted retract: gear travel forward since its last pass (Stage-1's push, or
    // the filament pushed in by hand, while the key was away) comes off the remaining length, whether
    // the retract resumes after a dip or the run goes on with a push.
    dm_s2_run_t r;
    uint8_t tries = 1u;
    uint8_t guard = 0xFFu;
    float remain = 0.080f;
    uint32_t last = 5000u;
    dm_s2_end(&r);
    dm_s2_leave(&r, DM_S2_STAGE_RETRACT, 1000u);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_RETRACT, dm_s2_enter(&r, &remain, &tries, &last, 0.120f, 5000u - CNT_10MM,
                                                             1000u + DM_S2_RESUME_MS - 1u, &guard));
    TEST_ASSERT_EQUAL_UINT8(DM_S2_GUARD_KEEP, guard);
    TEST_ASSERT_FLOAT_WITHIN(0.00002f, 0.070f, remain);
    TEST_ASSERT_EQUAL_UINT32(5000u - CNT_10MM, last);

    // 20 mm, across the count wrap, after a longer excursion: a push, a new stage.
    remain = 0.080f;
    last = 0x00000100u;
    dm_s2_leave(&r, DM_S2_STAGE_RETRACT, 2000u);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_PUSH, dm_s2_enter(&r, &remain, &tries, &last, 0.120f,
                                                          0x00000100u - 2u * CNT_10MM, 2000u + DM_S2_RESUME_MS, &guard));
    TEST_ASSERT_EQUAL_UINT8(DM_S2_GUARD_STAGE, guard);
    TEST_ASSERT_FLOAT_WITHIN(0.00002f, 0.060f, remain);
    TEST_ASSERT_EQUAL_UINT8(1u, tries);
}

static void test_each_excursion_is_used_once_and_an_end_forgets_it(void)
{
    dm_s2_run_t r;
    float remain = 0.05f;
    uint8_t tries = 1u;
    uint32_t last = 0u;
    uint8_t guard = 0xFFu;

    dm_s2_end(&r);
    dm_s2_leave(&r, DM_S2_STAGE_PUSH, 1000u);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_PUSH, dm_s2_enter(&r, &remain, &tries, &last, 0.120f, 0u, 1001u, &guard));
    TEST_ASSERT_EQUAL_UINT8(DM_S2_GUARD_KEEP, guard);
    // A second entry with no excursion in between (after the run ended by itself): a new run.
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_PUSH, dm_s2_enter(&r, &remain, &tries, &last, 0.120f, 0u, 1002u, &guard));
    TEST_ASSERT_EQUAL_UINT8(DM_S2_GUARD_NEW, guard);
    TEST_ASSERT_EQUAL_FLOAT(0.120f, remain);

    remain = 0.05f;
    tries = 1u;
    dm_s2_leave(&r, DM_S2_STAGE_PUSH, 2000u);
    dm_s2_end(&r); // key 'none', loaded changed, ...
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_PUSH, dm_s2_enter(&r, &remain, &tries, &last, 0.120f, 0u, 2001u, &guard));
    TEST_ASSERT_EQUAL_UINT8(DM_S2_GUARD_NEW, guard);
    TEST_ASSERT_EQUAL_FLOAT(0.120f, remain);
    TEST_ASSERT_EQUAL_UINT8(0u, tries);
}

// The guard of a Stage-2 stage as ml_dm_s2_start() starts it: 12 s, stall check, no distance limit.
static motion_guard s2_guard(uint64_t t0_ms, uint32_t pos_cnt)
{
    motion_guard g;
    ml_dm_s2_start(&g, t0_ms, pos_cnt, 0.120f);
    TEST_ASSERT_EQUAL_UINT32(12000u, g.max_ms);
    return g;
}

static void test_guard_pass_that_drives_is_the_plain_check(void)
{
    // 900 PWM pushing or retracting against a gear that does not turn: exactly motion_guard_check(),
    // the stall window trips at 1 s.
    motion_guard g = s2_guard(1000u, 5000u);
    motion_guard h = g;
    for (uint32_t t = 1u; t <= 1000u; t++)
    {
        const float pwm = (t & 1u) ? -900.0f : 900.0f;
        const ml_result r = dm_s2_guard_pass(&g, 1000u + t, 5000u, pwm, false);
        TEST_ASSERT_EQUAL_INT(motion_guard_check(&h, 1000u + t, 5000u, pwm), r);
        TEST_ASSERT_EQUAL_INT((t < 1000u) ? ML_OK : ML_STALL, r);
        TEST_ASSERT_EQUAL_UINT32(h.run_ms, g.run_ms);
        TEST_ASSERT_EQUAL_UINT32(h.stall_ms, g.stall_ms);
        TEST_ASSERT_EQUAL_UINT32(h.stall_cnt, g.stall_cnt);
    }
}

static void test_guard_pass_that_does_not_drive_counts_only_time(void)
{
    // 600 ms at 900 PWM against a gear that does not turn, 5 s of passes that drive nothing (the key
    // away from 'both'), then 900 PWM again: the window trips 400 ms later, at 1 s of drive in all,
    // and the 5 s count towards the time budget. (motion_guard_check() would restart the window on
    // the first pass that drives nothing, so it would not trip.)
    motion_guard g = s2_guard(1000u, 7u);
    uint64_t t = 1000u;
    for (int i = 0; i < 600; i++) TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, ++t, 7u, -900.0f, false));
    for (int i = 0; i < 5000; i++) TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, ++t, 7u, 0.0f, false));
    TEST_ASSERT_EQUAL_UINT32(600u, g.stall_ms);
    TEST_ASSERT_EQUAL_UINT32(5600u, g.run_ms);
    for (int i = 0; i < 399; i++) TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, ++t, 7u, -900.0f, false));
    TEST_ASSERT_EQUAL_INT(ML_STALL, dm_s2_guard_pass(&g, ++t, 7u, -900.0f, false));
}

static void test_guard_pass_that_does_not_drive_runs_the_budget_out(void)
{
    motion_guard g = s2_guard(1000u, 0u);
    for (uint32_t t = 1u; t < 12000u; t++) TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, 1000u + t, 0u, 0.0f, false));
    TEST_ASSERT_EQUAL_INT(ML_TIME, dm_s2_guard_pass(&g, 13000u, 0u, 0.0f, false));
}

static void test_guard_pass_leaves_gaps_out_as_motion_guard_check_does(void)
{
    // More than ML_STEP_MAX_MS since the last pass (the autoload did not run: offline, the printer
    // moved the channel): no time added and a new stall window, on a pass that drives nothing too.
    // Exactly ML_STEP_MAX_MS is no gap: it counts in full, and the window is kept.
    motion_guard g = s2_guard(1000u, 0u);
    uint64_t t = 1000u;
    for (int i = 0; i < 900; i++) dm_s2_guard_pass(&g, ++t, 0u, -900.0f, false);
    TEST_ASSERT_EQUAL_UINT32(900u, g.stall_ms);
    t += ML_STEP_MAX_MS;
    TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, t, 0u, 0.0f, false));
    TEST_ASSERT_EQUAL_UINT32(1100u, g.run_ms);
    TEST_ASSERT_EQUAL_UINT32(900u, g.stall_ms);
    t += ML_STEP_MAX_MS + 1u;
    TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, t, 0u, 0.0f, false));
    TEST_ASSERT_EQUAL_UINT32(1100u, g.run_ms);
    TEST_ASSERT_EQUAL_UINT32(0u, g.stall_ms);
    for (int i = 0; i < 999; i++) TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, ++t, 0u, -900.0f, false));
    TEST_ASSERT_EQUAL_INT(ML_STALL, dm_s2_guard_pass(&g, ++t, 0u, -900.0f, false));

    // A pass that drives: 200 ms count towards the window (it trips), 201 ms restart it.
    g = s2_guard(1000u, 0u);
    t = 1000u;
    for (int i = 0; i < 800; i++) dm_s2_guard_pass(&g, ++t, 0u, -900.0f, false);
    t += ML_STEP_MAX_MS + 1u;
    TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, t, 0u, -900.0f, false));
    TEST_ASSERT_EQUAL_UINT32(0u, g.stall_ms);
    for (int i = 0; i < 800; i++) dm_s2_guard_pass(&g, ++t, 0u, -900.0f, false);
    t += ML_STEP_MAX_MS;
    TEST_ASSERT_EQUAL_INT(ML_STALL, dm_s2_guard_pass(&g, t, 0u, -900.0f, false));
    TEST_ASSERT_EQUAL_UINT32(1000u, g.stall_ms);
}

static void test_guard_pass_gear_moved_while_not_driving_restarts_the_window(void)
{
    // The gear moved 1 mm while the autoload drove nothing (by hand, or the idle control): the next
    // pass that drives starts a new window there, as after 1 mm of travel while driving.
    motion_guard g = s2_guard(1000u, 0u);
    uint64_t t = 1000u;
    for (int i = 0; i < 900; i++) dm_s2_guard_pass(&g, ++t, 0u, -900.0f, false);
    for (uint32_t c = 0u; c <= 200u; c += 4u) dm_s2_guard_pass(&g, ++t, c, 0.0f, false);
    TEST_ASSERT_EQUAL_UINT32(900u, g.stall_ms);
    TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, ++t, 200u, -900.0f, false));
    TEST_ASSERT_EQUAL_UINT32(0u, g.stall_ms);
    TEST_ASSERT_EQUAL_UINT32(200u, g.stall_cnt);
}

static void test_guard_pass_new_stage_restarts_the_budget_not_the_stall_window(void)
{
    // A later stage of the run (the retract after a buffer abort, the push after it): 600 ms at
    // 900 PWM against a gear that does not turn and 5 s that drive nothing, then a new stage on a pass
    // that drives nothing (as on an abort). Its time budget starts again; the stall window goes on and
    // trips at 1 s of drive in all, 400 ms into the new stage, whichever way it drives. (A new guard
    // per stage gave a blocked gear another 1 s with every abort.)
    motion_guard g = s2_guard(1000u, 7u);
    uint64_t t = 1000u;
    for (int i = 0; i < 600; i++) dm_s2_guard_pass(&g, ++t, 7u, -900.0f, false);
    for (int i = 0; i < 5000; i++) dm_s2_guard_pass(&g, ++t, 7u, 0.0f, false);
    TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, ++t, 7u, 0.0f, true));
    TEST_ASSERT_EQUAL_UINT32(1u, g.run_ms);
    TEST_ASSERT_EQUAL_UINT32(600u, g.stall_ms);
    for (int i = 0; i < 399; i++) TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, ++t, 7u, 900.0f, false));
    TEST_ASSERT_EQUAL_INT(ML_STALL, dm_s2_guard_pass(&g, ++t, 7u, 900.0f, false));

    // On a pass that drives (the push an interrupted retract goes on with, from IDLE): the same, and
    // that pass counts.
    g = s2_guard(1000u, 7u);
    t = 1000u;
    for (int i = 0; i < 600; i++) dm_s2_guard_pass(&g, ++t, 7u, 900.0f, false);
    TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, ++t, 7u, -900.0f, true));
    TEST_ASSERT_EQUAL_UINT32(1u, g.run_ms);
    TEST_ASSERT_EQUAL_UINT32(601u, g.stall_ms);
    for (int i = 0; i < 398; i++) TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, ++t, 7u, -900.0f, false));
    TEST_ASSERT_EQUAL_INT(ML_STALL, dm_s2_guard_pass(&g, ++t, 7u, -900.0f, false));

    // The new stage's budget: 12 s, from the pass before it.
    g = s2_guard(1000u, 7u);
    t = 1000u;
    for (int i = 0; i < 11000; i++) dm_s2_guard_pass(&g, ++t, 7u, 0.0f, false);
    TEST_ASSERT_EQUAL_UINT32(11000u, g.run_ms);
    TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, ++t, 7u, 0.0f, true));
    for (int i = 0; i < 11998; i++) TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, ++t, 7u, 0.0f, false));
    TEST_ASSERT_EQUAL_INT(ML_TIME, dm_s2_guard_pass(&g, ++t, 7u, 0.0f, false));
}

static void test_guard_pass_new_stage_measures_the_1mm_from_its_start(void)
{
    // A push crept 156 counts (0.9 mm; the count falls while pushing) in 500 ms, and the retract after
    // the abort drives the gear back: 174 counts (1 mm) from where the retract began restart the
    // window, although that is only 18 counts from where the window began.
    motion_guard g = s2_guard(1000u, 10000u);
    uint64_t t = 1000u;
    for (uint32_t i = 1u; i <= 500u; i++) dm_s2_guard_pass(&g, ++t, 10000u - (156u * i) / 500u, -900.0f, false);
    TEST_ASSERT_EQUAL_UINT32(500u, g.stall_ms);
    TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, ++t, 9844u, 0.0f, true));
    TEST_ASSERT_EQUAL_UINT32(500u, g.stall_ms);
    for (uint32_t c = 1u; c < 174u; c++) TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, ++t, 9844u + c, 900.0f, false));
    TEST_ASSERT_EQUAL_UINT32(673u, g.stall_ms);
    TEST_ASSERT_EQUAL_INT(ML_OK, dm_s2_guard_pass(&g, ++t, 10018u, 900.0f, false));
    TEST_ASSERT_EQUAL_UINT32(0u, g.stall_ms);
    TEST_ASSERT_EQUAL_UINT32(10018u, g.stall_cnt);
}

// ---- The state machine: behaves as before ----

static void test_stage2_pushes_120mm_after_stage1(void)
{
    insert_and_start_stage2();
    const int32_t t = until_done(10000u);
    TEST_ASSERT_TRUE(t > 0);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_loaded[0]);
    TEST_ASSERT_EQUAL_UINT8(0u, dm_fail_latch[0]);
    ASSERT_MM_WITHIN(0.1, S2_LEN_MM, pushed_s2_mm);
    ASSERT_MM_WITHIN(0.1, S2_LEN_MM, pushed_mm);
    TEST_ASSERT_INT32_WITHIN(5, 2000, t); // 120 mm at 60 mm/s
    TEST_ASSERT_EQUAL_INT(0, aborts);
}

static void test_three_buffer_aborts_fail_as_before(void)
{
    insert_and_start_stage2();
    block_mm = 30.0;
    TEST_ASSERT_TRUE(until_done(20000u) > 0);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
    TEST_ASSERT_EQUAL_INT(3, aborts);
    TEST_ASSERT_EQUAL_UINT8(0xFFu, led_r);
    const double moved = pushed_mm + retracted_mm;
    run_for(5000u);
    TEST_ASSERT_TRUE(pushed_mm + retracted_mm == moved); // motor off
}

// ---- Key excursions ----

// Stage-2 with key noise from 50 ms in: it must end loaded, not failed.
static void noisy_run(int ks, uint32_t period_ms, uint32_t len_ms)
{
    setUp();
    insert_and_start_stage2();
    noise(ks, period_ms, len_ms);
    noise_t0 = now + 50u;
    TEST_ASSERT_TRUE(until_done(60000u) > 0);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_loaded[0]);
    TEST_ASSERT_EQUAL_UINT8(0u, dm_fail_latch[0]);
}

// ... with no Stage-1 push: 120 mm, all in S2_PUSH.
static void dip_case(int ks, uint32_t period_ms, uint32_t len_ms)
{
    noisy_run(ks, period_ms, len_ms);
    ASSERT_MM_WITHIN(0.1, S2_LEN_MM, pushed_s2_mm);
    ASSERT_MM_WITHIN(0.1, S2_LEN_MM, pushed_mm);
}

static void test_one_pass_dips_keep_the_120mm_countdown(void)
{
    // One pass at 'external only' (S1_DEBOUNCE, IDLE) or at the other state (IDLE) every 200 ms or
    // every 10 ms: the push still ends after 120 mm. It used to start over at every dip.
    dip_case(KS_EXT, 200u, 1u);
    dip_case(KS_OTHER, 200u, 1u);
    dip_case(KS_EXT, 10u, 1u);
    dip_case(KS_OTHER, 10u, 1u);
}

static void test_dips_up_to_99ms_keep_the_120mm_countdown(void)
{
    dip_case(KS_EXT, 500u, 99u); // S1_DEBOUNCE ends before Stage-1 pushes
    dip_case(KS_OTHER, 300u, 99u);
}

static void test_long_excursions_keep_the_120mm_countdown(void)
{
    // 150 ms at 'external only' every second with the filament at 'both': S1_DEBOUNCE, then Stage-1
    // pushes until the key reads 'both' again. The push goes on where it stopped, and Stage-1's
    // 3 mm per excursion count: 120 mm in all. It used to start over (51 mm between excursions).
    noisy_run(KS_EXT, 1000u, 150u);
    ASSERT_MM_WITHIN(0.1, S2_LEN_MM, pushed_mm);
    TEST_ASSERT_TRUE(pushed_s2_mm < S2_LEN_MM - 5.0);
    // 300 ms at the other state every 700 ms: IDLE, no Stage-1.
    dip_case(KS_OTHER, 700u, 300u);
}

static void test_buffer_aborts_are_counted_across_dips(void)
{
    // The path is blocked 30 mm past the inner switch: every push ends in a buffer abort. One-pass
    // dips during the pushes and retracts used to clear the abort count, so the channel cycled
    // push/retract at 900 PWM for good; the third abort fails it.
    insert_and_start_stage2();
    block_mm = 30.0;
    noise(KS_EXT, 150u, 1u);
    const int32_t t = until_done(60000u);
    TEST_ASSERT_TRUE(t > 0);
    TEST_ASSERT_TRUE(t < 2000);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
    TEST_ASSERT_EQUAL_INT(3, aborts);
    TEST_ASSERT_EQUAL_UINT8(3u, dm_auto_try[0]);
}

static void test_retracts_past_the_inner_switch_do_not_loop(void)
{
    // Blocked 5 mm past the inner switch, and the buffer slider sticks for 10 mm: each retract after
    // an abort takes the tip back behind the inner switch before the buffer relaxes ('external
    // only'), Stage-1 pushes it to 'both' again and the run goes on with a push. That used to start
    // a new run with no aborts, for good; the third abort now fails it.
    insert_and_start_stage2();
    block_mm = 5.0;
    backlash_mm = 10.0;
    const int32_t t = until_done(60000u);
    TEST_ASSERT_TRUE(t > 0);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
    TEST_ASSERT_EQUAL_INT(3, aborts);
    TEST_ASSERT_TRUE(pushed_mm < 60.0);
}

static void test_a_dip_during_a_retract_resumes_the_retract(void)
{
    // Blocked 30 mm in: the first push aborts at 40 mm and retracts. A 50 ms dip during the retract
    // resumes the retract (it used to start a new run with a push); the run then fails after three
    // aborts as without the dip.
    insert_and_start_stage2();
    block_mm = 30.0;
    for (int i = 0; (i < 5000) && (dm_auto_state[0] != DM_AUTO_S2_RETRACT); i++) pass();
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S2_RETRACT, dm_auto_state[0]);
    run_for(20u);
    noise(KS_EXT, 100000u, 50u);
    run_for(50u);
    TEST_ASSERT_TRUE(dm_auto_state[0] != DM_AUTO_S2_RETRACT);
    for (int i = 0; (i < 100) && (dm_auto_state[0] != DM_AUTO_S2_RETRACT) && (dm_auto_state[0] != DM_AUTO_S2_PUSH); i++)
        pass();
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S2_RETRACT, dm_auto_state[0]);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_auto_try[0]);
    TEST_ASSERT_TRUE(until_done(20000u) > 0);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
    TEST_ASSERT_EQUAL_INT(3, aborts);
}

static void test_an_interrupted_retract_counts_the_forward_travel_of_its_excursion(void)
{
    // Blocked 30 mm in: the first push aborts 40 mm in, with 80 mm left, and retracts (which adds
    // back what it undoes). 20 ms into the retract the key reads 'external only' for 150 ms:
    // S1_DEBOUNCE, then Stage-1 pushes for 50 ms inside the interrupted retract. At the next 'both'
    // the run goes on with a push, a new stage, with what Stage-1 pushed (3 mm) off its length.
    insert_and_start_stage2();
    block_mm = 30.0;
    for (int i = 0; (i < 5000) && (dm_auto_state[0] != DM_AUTO_S2_RETRACT); i++) pass();
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S2_RETRACT, dm_auto_state[0]);
    run_for(20u);
    forced_ks = KS_EXT;
    pass();
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S1_DEBOUNCE, dm_auto_state[0]);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_RETRACT, dm_s2_run[0].stage);
    const float remain0 = dm_auto_remain_m[0];
    ASSERT_MM_WITHIN(0.01, S2_LEN_MM - (pushed_s2_mm - retracted_mm), remain0 * 1000.0f);
    const double pushed0 = pushed_mm;
    run_for(149u);
    forced_ks = -1;
    pass();
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S2_PUSH, dm_auto_state[0]);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_NONE, dm_s2_run[0].stage);
    ASSERT_MM_WITHIN(0.1, 3.0, pushed_mm - pushed0);
    ASSERT_MM_WITHIN(0.01, (double)remain0 * 1000.0 - (pushed_mm - pushed0), dm_auto_remain_m[0] * 1000.0f);

    // The retract ends on a pass at the other key state (the run waits in IDLE), the filament is
    // pushed 10 mm further in by hand, and the key reads 'both' 1 ms later: the retract resumes with
    // 10 mm less to go (and ends at once, the buffer being back).
    setUp();
    insert_and_start_stage2();
    block_mm = 30.0;
    for (int i = 0; (i < 5000) && (dm_s2_run[0].stage == DM_S2_STAGE_NONE); i++)
    {
        forced_ks = ((dm_auto_state[0] == DM_AUTO_S2_RETRACT) && (buffer_pct() <= DM_AUTO_BUF_RECOVER_PCT)) ? (int)KS_OTHER : -1;
        pass();
    }
    forced_ks = -1;
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_IDLE, dm_auto_state[0]);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_RETRACT, dm_s2_run[0].stage);
    const float remain1 = dm_auto_remain_m[0];
    tip_mm += 10.0;
    gear_mm -= 10.0;
    forced_pct = 50.0f;
    pass();
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S2_PUSH, dm_auto_state[0]);
    ASSERT_MM_WITHIN(0.01, (double)remain1 * 1000.0 - 10.0, dm_auto_remain_m[0] * 1000.0f);
}

static void test_a_retract_that_ends_at_the_other_key_state_keeps_the_aborts(void)
{
    // Blocked 30 mm in: every push aborts, and each retract ends on a pass with the key at the other
    // state (the buffer is back at 50.2% on it): the run is interrupted there, not over, so the next
    // 'both' goes on with it and the third abort still fails it. (With the run dropped there, each
    // such retract started a new run with no aborts, for good.)
    insert_and_start_stage2();
    block_mm = 30.0;
    int other_ends = 0;
    for (int i = 0; (i < 60000) && !dm_fail_latch[0]; i++)
    {
        const bool end = (dm_auto_state[0] == DM_AUTO_S2_RETRACT) && (buffer_pct() <= DM_AUTO_BUF_RECOVER_PCT);
        forced_ks = end ? (int)KS_OTHER : -1;
        pass();
        if (end)
        {
            TEST_ASSERT_EQUAL_UINT8(DM_AUTO_IDLE, dm_auto_state[0]);
            TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_RETRACT, dm_s2_run[0].stage);
            other_ends++;
        }
    }
    forced_ks = -1;
    TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
    TEST_ASSERT_EQUAL_INT(3, aborts);
    TEST_ASSERT_EQUAL_INT(2, other_ends);
    TEST_ASSERT_EQUAL_UINT8(3u, dm_auto_try[0]);

    // Blocked 5 mm past the inner switch, the key at the other state for one pass every 5 ms: the
    // retracts that end on such a pass keep the aborts too.
    setUp();
    insert_and_start_stage2();
    block_mm = 5.0;
    noise(KS_OTHER, 5u, 1u);
    const int32_t t = until_done(120000u);
    TEST_ASSERT_TRUE(t > 0);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
    TEST_ASSERT_EQUAL_INT(3, aborts);
    TEST_ASSERT_TRUE(pushed_mm < 60.0);
}

static void test_stage1_inside_an_interrupted_retract_is_guarded(void)
{
    // The gear does not turn, the tip is 1 mm past the inner switch and the buffer reads 80% (held
    // up): the run starts from IDLE and its push aborts on its first pass. After 300 ms of retract
    // the key reads 'external only', so Stage-1 pushes inside the interrupted retract. The run's stall
    // window counts the retract and Stage-1's push alike: the motor stops after 999 ms of drive in all
    // (300 ms of retract, 699 ms of push). With no guard while a retract is interrupted, Stage-1
    // pushed for its own 5 s.
    v_mm_s = 0.0;
    forced_pct = 80.0f;
    tip_mm = 1.0;
    pass();
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S2_RETRACT, dm_auto_state[0]);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_auto_try[0]);
    run_for(300u);
    TEST_ASSERT_EQUAL_UINT32(300u, drive_ms);
    forced_ks = KS_EXT;
    const int32_t t = until_done(20000u);
    TEST_ASSERT_TRUE(t > 0);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
    TEST_ASSERT_EQUAL_UINT8(0xFFu, led_r);
    TEST_ASSERT_EQUAL_UINT32(ML_STALL_MS - 1u, drive_ms);
    TEST_ASSERT_EQUAL_UINT32(ML_STALL_MS - 1u - 300u, push_ms);
    TEST_ASSERT_EQUAL_INT32(1 + (int32_t)DM_AUTO_S1_DEBOUNCE_MS + (int32_t)(ML_STALL_MS - 300u), t);
}

static void test_a_blocked_gear_stops_despite_dips(void)
{
    // The gear does not turn at 900 PWM. One-pass dips every 300 ms used to restart the stall window
    // with every new push, so it never tripped; it now trips 1 s after the push started.
    insert_and_start_stage2();
    v_mm_s = 0.0;
    noise(KS_EXT, 300u, 1u);
    const int32_t t = until_done(60000u);
    TEST_ASSERT_TRUE(t > 0);
    TEST_ASSERT_INT32_WITHIN(15, 1000, t);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
}

static void test_a_blocked_gear_stops_despite_long_excursions(void)
{
    // 250 ms at 'external only' every 600 ms: S1_DEBOUNCE for 100 ms, then Stage-1 pushes for 149 ms.
    // The push stage's guard goes on through every excursion, and its stall window counts Stage-1's
    // push too, so it trips on the 1000th pass that pushes: 300 ms of S2_PUSH, 149 ms of Stage-1,
    // 349 ms of S2_PUSH, 149 ms of Stage-1, 52 ms of S2_PUSH. A new guard per excursion used to keep
    // the stalled motor at 900 PWM for good, and even the one guard kept left the time away out.
    insert_and_start_stage2();
    v_mm_s = 0.0;
    noise(KS_EXT, 600u, 250u);
    noise_t0 = now + 300u;
    const int32_t t = until_done(60000u);
    TEST_ASSERT_TRUE(t > 0);
    TEST_ASSERT_INT32_WITHIN(5, 1204, t);
    TEST_ASSERT_EQUAL_UINT32(ML_STALL_MS - 1u, push_ms);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
}

static void test_a_slow_gear_runs_out_of_time_despite_dips(void)
{
    // 5 mm/s: 120 mm would take 24 s, the push stage's budget is 12 s (120 mm at 12 mm/s + 2 s).
    // One-pass dips every 300 ms count as time in the stage.
    insert_and_start_stage2();
    v_mm_s = 5.0;
    noise(KS_EXT, 300u, 1u);
    const int32_t t = until_done(60000u);
    TEST_ASSERT_TRUE(t > 0);
    TEST_ASSERT_INT32_WITHIN(20, 12000, t);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
    ASSERT_MM_WITHIN(1.0, 60.0, pushed_s2_mm);
}

// ---- Stage-1 inside an interrupted run ----
// 'external only' during Stage-2 sends the run to S1_DEBOUNCE and Stage-1, whose 5 s timeout starts
// again on every such round. A tip stuck right at the inner switch's lever (1 mm behind it: 'external
// only') with the gear blocked, and the lever or the key ADC reading 'both' now and then: each 'both'
// starts or resumes the push, and its next pass is back in Stage-1. The push stage's guard used to
// wait while the key was away, and the 'both' pass that resumed it drove nothing, so Stage-1 pushed
// at 900 PWM about 90% of the time for good (one 'both' pass a second: more than 3200 s in an hour).

// A gear that does not turn gets at most this much push per insertion, whatever the key, the buffer
// and the auto-unloads do (dm_stage2.h), and at most this much drive (push or retract; the
// auto-unload's own retract aside): Stage-1 before the run pushes on the passes 1 ms to 4999 ms
// after it starts (it times out at 5000 ms), and a run that starts from it drives on at most 999
// more (the run's stall window trips on its 1000th pass that drives, which then drives no more; it
// goes on across the stages a buffer abort starts). A run that starts from IDLE has had no Stage-1
// push before it, and drives on at most 1000 passes (its first pass starts the guard).
#define BLOCKED_PUSH_MAX_MS ((uint32_t)(DM_AUTO_S1_TIMEOUT_MS - 1u) + (ML_STALL_MS - 1u))
// And a stage fails at most its time budget after it began (every pass counts, an auto-unload's too);
// a run has at most five stages (three pushes, two retracts).
#define STAGE_FAIL_MAX_MS 12000u
#define RUN_STAGES_MAX 5u

// xorshift32: every run of the test sees the same patterns.
static uint32_t rng_state = 1u;
static uint32_t rng_next(void)
{
    uint32_t x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state = x;
    return x;
}
static uint32_t rng_in(uint32_t lo, uint32_t hi) { return lo + rng_next() % (hi - lo + 1u); }
static void rng_seed(uint32_t k) { rng_state = 0x2545F491u ^ (0x9E3779B9u * (k + 1u)); if (!rng_state) rng_state = 1u; }

static uint32_t push_at_none;   // push_ms on the last pass at key 'none' (or at the insertion)
static uint32_t drive_at_none;  // drive_ms then
static uint32_t worst_push_ms;  // the most push_ms - push_at_none any pass saw
static uint32_t worst_drive_ms; // the most drive_ms - drive_at_none any pass saw
static uint32_t au_total;       // auto-unloads started in a sweep

// Insertion (from no filament) to a tip at the lever, the gear blocked.
static void blocked_at_lever(void)
{
    setUp();
    tip_mm = -1.0;
    v_mm_s = 0.0;
    push_at_none = drive_at_none = 0u;
}

// One pass, checking the bounds of a blocked gear: the push and the drive since the last 'none'
// pass stay within BLOCKED_PUSH_MAX_MS, a stage has failed STAGE_FAIL_MAX_MS after it began, a run
// has at most RUN_STAGES_MAX stages, one run at most begins per insertion, and a failed channel does
// not drive until the next 'none'.
static void blocked_pass(void)
{
    const bool failed = (fail_t0 != 0u);
    const uint32_t d = drive_ms;
    pass();
    if (MC_ONLINE_key_stu[0] == KS_NONE)
    {
        push_at_none = push_ms;
        drive_at_none = drive_ms;
    }
    TEST_ASSERT_TRUE(push_ms - push_at_none <= BLOCKED_PUSH_MAX_MS);
    TEST_ASSERT_TRUE(drive_ms - drive_at_none <= BLOCKED_PUSH_MAX_MS);
    if (push_ms - push_at_none > worst_push_ms) worst_push_ms = push_ms - push_at_none;
    if (drive_ms - drive_at_none > worst_drive_ms) worst_drive_ms = drive_ms - drive_at_none;
    if ((run_t0 != 0u) && (fail_t0 == 0u))
    {
        TEST_ASSERT_TRUE((now - 1u) - stage_t0 < STAGE_FAIL_MAX_MS);
        TEST_ASSERT_TRUE(run_stages <= RUN_STAGES_MAX);
    }
    TEST_ASSERT_TRUE(runs <= 1u);
    if (failed && (fail_t0 != 0u)) TEST_ASSERT_EQUAL_UINT32(d, drive_ms);
}

// 'both' for len_lo..len_hi ms, starting every every_lo..every_hi ms, for ms; the tip sets the key
// otherwise.
static void both_blips(uint32_t ms, uint32_t len_lo, uint32_t len_hi, uint32_t every_lo, uint32_t every_hi)
{
    const uint64_t end = now + ms;
    while (now < end)
    {
        const uint32_t every = rng_in(every_lo, every_hi);
        const uint32_t len = rng_in(len_lo, len_hi);
        forced_ks = -1;
        for (uint32_t i = len; (i < every) && (now < end); i++) blocked_pass();
        forced_ks = KS_BOTH;
        for (uint32_t i = 0u; (i < len) && (now < end); i++) blocked_pass();
    }
    forced_ks = -1;
}

// After a blocked-gear scenario: failed, red, motor off, the run record dropped.
static void check_blocked_failed(void)
{
    TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
    // Red, unless an auto-unload still retracts: its purple wins while it does.
    TEST_ASSERT_EQUAL_UINT8(g_auto_unload[0].active ? 0xA0u : 0xFFu, led_r);
    TEST_ASSERT_EQUAL_UINT8(g_auto_unload[0].active ? 0x2Du : 0x00u, led_g);
    TEST_ASSERT_TRUE(fail_t0 != 0u);
    TEST_ASSERT_TRUE(push_ms <= BLOCKED_PUSH_MAX_MS);
    TEST_ASSERT_TRUE(drive_ms <= BLOCKED_PUSH_MAX_MS);
}

static void test_a_blocked_gear_at_the_lever_stops_despite_both_blips(void)
{
    // 'both' for one pass every second or every 4 s, and for 5 ms every second, at several phases.
    const uint32_t period[] = {1000u, 4000u, 1000u};
    const uint32_t len[] = {1u, 1u, 5u};
    const uint32_t phase[] = {0u, 1u, 99u, 100u, 101u, 137u, 500u, 999u, 3999u};
    for (unsigned i = 0; i < sizeof(period) / sizeof(period[0]); i++)
        for (unsigned k = 0; k < sizeof(phase) / sizeof(phase[0]); k++)
        {
            if (phase[k] >= period[i]) continue;
            blocked_at_lever();
            noise(KS_BOTH, period[i], len[i]);
            noise_t0 = now + phase[k];
            const uint64_t t0 = now;
            for (int n = 0; n < 60000; n++) blocked_pass();
            check_blocked_failed();
            TEST_ASSERT_TRUE(fail_t0 - t0 <= DM_AUTO_S1_DEBOUNCE_MS + DM_AUTO_S1_TIMEOUT_MS + STAGE_FAIL_MAX_MS);
        }

    // Random blips: 1-10 ms every 0.2-3 s, and 1-30 ms every 0.25-1 s.
    for (uint32_t k = 0u; k < 20u; k++)
    {
        blocked_at_lever();
        rng_seed(k);
        both_blips(60000u, 1u, 10u, 200u, 3000u);
        check_blocked_failed();

        blocked_at_lever();
        rng_seed(100u + k);
        both_blips(60000u, 1u, 30u, 250u, 1000u);
        check_blocked_failed();
    }
}

static void test_the_worst_key_pattern_for_a_blocked_gear(void)
{
    // The most push a key can get out of a blocked gear: Stage-1 up to its timeout, 'both' on the
    // pass it would time out, then 'external only' again, so that Stage-1 pushes for the run until
    // the stall window trips. 4999 ms + 999 ms.
    blocked_at_lever();
    forced_ks = KS_EXT;
    run_for(1u + (uint32_t)DM_AUTO_S1_DEBOUNCE_MS + (uint32_t)DM_AUTO_S1_TIMEOUT_MS - 1u);
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S1_PUSH, dm_auto_state[0]);
    TEST_ASSERT_EQUAL_UINT32(DM_AUTO_S1_TIMEOUT_MS - 1u, push_ms);
    forced_ks = KS_BOTH;
    pass();
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S2_PUSH, dm_auto_state[0]);
    forced_ks = KS_EXT;
    run_for(1u + (uint32_t)DM_AUTO_S1_DEBOUNCE_MS + ML_STALL_MS - 1u);
    TEST_ASSERT_EQUAL_UINT8(0u, dm_fail_latch[0]);
    pass();
    check_blocked_failed();
    TEST_ASSERT_EQUAL_UINT32(BLOCKED_PUSH_MAX_MS, push_ms);
    TEST_ASSERT_EQUAL_UINT32(5998u, BLOCKED_PUSH_MAX_MS);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_NONE, dm_s2_run[0].stage);
    run_for(60000u);
    TEST_ASSERT_EQUAL_UINT32(BLOCKED_PUSH_MAX_MS, push_ms);
}

static void test_buffer_aborts_do_not_restart_the_stall_window(void)
{
    // The buffer reading crosses 75% and back while the gear does not turn (a person moving the
    // buffer, or ADC noise near 75%), tip at the lever: 'external only' for 5099 ms (Stage-1 up to its
    // timeout), then three times 'both' for 990 ms at 50%, 'both' for 2 ms at 78% (an abort, and one
    // pass of retract) and 'external only' for 1095 ms at 50% (Stage-1 inside the interrupted
    // retract). Each abort's retract, the push after it and the push an interrupted retract goes on
    // with used to start a new stall window: 9953 ms of push, three aborts, failed after 10.3 s. The
    // run's window goes on across them: failed in the first round, 1 ms under the bound (the first
    // 'both' comes one pass before Stage-1's timeout: 4998 ms of Stage-1, then 989 ms of push, 1 ms
    // of retract and 9 ms of Stage-1 inside the run). (78%, not the review's 80%: 80% and back to 50%
    // within 1 s is also the auto-unload's gesture, covered below.)
    blocked_at_lever();
    forced_pct = 50.0f;
    forced_ks = KS_EXT;
    for (int n = 0; n < 5099; n++) blocked_pass();
    for (int k = 0; k < 3; k++)
    {
        forced_ks = KS_BOTH;
        forced_pct = 50.0f;
        for (int n = 0; n < 990; n++) blocked_pass();
        forced_pct = 78.0f;
        for (int n = 0; n < 2; n++) blocked_pass();
        forced_ks = KS_EXT;
        forced_pct = 50.0f;
        for (int n = 0; n < 1095; n++) blocked_pass();
    }
    check_blocked_failed();
    TEST_ASSERT_EQUAL_INT(1, aborts);
    TEST_ASSERT_TRUE(fail_t0 - 10000u < 5099u + 990u + 2u + 1095u);
    TEST_ASSERT_EQUAL_UINT32(BLOCKED_PUSH_MAX_MS - 2u, push_ms);
    TEST_ASSERT_EQUAL_UINT32(BLOCKED_PUSH_MAX_MS - 1u, drive_ms);
}

static void test_an_adaptive_key_and_buffer_pattern_bounds_a_blocked_gear(void)
{
    // An adversary that watches the run's stall window, tip at the lever: Stage-1 up to its timeout,
    // then 'both'. In a stage, as soon as the window is 2 ms short of ML_STALL_MS it starts the next
    // one (78% in S2_PUSH: an abort; 50% in S2_RETRACT: the retract ends), and otherwise keeps the
    // stage driving (50% in S2_PUSH, 78% in S2_RETRACT). With a window per stage that got 4999 ms of
    // Stage-1 and then about 1 s of drive in each of the five stages. Now it gets 998 ms of push after
    // Stage-1, 1 ms under the bound, and then uses its three aborts up without driving (the window
    // has no time left): the third fails the run.
    blocked_at_lever();
    forced_ks = KS_EXT;
    forced_pct = 50.0f;
    while (!((dm_auto_state[0] == DM_AUTO_S1_PUSH) && (now - dm_auto_t0_ms[0] >= DM_AUTO_S1_TIMEOUT_MS))) blocked_pass();
    forced_ks = KS_BOTH;
    for (int n = 0; n < 60000; n++)
    {
        const uint8_t st = dm_auto_state[0];
        const bool near = (dm_s2_guard[0].stall_ms + 2u >= ML_STALL_MS);
        if (st == DM_AUTO_S2_PUSH) forced_pct = near ? 78.0f : 50.0f;
        else if (st == DM_AUTO_S2_RETRACT) forced_pct = near ? 50.0f : 78.0f;
        blocked_pass();
    }
    check_blocked_failed();
    TEST_ASSERT_EQUAL_UINT32(BLOCKED_PUSH_MAX_MS - 1u, push_ms);
    TEST_ASSERT_EQUAL_UINT32(BLOCKED_PUSH_MAX_MS - 1u, drive_ms);
    TEST_ASSERT_EQUAL_UINT8(3u, dm_auto_try[0]);
}

static void test_periodic_key_patterns_bound_the_push_into_a_blocked_gear(void)
{
    // After 150 ms at 'external only' (Stage-1 starts), 'external only' or the other state for `away`
    // ms, then 'both' for `both` ms, over and over, around the 100 ms debounces, the 200 ms step cap
    // and the 5 s timeout. 4950 ms: the first 'both' comes on the pass Stage-1 would time out, and
    // with 'external only' after it that is the worst case.
    const uint32_t both_ms[] = {1u, 2u, 5u, 30u, 99u, 150u, 250u, 1000u};
    const uint32_t away_ms[] = {1u, 2u, 50u, 99u, 100u, 101u, 150u, 199u, 200u, 201u, 202u, 250u,
                                999u, 1000u, 3000u, 4950u, 4999u, 5100u};
    const uint8_t away_ks[] = {KS_EXT, KS_OTHER};
    worst_push_ms = 0u;
    for (unsigned a = 0; a < sizeof(away_ks); a++)
        for (unsigned i = 0; i < sizeof(both_ms) / sizeof(both_ms[0]); i++)
            for (unsigned j = 0; j < sizeof(away_ms) / sizeof(away_ms[0]); j++)
            {
                blocked_at_lever();
                forced_ks = KS_EXT;
                for (int n = 0; n < 150; n++) blocked_pass();
                const uint64_t end = now + 40000u;
                while (now < end)
                {
                    forced_ks = away_ks[a];
                    for (uint32_t n = 0u; (n < away_ms[j]) && (now < end); n++) blocked_pass();
                    forced_ks = KS_BOTH;
                    for (uint32_t n = 0u; (n < both_ms[i]) && (now < end); n++) blocked_pass();
                }
                TEST_ASSERT_TRUE(fail_t0 != 0u);
            }
    TEST_ASSERT_EQUAL_UINT32(BLOCKED_PUSH_MAX_MS, worst_push_ms);
}

// Runs of 'both', 'external only', the other state and (with none_pct > 0) 'none', each 1 ms to 3 s
// long (spread over four decades; up to 300 ms with short), for ms, with the gear blocked. With
// buffer, each run also sets the buffer reading to 45%, 50.2% (a retract ends), 60%, 75% (no abort
// yet) or 78% (an abort; below the auto-unload's 80% lift, which random_gesture_sweep adds). With
// lead, the key first reads 'external only' for 0 to 5.2 s, so that the run starts anywhere in
// Stage-1's 5 s. Then 'both' (at 50%) until the channel has failed, which must take less than 13 s.
static void random_key_sweep(uint32_t seed0, uint32_t trials, uint32_t none_pct, bool buffer, uint32_t ms,
                             bool short_runs = false, bool lead = false)
{
    static const float pct[] = {45.0f, 50.2f, 60.0f, 75.0f, 78.0f};
    for (uint32_t k = 0u; k < trials; k++)
    {
        blocked_at_lever();
        rng_seed(seed0 + k);
        if (lead)
        {
            forced_ks = KS_EXT;
            forced_pct = 50.0f;
            const uint32_t len = rng_in(0u, 5200u);
            for (uint32_t n = 0u; n < len; n++) blocked_pass();
        }
        const uint64_t end = now + ms;
        while (now < end)
        {
            const uint32_t r = rng_in(0u, 99u);
            forced_ks = (r < none_pct) ? (int)KS_NONE :
                        (r < none_pct + (100u - none_pct) / 3u) ? (int)KS_BOTH :
                        (r < none_pct + 2u * (100u - none_pct) / 3u) ? (int)KS_EXT : (int)KS_OTHER;
            if (buffer) forced_pct = pct[rng_in(0u, 4u)];
            const uint32_t decade = rng_in(0u, short_runs ? 2u : 3u);
            const uint32_t len = rng_in(1u, (decade == 0u) ? 3u : (decade == 1u) ? 30u : (decade == 2u) ? 300u : 3000u);
            for (uint32_t n = 0u; (n < len) && (now < end); n++) blocked_pass();
        }
        forced_ks = KS_BOTH;
        forced_pct = buffer ? 50.0f : -1.0f;
        for (uint32_t n = 0u; (n < 13000u) && !dm_fail_latch[0]; n++) blocked_pass();
        TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
    }
}

static void test_any_key_pattern_bounds_the_push_into_a_blocked_gear(void)
{
    worst_push_ms = worst_drive_ms = 0u;
    random_key_sweep(1000u, 300u, 0u, false, 60000u);
    TEST_ASSERT_TRUE(worst_push_ms > 5000u);
}

static void test_any_key_pattern_with_removals_bounds_each_insertion(void)
{
    // Key 'none' ends the insertion (the fail latch too): the bound holds from each 'none' on.
    worst_push_ms = worst_drive_ms = 0u;
    random_key_sweep(5000u, 300u, 5u, false, 60000u);
    TEST_ASSERT_TRUE(worst_push_ms > 5000u);
}

static void test_any_key_and_buffer_pattern_bounds_the_drive_into_a_blocked_gear(void)
{
    // The buffer reading crosses the 75% abort and the 50.2% recovery levels as well, so the run
    // aborts, retracts and pushes again: its stall window goes on across all of it. 600 patterns of
    // runs up to 300 ms after a Stage-1 of any length (with a stall window per stage, 32 of them got
    // more than the bound, up to 7840 ms), and 300 with runs up to 3 s and removals.
    worst_push_ms = worst_drive_ms = 0u;
    random_key_sweep(9000u, 600u, 0u, true, 30000u, true, true);
    random_key_sweep(13000u, 300u, 5u, true, 30000u);
    TEST_ASSERT_TRUE(worst_push_ms > 5800u);
    TEST_ASSERT_TRUE(worst_drive_ms > 5900u);
}

static void test_a_tip_stuck_at_the_lever_still_loads_when_the_gear_turns(void)
{
    // The same 'both' blips with a gear that turns: Stage-1 pushes the tip to 'both', and the run
    // feeds 120 mm from where it began (at a blip behind the lever, or at the inner switch).
    const uint32_t period[] = {1000u, 4000u, 1000u};
    const uint32_t len[] = {1u, 1u, 5u};
    const uint32_t phase[] = {0u, 1u, 50u, 99u, 100u, 101u, 102u, 110u, 117u, 500u};
    for (unsigned i = 0; i < sizeof(period) / sizeof(period[0]); i++)
        for (unsigned k = 0; k < sizeof(phase) / sizeof(phase[0]); k++)
        {
            setUp();
            tip_mm = -1.0;
            noise(KS_BOTH, period[i], len[i]);
            noise_t0 = now + phase[k];
            TEST_ASSERT_TRUE(until_done(20000u) > 0);
            TEST_ASSERT_EQUAL_UINT8(1u, dm_loaded[0]);
            TEST_ASSERT_EQUAL_UINT8(0u, dm_fail_latch[0]);
            ASSERT_MM_WITHIN(0.1, S2_LEN_MM, pushed_mm - run_pushed0);
            TEST_ASSERT_TRUE(pushed_mm <= S2_LEN_MM + 1.1);
            TEST_ASSERT_TRUE(retracted_mm == 0.0);
        }

    // Random blips of 1-30 ms every 0.25-1 s, and a slower gear (20 mm/s).
    for (uint32_t k = 0u; k < 20u; k++)
    {
        setUp();
        tip_mm = -1.0;
        v_mm_s = (k & 1u) ? 20.0 : 60.0;
        rng_seed(200u + k);
        for (int n = 0; (n < 20000) && !dm_loaded[0] && !dm_fail_latch[0]; n++)
        {
            const uint32_t every = rng_in(250u, 1000u), blip = rng_in(1u, 30u);
            forced_ks = -1;
            for (uint32_t m = blip; (m < every) && !dm_loaded[0] && !dm_fail_latch[0]; m++) pass();
            forced_ks = KS_BOTH;
            for (uint32_t m = 0u; (m < blip) && !dm_loaded[0] && !dm_fail_latch[0]; m++) pass();
        }
        TEST_ASSERT_EQUAL_UINT8(1u, dm_loaded[0]);
        TEST_ASSERT_EQUAL_UINT8(0u, dm_fail_latch[0]);
        ASSERT_MM_WITHIN(0.1, S2_LEN_MM, pushed_mm - run_pushed0);
    }
}

// ---- Auto-unloads during the autoload ----
// The buffer lifted to 80% or more and let back into 45-55% within 1 s starts the auto-unload, also
// while the autoload runs (it needs only the idle control). motor_motion_run then retracts at 850 PWM
// instead of running run(), so the DM block does not run until the auto-unload ends (the buffer below
// 35%, 1.5 s after the key left 'both', or 15 s).

// A gesture from the pass it starts on: the buffer at lift_pct for up_ms, then let back to 50%.
static void gesture(uint32_t up_ms, float lift_pct = 85.0f)
{
    forced_pct = lift_pct;
    for (uint32_t n = 0u; n < up_ms; n++) blocked_pass();
    forced_pct = 50.0f;
}

static void test_an_auto_unload_counts_for_the_run_as_time_not_as_drive(void)
{
    // The gear does not turn, the tip rests at the lever, the buffer at 50%: Stage-1 for 1 s, 'both'
    // for one pass (the run starts), then 'external only', and Stage-1 pushes inside the run. After
    // 500 ms of that push a gesture: 20 ms at 85%, and the auto-unload retracts for 1.5 s. Its passes
    // go to the run's guard as passes that drive nothing: the stage's budget counts them, and the
    // stall window neither grows (no fail during the auto-unload) nor restarts: once it has ended,
    // Stage-1 pushes on until the run's 1000th pass that drives, which trips the window.
    blocked_at_lever();
    forced_pct = 50.0f;
    forced_ks = KS_EXT;
    for (uint32_t n = 0u; n < 1u + (uint32_t)DM_AUTO_S1_DEBOUNCE_MS + 1000u; n++) blocked_pass();
    forced_ks = KS_BOTH;
    blocked_pass();
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S2_PUSH, dm_auto_state[0]);
    const uint32_t d0 = drive_ms;
    forced_ks = KS_EXT;
    while (drive_ms - d0 < 500u) blocked_pass();
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S1_PUSH, dm_auto_state[0]);
    gesture(20u);
    blocked_pass();
    TEST_ASSERT_EQUAL_UINT8(1u, g_auto_unload[0].active);
    TEST_ASSERT_EQUAL_UINT32(1u, au_ms);
    const uint32_t stall0 = dm_s2_guard[0].stall_ms;
    const uint32_t run0 = dm_s2_guard[0].run_ms;
    TEST_ASSERT_EQUAL_UINT32(520u, stall0);
    while (au_ms < (uint32_t)AUTO_UNLOAD_EMPTY_MS) blocked_pass();
    TEST_ASSERT_EQUAL_UINT8(1u, g_auto_unload[0].active);
    TEST_ASSERT_EQUAL_UINT8(0u, dm_fail_latch[0]);
    TEST_ASSERT_EQUAL_UINT32(stall0, dm_s2_guard[0].stall_ms);
    TEST_ASSERT_EQUAL_UINT32(run0 + (uint32_t)AUTO_UNLOAD_EMPTY_MS - 1u, dm_s2_guard[0].run_ms);
    TEST_ASSERT_EQUAL_UINT32(520u, drive_ms - d0);
    // It ends on the next pass, on which run() runs again and Stage-1 pushes.
    blocked_pass();
    TEST_ASSERT_EQUAL_UINT8(0u, g_auto_unload[0].active);
    TEST_ASSERT_EQUAL_UINT32(stall0 + 1u, dm_s2_guard[0].stall_ms);
    TEST_ASSERT_EQUAL_UINT32(521u, drive_ms - d0);
    for (int n = 0; (n < 5000) && !dm_fail_latch[0]; n++) blocked_pass();
    check_blocked_failed();
    TEST_ASSERT_EQUAL_UINT32(ML_STALL_MS - 1u, drive_ms - d0);
    run_for(20000u);
    TEST_ASSERT_EQUAL_UINT32(ML_STALL_MS - 1u, drive_ms - d0);
}

static void test_auto_unload_gestures_and_key_blips_bound_a_blocked_gear(void)
{
    // The review's pattern. The gear does not turn and the tip rests at the lever, the buffer at 50%:
    // Stage-1 up to the pass before its timeout, then 'both' for one pass (the run starts), then
    // 'external only', and Stage-1 pushes inside the run. Each time the run's stall window has
    // counted 800 ms since it (re)started, a gesture (20 ms at 85%) starts an auto-unload, which
    // retracts until 1.5 s after the key left 'both'. Every 3 s, when no auto-unload runs, 'both' for
    // one pass, which restarts Stage-1's 5 s. With the auto-unload's passes a gap for the run's guard,
    // each gesture restarted the stall window and kept its time out of the budget: this got 16 094 ms
    // of push, 13 gestures, and failed 31.5 s after the run began. Now they count as time: one
    // gesture, and the window trips on the run's 1000th pass that drives, 5998 ms of push in all.
    blocked_at_lever();
    forced_pct = 50.0f;
    forced_ks = KS_EXT;
    for (uint32_t n = 0u; n < 1u + (uint32_t)DM_AUTO_S1_DEBOUNCE_MS + (uint32_t)DM_AUTO_S1_TIMEOUT_MS - 1u; n++)
        blocked_pass();
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S1_PUSH, dm_auto_state[0]);
    forced_ks = KS_BOTH;
    blocked_pass();
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S2_PUSH, dm_auto_state[0]);
    forced_ks = KS_EXT;
    uint64_t blip = now;
    uint32_t gestures = 0u;
    bool armed = true;
    uint32_t window = 0u;
    const uint64_t end = now + 60000u;
    while (now < end)
    {
        if (dm_s2_guard[0].stall_ms < window) armed = true; // the window restarted
        window = dm_s2_guard[0].stall_ms;
        if (armed && !g_auto_unload[0].active && !dm_fail_latch[0] && (window >= 800u))
        {
            armed = false;
            gestures++;
            gesture(20u);
        }
        else if (!g_auto_unload[0].active && (now - blip >= 3000u))
        {
            forced_ks = KS_BOTH;
            blocked_pass();
            forced_ks = KS_EXT;
            blip = now;
        }
        else
        {
            blocked_pass();
        }
    }
    check_blocked_failed();
    TEST_ASSERT_EQUAL_UINT32(BLOCKED_PUSH_MAX_MS, push_ms);
    TEST_ASSERT_EQUAL_UINT32(BLOCKED_PUSH_MAX_MS, drive_ms);
    TEST_ASSERT_EQUAL_UINT32(1u, gestures);
    TEST_ASSERT_EQUAL_UINT32(1u, au_starts);
    TEST_ASSERT_EQUAL_UINT32((uint32_t)AUTO_UNLOAD_EMPTY_MS, au_ms);
}

static void test_a_long_auto_unload_runs_the_stage_budget_out(void)
{
    // The gear does not turn and the tip is 1 mm past the inner switch ('both'), the buffer at 50%:
    // the run starts from IDLE and pushes for 300 ms. A gesture: 10 ms at 85%, where the push aborts
    // and the retract after it drives, then 50%, and the auto-unload retracts. The key stays at 'both',
    // so it goes on for its 15 s. The retract stage's budget counts the auto-unload's passes and runs
    // out 12 s after the pass before the abort, during the auto-unload: failed, and the autoload drives
    // nothing more.
    // The auto-unload retracts for its 15 s (the person's), then the LED shows red. With the
    // auto-unload's passes left out, the stage lived on through them and pushed another second after
    // them.
    v_mm_s = 0.0;
    tip_mm = 1.0;
    forced_pct = 50.0f;
    for (int n = 0; n < 300; n++) blocked_pass();
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S2_PUSH, dm_auto_state[0]);
    TEST_ASSERT_EQUAL_UINT32(300u, push_ms);
    gesture(10u);
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S2_RETRACT, dm_auto_state[0]);
    TEST_ASSERT_EQUAL_INT(1, aborts);
    const uint64_t abort_t = stage_t0;
    TEST_ASSERT_EQUAL_UINT32(309u, drive_ms);
    blocked_pass();
    TEST_ASSERT_EQUAL_UINT8(1u, g_auto_unload[0].active);
    const uint64_t au_t0 = now - 1u;
    while (!dm_fail_latch[0] && g_auto_unload[0].active) blocked_pass();
    TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
    TEST_ASSERT_EQUAL_UINT8(1u, g_auto_unload[0].active);
    TEST_ASSERT_EQUAL_UINT64(abort_t + STAGE_FAIL_MAX_MS - 1u, fail_t0);
    check_blocked_failed();
    while (g_auto_unload[0].active) blocked_pass();
    TEST_ASSERT_EQUAL_UINT64(au_t0 + AUTO_UNLOAD_MAX_MS, au_last + 1u);
    TEST_ASSERT_EQUAL_UINT32((uint32_t)AUTO_UNLOAD_MAX_MS, au_ms);
    check_blocked_failed();
    for (int n = 0; n < 20000; n++) blocked_pass();
    TEST_ASSERT_EQUAL_UINT32(309u, drive_ms);
    TEST_ASSERT_EQUAL_UINT32(300u, push_ms);
}

static void test_an_auto_unload_while_the_printer_has_the_channel_is_not_counted(void)
{
    // As test_a_long_auto_unload_runs_the_stage_budget_out, but the printer takes the channel out of
    // the idle control once the auto-unload runs, and leaves it in idle again when it has ended. The
    // DM block would not have run on those passes either, so the run's guard is left alone, as for
    // any time the printer has the channel: no fail while the printer has it (the LED would show the
    // autoload's red during the printer's move). Back in idle, the time away was a gap: the run goes
    // on, with a new stall window, and fails 1 s of drive later. (Not a blocked_pass() scenario: the
    // printer takes the channel.)
    v_mm_s = 0.0;
    tip_mm = 1.0;
    forced_pct = 50.0f;
    run_for(300u);
    forced_pct = 85.0f;
    run_for(10u);
    forced_pct = 50.0f;
    pass();
    TEST_ASSERT_EQUAL_UINT8(1u, g_auto_unload[0].active);
    printer_idle = false;
    while (g_auto_unload[0].active) pass();
    TEST_ASSERT_EQUAL_UINT32((uint32_t)AUTO_UNLOAD_MAX_MS, au_ms);
    TEST_ASSERT_EQUAL_UINT8(0u, dm_fail_latch[0]);
    printer_idle = true;
    const uint32_t d0 = drive_ms;
    for (int n = 0; (n < 5000) && !dm_fail_latch[0]; n++) pass();
    TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
    TEST_ASSERT_EQUAL_UINT32(ML_STALL_MS - 1u, drive_ms - d0);
}

static void test_an_auto_unload_during_the_autoload_unloads_and_the_next_insertion_loads(void)
{
    // The gear turns. 500 ms into the Stage-2 push (30 mm) a gesture: 50 ms at 85%, where the push
    // aborts and the retract after it drives, then 50%: the auto-unload retracts the filament past
    // both switches (1.5 s after the key left 'both'), which ends the run, with no fail. Inserted
    // again, the autoload runs as on a new insertion: Stage-1, then 120 mm of Stage-2.
    insert_and_start_stage2();
    run_for(500u);
    forced_pct = 85.0f;
    run_for(50u);
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S2_RETRACT, dm_auto_state[0]);
    forced_pct = 50.0f;
    pass();
    TEST_ASSERT_EQUAL_UINT8(1u, g_auto_unload[0].active);
    for (int n = 0; (n < 20000) && g_auto_unload[0].active; n++)
    {
        pass();
        TEST_ASSERT_EQUAL_UINT8(0u, dm_fail_latch[0]);
    }
    TEST_ASSERT_EQUAL_UINT8(0u, g_auto_unload[0].active);
    TEST_ASSERT_EQUAL_UINT8(KS_NONE, MC_ONLINE_key_stu[0]);
    TEST_ASSERT_EQUAL_UINT8(0u, dm_loaded[0]);
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_IDLE, dm_auto_state[0]);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_NONE, dm_s2_run[0].stage);
    TEST_ASSERT_EQUAL_UINT8(0u, dm_autoload_gate[0]);
    run_for(500u);
    forced_pct = -1.0f;
    insert_and_start_stage2();
    TEST_ASSERT_TRUE(until_done(10000u) > 0);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_loaded[0]);
    TEST_ASSERT_EQUAL_UINT8(0u, dm_fail_latch[0]);
    ASSERT_MM_WITHIN(0.1, S2_LEN_MM, pushed_s2_mm);
}

// Random key runs as in random_key_sweep (removals with none_pct), the buffer at 30% (an auto-unload
// ends), 45%, 50.2%, 60%, 75% or 78%, and gestures: the buffer at 80%, 85% or 95% for 1 ms to 1.2 s,
// then, three times in four, back to 46%, 50% or 54%, which starts an auto-unload unless one runs or
// the release falls on the two passes where a lift held for over 1 s re-arms. After a lead of
// 'external only' for 0 to 5.2 s at 50%, for ms, with the gear blocked.
// Then 'both' at 50% until the channel has failed, which must take less than 13 s from the pattern's
// end or from the end of an auto-unload still running then.
static void random_gesture_sweep(uint32_t seed0, uint32_t trials, uint32_t none_pct, uint32_t ms)
{
    static const float pct[] = {30.0f, 45.0f, 50.2f, 60.0f, 75.0f, 78.0f};
    static const float lift[] = {80.0f, 85.0f, 95.0f};
    static const float back[] = {46.0f, 50.0f, 54.0f};
    for (uint32_t k = 0u; k < trials; k++)
    {
        blocked_at_lever();
        rng_seed(seed0 + k);
        forced_ks = KS_EXT;
        forced_pct = 50.0f;
        const uint32_t lead = rng_in(0u, 5200u);
        for (uint32_t n = 0u; n < lead; n++) blocked_pass();
        const uint64_t end = now + ms;
        while (now < end)
        {
            if (rng_in(0u, 99u) < 30u)
            {
                forced_pct = lift[rng_in(0u, 2u)];
                const uint32_t up = rng_in(1u, (rng_in(0u, 3u) == 0u) ? 1200u : 300u);
                for (uint32_t n = 0u; (n < up) && (now < end); n++) blocked_pass();
                if (rng_in(0u, 3u) != 0u) forced_pct = back[rng_in(0u, 2u)];
            }
            else
            {
                const uint32_t r = rng_in(0u, 99u);
                forced_ks = (r < none_pct) ? (int)KS_NONE :
                            (r < none_pct + (100u - none_pct) / 3u) ? (int)KS_BOTH :
                            (r < none_pct + 2u * (100u - none_pct) / 3u) ? (int)KS_EXT : (int)KS_OTHER;
                if (rng_in(0u, 1u) == 0u) forced_pct = pct[rng_in(0u, 5u)];
            }
            const uint32_t decade = rng_in(0u, 3u);
            const uint32_t len = rng_in(1u, (decade == 0u) ? 3u : (decade == 1u) ? 30u : (decade == 2u) ? 300u : 3000u);
            for (uint32_t n = 0u; (n < len) && (now < end); n++) blocked_pass();
        }
        forced_ks = KS_BOTH;
        forced_pct = 50.0f;
        const uint64_t t_end = now;
        for (uint32_t n = 0u; (n < 13000u + (uint32_t)AUTO_UNLOAD_MAX_MS) && !dm_fail_latch[0]; n++) blocked_pass();
        TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
        TEST_ASSERT_EQUAL_UINT8(g_auto_unload[0].active ? 0xA0u : 0xFFu, led_r);
        const uint64_t from = ((au_last + 1u) > t_end) ? (au_last + 1u) : t_end;
        TEST_ASSERT_TRUE((fail_t0 < from) || (fail_t0 - from < 13000u));
        au_total += au_starts;
    }
}

static void test_any_key_buffer_and_gesture_pattern_bounds_a_blocked_gear(void)
{
    // 300 patterns of 60 s, and 300 with removals: auto-unloads started in between do not give a
    // blocked gear more than the bound per insertion, or a stage more than 12 s, and a failed channel's
    // autoload drives no more (the auto-unload still retracts: the person's).
    worst_push_ms = worst_drive_ms = 0u;
    au_total = 0u;
    random_gesture_sweep(20000u, 300u, 0u, 60000u);
    random_gesture_sweep(30000u, 300u, 5u, 60000u);
    TEST_ASSERT_TRUE(worst_push_ms > 5800u);
    TEST_ASSERT_TRUE(au_total > 3000u);
}

static void test_gestures_and_key_dips_do_not_take_a_run_past_120mm(void)
{
    // A gear that turns (60 or 20 mm/s), the key following the tip but for dips of 1 to 99 ms to
    // 'external only' or the other state (too short for Stage-1 to push), and gestures now and then
    // (an auto-unload that pulls the filament out, or back behind the inner switch when the buffer
    // drops below 35% during it): whenever the filament is out, it is inserted again at the outer
    // switch. No run moves the filament more than 120 mm net forward from where it began (plus the
    // last pass of push), and runs still end loaded.
    uint32_t loaded = 0u;
    uint32_t starts = 0u;
    for (uint32_t k = 0u; k < 100u; k++)
    {
        setUp();
        rng_seed(50000u + k);
        v_mm_s = (k & 1u) ? 20.0 : 60.0;
        tip_mm = -10.0;
        uint32_t dip = 0u;
        for (uint32_t n = 0u; n < 60000u; n++)
        {
            const uint32_t r = rng_in(0u, 9999u);
            if ((dip == 0u) && (r < 20u))
            {
                dip = rng_in(1u, 99u);
                forced_ks = (r < 10u) ? (int)KS_EXT : (int)KS_OTHER;
            }
            else if (r < 25u) forced_pct = 85.0f;
            else if ((r < 60u) && (forced_pct > 80.0f)) forced_pct = 50.0f;
            else if ((r < 62u) && g_auto_unload[0].active) forced_pct = 30.0f;
            else if (r < 70u) forced_pct = -1.0f;
            const bool loaded0 = dm_loaded[0] != 0u;
            pass();
            if (!loaded0 && dm_loaded[0]) loaded++;
            if (dip > 0u && --dip == 0u) forced_ks = -1;
            TEST_ASSERT_TRUE(run_fwd_max <= S2_LEN_MM + v_mm_s * 0.001 + 0.01);
            if ((key_from_tip() == KS_NONE) && !g_auto_unload[0].active) tip_mm = -10.0;
        }
        starts += au_starts;
    }
    TEST_ASSERT_TRUE(loaded > 100u);
    TEST_ASSERT_TRUE(starts > 1000u);
}

static void test_stage1_can_push_past_the_run_only_on_a_misread_key(void)
{
    // What the 120 mm do not bound: Stage-1 pushes while the key reads 'external only', until it reads
    // 'both' again. 1 mm before the push is done, the key misreads the tip (past the inner switch) as
    // 'external only' for 300 ms: after the 100 ms debounce Stage-1 pushes 12 mm more, and the next
    // 'both' ends the run, loaded, 131 mm from where it began.
    insert_and_start_stage2();
    while (dm_auto_remain_m[0] > 0.001f) pass();
    forced_ks = KS_EXT;
    run_for(300u);
    forced_ks = -1;
    TEST_ASSERT_TRUE(until_done(100u) > 0);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_loaded[0]);
    ASSERT_MM_WITHIN(0.2, S2_LEN_MM - 1.0 + 12.0, pushed_mm);
    ASSERT_MM_WITHIN(0.2, S2_LEN_MM - 1.0 + 12.0, run_fwd_max);
}

// ---- Stage-1 from IDLE for an interrupted run ----

static void test_an_interrupted_push_goes_on_through_stage1_from_idle(void)
{
    // 500 ms into the push (30 mm) the key reads the other state for one pass (the run waits in
    // IDLE), and the filament is then pulled back by hand until its tip is 5 mm behind the inner
    // switch ('external only'). Stage-1 runs from IDLE only once per insertion, and it has run, so
    // the interrupted run used to wait there until its budget ran out and then failed (red until the
    // filament was pulled out). Stage-1 now runs for the interrupted run: it pushes the tip back to
    // 'both', and the run goes on with the 90 mm it had left (the hand pull does not lengthen it).
    insert_and_start_stage2();
    run_for(500u);
    forced_ks = KS_OTHER;
    pass();
    forced_ks = -1;
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_IDLE, dm_auto_state[0]);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_PUSH, dm_s2_run[0].stage);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_autoload_gate[0]);
    const double pull = tip_mm + 5.0;
    tip_mm -= pull;
    gear_mm += pull;
    const int32_t t = until_done(20000u);
    TEST_ASSERT_TRUE(t > 0);
    TEST_ASSERT_TRUE(t < 2000);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_loaded[0]);
    TEST_ASSERT_EQUAL_UINT8(0u, dm_fail_latch[0]);
    ASSERT_MM_WITHIN(0.1, S2_LEN_MM, pushed_s2_mm);
    ASSERT_MM_WITHIN(0.1, 5.0, pushed_mm - pushed_s2_mm);

    // The same with the gear blocked from the pull on: that Stage-1 push is inside the run's stall
    // window, which stops it after 1 s of drive; failed, red, motor off.
    setUp();
    insert_and_start_stage2();
    run_for(500u);
    forced_ks = KS_OTHER;
    pass();
    forced_ks = -1;
    tip_mm = -5.0;
    v_mm_s = 0.0;
    const uint32_t d0 = drive_ms;
    TEST_ASSERT_TRUE(until_done(20000u) > 0);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
    TEST_ASSERT_TRUE(drive_ms - d0 <= ML_STALL_MS);
    TEST_ASSERT_TRUE(drive_ms - d0 >= ML_STALL_MS - 100u);

    // An interrupted retract the same way: blocked 30 mm in, the first retract ends on a pass at the
    // other state, and the tip is then pulled back behind the inner switch. Stage-1 pushes it back,
    // the run goes on with a push and fails at its third abort (it used to wait in IDLE with one
    // abort until its budget ran out).
    setUp();
    insert_and_start_stage2();
    block_mm = 30.0;
    for (int i = 0; (i < 5000) && (dm_s2_run[0].stage == DM_S2_STAGE_NONE); i++)
    {
        forced_ks = ((dm_auto_state[0] == DM_AUTO_S2_RETRACT) && (buffer_pct() <= DM_AUTO_BUF_RECOVER_PCT)) ? (int)KS_OTHER : -1;
        pass();
    }
    forced_ks = -1;
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_RETRACT, dm_s2_run[0].stage);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_auto_try[0]);
    const double pull2 = tip_mm + 5.0;
    tip_mm -= pull2;
    gear_mm += pull2;
    const uint32_t p0 = push_ms;
    TEST_ASSERT_TRUE(until_done(20000u) > 0);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_fail_latch[0]);
    TEST_ASSERT_EQUAL_UINT8(3u, dm_auto_try[0]);
    TEST_ASSERT_TRUE(push_ms - p0 > 80u); // Stage-1's 5 mm, then the pushes to the aborts
}

static void test_stage1_from_idle_still_runs_once_per_insertion_without_a_run(void)
{
    // No run interrupted: Stage-1 from IDLE is still allowed once per insertion. The key leaves
    // 'external only' for the other state during Stage-1's debounce, and the next 'external only'
    // pushes nothing, for good; so does a boot with the filament at 'external only'.
    tip_mm = -5.0;
    run_for(50u);
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_S1_DEBOUNCE, dm_auto_state[0]);
    forced_ks = KS_OTHER;
    pass();
    forced_ks = -1;
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_IDLE, dm_auto_state[0]);
    run_for(20000u);
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_IDLE, dm_auto_state[0]);
    TEST_ASSERT_EQUAL_UINT32(0u, drive_ms);
    TEST_ASSERT_EQUAL_UINT8(0u, dm_fail_latch[0]);

    setUp();
    tip_mm = -5.0;
    MC_ONLINE_key_stu[0] = key_from_tip();
    dm_init();
    run_for(20000u);
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_IDLE, dm_auto_state[0]);
    TEST_ASSERT_EQUAL_UINT32(0u, drive_ms);
}

// ---- A new run ----

static void test_removal_and_reinsertion_start_a_new_run(void)
{
    // 60 mm into the push the key leaves 'both' (the other state), and during that excursion the
    // filament is pulled out past both switches; inserted again, the new run pushes the full 120 mm.
    insert_and_start_stage2();
    run_for(1000u);
    noise(KS_OTHER, 100000u, 20u);
    run_for(20u);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_PUSH, dm_s2_run[0].stage);
    tip_mm = -100.0;
    run_for(200u);
    TEST_ASSERT_EQUAL_UINT8(DM_AUTO_IDLE, dm_auto_state[0]);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_NONE, dm_s2_run[0].stage);
    insert_and_start_stage2();
    TEST_ASSERT_TRUE(until_done(10000u) > 0);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_loaded[0]);
    ASSERT_MM_WITHIN(0.1, S2_LEN_MM, pushed_s2_mm);
}

static void test_a_finished_run_is_not_resumed(void)
{
    // After the push is done, the next run (armed again by a real event, here the filament out and
    // in) is a new one.
    insert_and_start_stage2();
    TEST_ASSERT_TRUE(until_done(10000u) > 0);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_NONE, dm_s2_run[0].stage);
    tip_mm = -100.0;
    run_for(200u);
    insert_and_start_stage2();
    TEST_ASSERT_TRUE(until_done(10000u) > 0);
    ASSERT_MM_WITHIN(0.1, S2_LEN_MM, pushed_s2_mm);
}

static void test_a_printer_load_ends_an_interrupted_run(void)
{
    // A push interrupted by the other key state; the printer then loads the channel and holds it
    // with the key at 'both' (before_on_use): the channel is loaded, the run is over.
    insert_and_start_stage2();
    run_for(500u);
    noise(KS_OTHER, 100000u, 1u);
    run_for(1u);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_PUSH, dm_s2_run[0].stage);
    printer_idle = false;
    ams[0].now_filament_num = 0u;
    ams[0].filament[0].motion = _filament_motion::before_on_use;
    run_for(10u);
    TEST_ASSERT_EQUAL_UINT8(1u, dm_loaded[0]);
    TEST_ASSERT_EQUAL_UINT8(DM_S2_STAGE_NONE, dm_s2_run[0].stage);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_enter_without_an_interrupted_run_starts_a_new_one);
    RUN_TEST(test_an_interrupted_push_goes_on_with_its_length_aborts_and_guard);
    RUN_TEST(test_an_interrupted_retract_resumes_only_after_a_dip);
    RUN_TEST(test_forward_travel_during_the_excursion_counts);
    RUN_TEST(test_forward_travel_during_a_retract_excursion_counts);
    RUN_TEST(test_each_excursion_is_used_once_and_an_end_forgets_it);
    RUN_TEST(test_guard_pass_that_drives_is_the_plain_check);
    RUN_TEST(test_guard_pass_that_does_not_drive_counts_only_time);
    RUN_TEST(test_guard_pass_that_does_not_drive_runs_the_budget_out);
    RUN_TEST(test_guard_pass_leaves_gaps_out_as_motion_guard_check_does);
    RUN_TEST(test_guard_pass_gear_moved_while_not_driving_restarts_the_window);
    RUN_TEST(test_guard_pass_new_stage_restarts_the_budget_not_the_stall_window);
    RUN_TEST(test_guard_pass_new_stage_measures_the_1mm_from_its_start);
    RUN_TEST(test_stage2_pushes_120mm_after_stage1);
    RUN_TEST(test_three_buffer_aborts_fail_as_before);
    RUN_TEST(test_one_pass_dips_keep_the_120mm_countdown);
    RUN_TEST(test_dips_up_to_99ms_keep_the_120mm_countdown);
    RUN_TEST(test_long_excursions_keep_the_120mm_countdown);
    RUN_TEST(test_buffer_aborts_are_counted_across_dips);
    RUN_TEST(test_retracts_past_the_inner_switch_do_not_loop);
    RUN_TEST(test_a_dip_during_a_retract_resumes_the_retract);
    RUN_TEST(test_an_interrupted_retract_counts_the_forward_travel_of_its_excursion);
    RUN_TEST(test_a_retract_that_ends_at_the_other_key_state_keeps_the_aborts);
    RUN_TEST(test_stage1_inside_an_interrupted_retract_is_guarded);
    RUN_TEST(test_a_blocked_gear_stops_despite_dips);
    RUN_TEST(test_a_blocked_gear_stops_despite_long_excursions);
    RUN_TEST(test_a_slow_gear_runs_out_of_time_despite_dips);
    RUN_TEST(test_a_blocked_gear_at_the_lever_stops_despite_both_blips);
    RUN_TEST(test_the_worst_key_pattern_for_a_blocked_gear);
    RUN_TEST(test_buffer_aborts_do_not_restart_the_stall_window);
    RUN_TEST(test_an_adaptive_key_and_buffer_pattern_bounds_a_blocked_gear);
    RUN_TEST(test_periodic_key_patterns_bound_the_push_into_a_blocked_gear);
    RUN_TEST(test_any_key_pattern_bounds_the_push_into_a_blocked_gear);
    RUN_TEST(test_any_key_pattern_with_removals_bounds_each_insertion);
    RUN_TEST(test_any_key_and_buffer_pattern_bounds_the_drive_into_a_blocked_gear);
    RUN_TEST(test_a_tip_stuck_at_the_lever_still_loads_when_the_gear_turns);
    RUN_TEST(test_an_auto_unload_counts_for_the_run_as_time_not_as_drive);
    RUN_TEST(test_auto_unload_gestures_and_key_blips_bound_a_blocked_gear);
    RUN_TEST(test_a_long_auto_unload_runs_the_stage_budget_out);
    RUN_TEST(test_an_auto_unload_while_the_printer_has_the_channel_is_not_counted);
    RUN_TEST(test_an_auto_unload_during_the_autoload_unloads_and_the_next_insertion_loads);
    RUN_TEST(test_any_key_buffer_and_gesture_pattern_bounds_a_blocked_gear);
    RUN_TEST(test_gestures_and_key_dips_do_not_take_a_run_past_120mm);
    RUN_TEST(test_stage1_can_push_past_the_run_only_on_a_misread_key);
    RUN_TEST(test_an_interrupted_push_goes_on_through_stage1_from_idle);
    RUN_TEST(test_stage1_from_idle_still_runs_once_per_insertion_without_a_run);
    RUN_TEST(test_removal_and_reinsertion_start_a_new_run);
    RUN_TEST(test_a_finished_run_is_not_resumed);
    RUN_TEST(test_a_printer_load_ends_an_interrupted_run);
    return UNITY_END();
}
