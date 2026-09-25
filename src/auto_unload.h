#pragma once
// The BMCU's own retracts in motor_motion_run (Motion_control.cpp) when a person lifts the buffer:
// the auto-unload and the manual empty pull. Hardware-free, so the decisions are tested on the host
// (test/test_auto_unload). motor_motion_run calls auto_unload_pass() once per main-loop pass for
// every channel whose AS5600 reads are good, and drives the channel as it returns instead of
// running the channel's motor control.
//
// - Auto-unload (upstream V10.5, "automatic filament unload when the buffer is lifted manually"):
//   while the channel runs its idle control, the buffer lifted to AUTO_UNLOAD_START_PCT and let back
//   into the neutral band (45-55%) within AUTO_UNLOAD_ARM_MS starts a retract at 850 PWM. It ends
//   when the buffer falls below AUTO_UNLOAD_ABORT_PCT, when the key has been away from 'both' (no
//   filament at a non-DM channel's switch) for AUTO_UNLOAD_EMPTY_MS, or after AUTO_UNLOAD_MAX_MS;
//   another one then needs a new lift.
// - Manual empty pull (upstream V10.4, "retraction when the buffer is pulled up manually, even when
//   there is no filament inside"): no filament at the switches and the buffer above
//   AUTO_UNLOAD_EMPTY_PULL_PCT: a retract at 700 PWM for as long as the buffer reads above it.
//
// Offline (motor_motion_run's error != 0: from boot until the first heartbeat, and after a lost
// link) motor_motion_run stops every channel, but an auto-unload that was already running kept its
// 850 PWM for up to 15 s, and the manual empty pull had no link check at all. Now an auto-unload
// ends when the link goes (its state is cleared, as when the channel leaves the idle control) and
// the manual pull does not run offline, so the stop motor_motion_run sets brakes the motor. Nothing
// in upstream's intent needs either without the printer: the auto-unload cannot even start offline
// (it needs the idle control, which only a live link sets), on the A1 the BMCU is powered by the
// printer that sends the heartbeats, offline means the boot before the first heartbeat or a link
// fault, in which the BMCU otherwise keeps its motors stopped, and the manual pull ends only when
// the buffer reading drops, so a buffer stuck or misread above 80% drove 700 PWM with no end. Both
// work again from the first pass with the link up (an auto-unload after a new lift).
//
// Jam latch (jam_latch.h): outside on_use/stop_on_use a person releases it by holding the buffer at
// or above JAM_RELEASE_AWAY_PCT (85%) for JAM_RELEASE_MS (1 s), the auto-unload's own lift. A
// latched active channel is stopped in idle (motor_motion_switch), so it had no idle control and no
// auto-unload; but the pass that released it put it back into the idle control with the buffer
// still held up, the lift armed the auto-unload (and re-armed it every second while held), and
// letting go started an 850 PWM unload on the way back through the neutral band. A latched channel
// that is not the active one runs the idle control (braked, jam_latch_brakes()), so there any lift
// armed it, also one too short or too low to release the latch. Now Motion_control_run calls
// auto_unload_hold() on every pass while the channel's jam latch is set and on the pass that
// releases it: no lift arms the auto-unload until a pass, in any state, with the buffer below
// AUTO_UNLOAD_START_PCT. So lifting a latched channel's buffer only ever releases the latch, letting
// go starts nothing, and a new lift after that unloads as before. That is also upstream's handling
// of a jam: the pull back of a latched channel is 100 mm instead of the full retract, and a latched
// active channel is stopped in idle, where the gesture never reached it. Upstream did let a quick
// lift unload a latched channel that is not the active one; this fork blocks that on purpose, since
// a release attempt that fell short would otherwise unload the channel. A latched channel is
// unloaded by the printer, by hand, or by the gesture once released. A running auto-unload keeps
// its own ends (it only retracts), and the manual empty pull needs no filament at the switch, which
// clears the latch, so it is unchanged.
#include <stdbool.h>
#include <stdint.h>

#define AUTO_UNLOAD_START_PCT      80.0f
#define AUTO_UNLOAD_NEUTRAL_LO_PCT 45.0f
#define AUTO_UNLOAD_NEUTRAL_HI_PCT 55.0f
#define AUTO_UNLOAD_ABORT_PCT      35.0f
#define AUTO_UNLOAD_ARM_MS         1000u
#define AUTO_UNLOAD_MAX_MS         15000u
#define AUTO_UNLOAD_EMPTY_MS       1500u
#define AUTO_UNLOAD_EMPTY_PULL_PCT 80.0f

typedef struct
{
    uint8_t  arm;           // lifted to AUTO_UNLOAD_START_PCT: waiting for the neutral band
    uint8_t  active;        // the auto-unload retracts
    uint8_t  blocked;       // no new start until the buffer is lifted again
    uint8_t  wait_low;      // auto_unload_hold(): no arming until a pass below AUTO_UNLOAD_START_PCT
    uint64_t arm_t0_ms;
    uint64_t active_t0_ms;
    uint64_t empty_t0_ms;   // first pass with the key away from 'both' while active (0 = none)
} auto_unload_t;

// wait_low is left as it is: only the buffer ends the hold-off (auto_unload_pass()).
static inline void auto_unload_reset(auto_unload_t *s)
{
    s->arm          = 0u;
    s->active       = 0u;
    s->blocked      = 0u;
    s->arm_t0_ms    = 0u;
    s->active_t0_ms = 0u;
    s->empty_t0_ms  = 0u;
}

// The channel's jam latch is set, or was released on this pass (Motion_control_run): a lift that
// is under way no longer counts, and none arms the auto-unload until the buffer has been below
// AUTO_UNLOAD_START_PCT. A running auto-unload goes on.
static inline void auto_unload_hold(auto_unload_t *s)
{
    s->wait_low  = 1u;
    s->arm       = 0u;
    s->arm_t0_ms = 0u;
}

typedef enum
{
    AU_DRIVE_NONE = 0,    // the channel's motor control runs
    AU_DRIVE_UNLOAD,      // auto-unload: retract at AUTO_UNLOAD_PWM_PULL (850)
    AU_DRIVE_EMPTY_PULL,  // manual empty pull: retract at 700 PWM
} au_drive_t;

typedef struct
{
    bool     online;     // motor_motion_run's error == 0
    bool     inserted;   // filament_channel_inserted[ch]
    bool     idle_ctrl;  // the channel runs its idle control (filament_motion_pressure_ctrl_idle)
    float    pct;        // MC_PULL_pct_f[ch]
    uint8_t  ks;         // MC_ONLINE_key_stu[ch]
    uint64_t now_ms;
} au_in_t;

// One main-loop pass for one channel: updates its state and returns what motor_motion_run drives.
static inline au_drive_t auto_unload_pass(auto_unload_t *s, const au_in_t *in)
{
    if (in->pct < AUTO_UNLOAD_START_PCT) s->wait_low = 0u;

    if (!in->online || !in->inserted || (!s->active && !in->idle_ctrl))
    {
        auto_unload_reset(s);
    }
    else
    {
        const float pct = in->pct;
        const uint8_t ks = in->ks;
        const uint64_t time_now = in->now_ms;

        if (pct >= AUTO_UNLOAD_START_PCT)
        {
            s->blocked = 0u;

            if (!s->arm && !s->active && !s->wait_low)
            {
                s->arm = 1u;
                s->arm_t0_ms = time_now;
            }
        }

        if (s->arm && !s->active)
        {
            const uint64_t dt = time_now - s->arm_t0_ms;

            if ((pct > AUTO_UNLOAD_NEUTRAL_LO_PCT) && (pct < AUTO_UNLOAD_NEUTRAL_HI_PCT))
            {
                if (!s->blocked && dt <= AUTO_UNLOAD_ARM_MS)
                {
                    s->active       = 1u;
                    s->active_t0_ms = time_now;
                    s->empty_t0_ms  = 0u;
                    s->blocked      = 1u;
                }

                s->arm       = 0u;
                s->arm_t0_ms = 0u;
            }
            else if (dt > AUTO_UNLOAD_ARM_MS)
            {
                s->arm       = 0u;
                s->arm_t0_ms = 0u;
            }
        }

        if (s->active)
        {
            if (pct < AUTO_UNLOAD_ABORT_PCT)
            {
                s->active       = 0u;
                s->active_t0_ms = 0u;
                s->empty_t0_ms  = 0u;
                s->blocked      = 1u;
            }
            else if (ks == 1u)
            {
                s->empty_t0_ms = 0u;

                if ((time_now - s->active_t0_ms) >= AUTO_UNLOAD_MAX_MS)
                {
                    s->active       = 0u;
                    s->active_t0_ms = 0u;
                    s->empty_t0_ms  = 0u;
                    s->blocked      = 1u;
                }
            }
            else
            {
                if (s->empty_t0_ms == 0u)
                {
                    s->empty_t0_ms = time_now;
                }
                else if ((time_now - s->empty_t0_ms) >= AUTO_UNLOAD_EMPTY_MS)
                {
                    s->active       = 0u;
                    s->active_t0_ms = 0u;
                    s->empty_t0_ms  = 0u;
                    s->blocked      = 1u;
                }
            }
        }
    }

    if (!in->online) return AU_DRIVE_NONE;
    if (s->active) return AU_DRIVE_UNLOAD;
    if (in->inserted && (in->ks == 0u) && (in->pct > AUTO_UNLOAD_EMPTY_PULL_PCT)) return AU_DRIVE_EMPTY_PULL;
    return AU_DRIVE_NONE;
}
