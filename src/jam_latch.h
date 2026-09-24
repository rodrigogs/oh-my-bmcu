#pragma once
// On-use jam (tangle) latch of Motion_control.cpp: when a low buffer while printing is reported as
// a jam, and when that latch is released again. Hardware-free, so the decisions are tested on the
// host (test/test_jam_latch). Motion_control_run calls jam_latch_pass() once per main-loop pass for
// every channel, with the printer's command for it mapped by jam_host_from_motion().
//
// While the printer prints from a channel (on_use) the BMCU holds its buffer at
// MC_ON_USE_TARGET_PCT (52% on A1) and, below 50%, pushes with its full on_use force (900 of 1000
// PWM). A buffer that still falls below JAM_TRIP_PCT means the extruder is drawing filament the
// BMCU cannot supply: the spool is tangled or stuck. The channel then latches: motor braked, state
// LED red, and pressure 0xF06F reported, on which the printer pauses the print.
//
// It used to latch on the first reading below 40% (a ~4.8 ms ADC average) and was only cleared by
// pulling the filament out past the switches, by lifting the buffer above 85% while the printer
// was in send_out, or by a reboot: a resumed print paused again at once.
#include <stdbool.h>
#include <stdint.h>

#include "ams.h"

// ---- Trip ----
// Level unchanged: 12% of buffer travel below the on_use target.
#define JAM_TRIP_PCT 40.0f

// The buffer must stay below JAM_TRIP_PCT on every pass of the on_use control for this long. Each
// counted pass is one in which the BMCU pushes with full force, a pass of the 500 ms anti-stall rest
// that only follows 0.8 s of that push with the AS5600 showing the motor not turning, or a pass in
// which the 20 s high-PWM latch has braked the channel (then nothing feeds the buffer at all). A
// spool that comes free refills far faster than any extruder drains (the same motor feeds 60 mm/s
// in send_out; 30 mm^3/s through the hotend is 12.5 mm/s of 1.75 mm filament), so a dip that ends
// within 0.5 s is a snag the push cleared (the anti-stall kick comes at 0.15 s). 0.5 s is below the
// anti-stall's own 0.8 s give-up. It delays the report of a real tangle by 0.5 s after the buffer
// reached 40%, i.e. after the extruder had already drawn 12% of the buffer travel with no feed: in
// that 0.5 s it draws about 0.6 mm more at 3 mm^3/s (0.2 mm nozzle) and about 6 mm at 30 mm^3/s.
#define JAM_TRIP_MS 500u

typedef struct
{
    uint8_t  low;           // the last pass saw the buffer below JAM_TRIP_PCT
    uint32_t low_since_ms;  // first pass of that run of low passes
} jam_trip_t;

static inline void jam_trip_reset(jam_trip_t *t)
{
    t->low = 0u;
    t->low_since_ms = 0u;
}

// Call on every pass of the on_use control while the jam latch is clear and filament is present,
// and jam_trip_reset() whenever that is not the case. now_ms may wrap (unsigned differences).
// Returns true when the channel must latch. The first low pass only starts the timer.
static inline bool jam_trip_update(jam_trip_t *t, float pct, uint32_t now_ms)
{
    if (!(pct < JAM_TRIP_PCT))
    {
        jam_trip_reset(t);
        return false;
    }
    if (!t->low)
    {
        t->low = 1u;
        t->low_since_ms = now_ms;
    }
    return (uint32_t)(now_ms - t->low_since_ms) >= JAM_TRIP_MS;
}

// ---- Release ----
// The upstream intent (V9: "released once the buffer returns to the neutral position", V10: "then
// you can resume normally"). The latch is released when either:
// - the printer resumes the channel: since the trip it has commanded anything other than on_use
//   for it (stop_on_use, send_out, pull-back, idle, another channel) and then commands
//   before_on_use or on_use for it again, and the buffer has been at or above JAM_TRIP_PCT at
//   least once since the trip. If it has not, no filament has moved since the trip and the tangle
//   is still there: the channel stays latched, i.e. it is latched again at once, without another
//   JAM_TRIP_MS of push (the time below JAM_TRIP_PCT has already qualified; a reading at that level
//   is also what restarts the trip timer). So a printer that retries by itself (stop_on_use, on_use,
//   ...) gets 0xF06F on every retry instead of a brief release that hides the tangle. Fixing it
//   (press the lever and feed filament, or lift the buffer) raises the buffer above that level,
//   and the resume then releases it; or
// - the buffer is at or above a release level on every pass for JAM_RELEASE_MS. The BMCU does not
//   feed a latched channel while it is the printer's active one (braked in on_use, before_on_use
//   and stop_on_use, stopped in idle and send_out: jam_latch_brakes() below). One that is not
//   (another channel or none is active) runs the idle control, which pushes only below 30% (and
//   the DM autoload in it stops its 120 mm push above 75%). So only a person or the printer moving
//   filament can get it to a release level:
//   - while the printer is in on_use or stop_on_use (printing or paused, filament held in the
//     extruder): the on_use band's low edge (MC_ON_USE_TARGET_PCT - MC_ON_USE_BAND_LO_DELTA, where
//     the on_use control stops pushing). The spring's rest position (50%, the calibrated neutral)
//     is below it, so a slack filament alone does not release it;
//   - in any other state (send_out, pull-back, idle, another channel active):
//     JAM_RELEASE_AWAY_PCT.
// send_out alone does not release it: a latched send_out still waits (upstream's >85% lift in
// send_out also still releases at once), so a reload the printer starts by itself does not drive
// the motor into a tangle nobody has cleared. After a release, a tangle that is still there drains
// the buffer below JAM_TRIP_PCT again and latches again JAM_TRIP_MS later: the print still pauses.

// 1 s: about 200 ADC windows (4.8 ms), so a noisy reading or a bounce of the buffer cannot release
// it, while a person holding the buffer up does it easily.
#define JAM_RELEASE_MS 1000u

// Outside on_use/stop_on_use the printer may move filament (a pull-back pushes it back into the
// buffer), and nothing brings the buffer back to its rest: a latched channel is stopped in idle and
// send_out while it is the printer's active one, and the idle control, which it runs otherwise,
// leaves the buffer wherever it is between 30% and 70%
// (MC_PULL_DEADBAND_PCT_LOW/HIGH). So after a printer-driven unload slider friction can hold it at
// or above the on_use band (51.8% to 60% on A1), and a release there would let the printer's
// reload drive the motor into the tangle. 85% is above that 30-70% range, and it is the level at
// which upstream already releases the latch at once in send_out, where a person lifts the buffer
// to resume a load.
#define JAM_RELEASE_AWAY_PCT 85.0f

// What the printer commands for the latched channel.
typedef enum
{
    JAM_HOST_ON_USE = 0,     // on_use
    JAM_HOST_STOP_ON_USE,    // stop_on_use
    JAM_HOST_BEFORE_ON_USE,  // before_on_use
    JAM_HOST_OTHER,          // anything else, or another channel (or none) is active
} jam_host_t;

// active: the printer's active channel (A.now_filament_num) is this one; motion: what it commands
// for it (A.filament[ch].motion).
static inline jam_host_t jam_host_from_motion(bool active, _filament_motion motion)
{
    if (!active) return JAM_HOST_OTHER;
    switch (motion)
    {
    case _filament_motion::on_use:        return JAM_HOST_ON_USE;
    case _filament_motion::stop_on_use:   return JAM_HOST_STOP_ON_USE;
    case _filament_motion::before_on_use: return JAM_HOST_BEFORE_ON_USE;
    default:                              return JAM_HOST_OTHER;
    }
}

typedef struct
{
    uint8_t  host_left;      // since the trip the printer has commanded something other than on_use
    uint8_t  risen;          // since the trip the buffer has been at or above JAM_TRIP_PCT
    uint8_t  full;           // the last pass saw the buffer at or above the release level
    uint32_t full_since_ms;  // first pass of that run of full passes
} jam_release_t;

static inline void jam_release_reset(jam_release_t *r)
{
    r->host_left = 0u;
    r->risen = 0u;
    r->full = 0u;
    r->full_since_ms = 0u;
}

// Call on every main-loop pass while the jam latch is set, and jam_release_reset() while it is
// not. band_lo_pct: the on_use band's low edge. Returns true when the latch must be released.
static inline bool jam_release_update(jam_release_t *r, jam_host_t host, float pct, float band_lo_pct,
                                      uint32_t now_ms)
{
    if (!(pct < JAM_TRIP_PCT)) r->risen = 1u;
    if (host != JAM_HOST_ON_USE) r->host_left = 1u;
    if (r->host_left && r->risen && (host == JAM_HOST_ON_USE || host == JAM_HOST_BEFORE_ON_USE))
        return true;

    const float level =
        (host == JAM_HOST_ON_USE || host == JAM_HOST_STOP_ON_USE) ? band_lo_pct : JAM_RELEASE_AWAY_PCT;
    if (!(pct >= level))
    {
        r->full = 0u;
        return false;
    }
    if (!r->full)
    {
        r->full = 1u;
        r->full_since_ms = now_ms;
    }
    return (uint32_t)(now_ms - r->full_since_ms) >= JAM_RELEASE_MS;
}

// ---- One pass ----
typedef struct
{
    jam_trip_t    trip;
    jam_release_t release;
} jam_latch_t;

typedef struct
{
    bool             on_use_ctrl;  // the BMCU runs its on_use control for the channel
    bool             filament;     // filament at the channel's switch (MC_ONLINE_key_stu != 0)
    bool             active;       // A.now_filament_num is this channel
    _filament_motion motion;       // A.filament[ch].motion
    float            pct;          // buffer level (MC_PULL_pct_f)
    uint32_t         now_ms;
} jam_in_t;

typedef enum
{
    JAM_EVENT_NONE = 0,
    JAM_EVENT_TRIP,     // brake and jam were set: motor braked, red LED, 0xF06F
    JAM_EVENT_RELEASE,  // brake and jam were cleared
} jam_event_t;

// One main-loop pass for one channel. brake/jam: the channel's g_on_use_low_latch and
// g_on_use_jam_latch, set on a trip and cleared on a release. The trip only looks at jam, so a
// channel the 20 s high-PWM latch has braked silently (brake set, jam clear) whose buffer then
// stays below JAM_TRIP_PCT for JAM_TRIP_MS is still reported. Motion_control_run clears both
// itself when the filament leaves the switch (MC_ONLINE_key_stu == 0), before this call.
static inline jam_event_t jam_latch_pass(jam_latch_t *s, uint8_t *brake, uint8_t *jam,
                                         const jam_in_t *in, float band_lo_pct)
{
    if (*jam)
    {
        jam_trip_reset(&s->trip);
        if (!jam_release_update(&s->release, jam_host_from_motion(in->active, in->motion), in->pct,
                                band_lo_pct, in->now_ms))
            return JAM_EVENT_NONE;

        jam_release_reset(&s->release);
        *brake = 0u;
        *jam = 0u;
        return JAM_EVENT_RELEASE;
    }

    jam_release_reset(&s->release);

    if (!(in->on_use_ctrl && in->filament))
    {
        jam_trip_reset(&s->trip);
        return JAM_EVENT_NONE;
    }
    if (!jam_trip_update(&s->trip, in->pct, in->now_ms)) return JAM_EVENT_NONE;

    jam_trip_reset(&s->trip);
    *brake = 1u;
    *jam = 1u;
    return JAM_EVENT_TRIP;
}

// ---- Motor ----
// What run() in Motion_control.cpp runs for the channel (MOTOR_CONTROL[ch].motion).
typedef enum
{
    JAM_CTRL_ON_USE = 0,     // the on_use control (filament_motion_pressure_ctrl_on_use)
    JAM_CTRL_BEFORE_ON_USE,  // hold_load in before_on_use: below MC_LOAD_S2_HOLD_TARGET_PCT (90% on
                             // A1) it pushes, at up to 1000 PWM from MC_LOAD_S2_PUSH_START_PCT down
    JAM_CTRL_OTHER,          // anything else
} jam_ctrl_t;

// True when run() must brake the channel instead of running its control. brake/jam: the channel's
// g_on_use_low_latch and g_on_use_jam_latch.
// - The on_use control: while either latch is set (unchanged).
// - hold_load in before_on_use: while the jam latch is set. It used to run on, so a printer that
//   resumed a paused print with before_on_use while the spool was still held had the motor push
//   into the tangle at up to 1000 PWM (duty-cycled by the anti-stall). Its silent 20 s latch cannot
//   be set there: set_motion() clears it when the channel enters before_on_use.
// The active channel's other states need nothing here: stop_on_use brakes anyway,
// motor_motion_switch stops a latched active channel in idle and send_out, and before_pull_back and
// the pull back only retract. Not covered: a latched channel that is not the printer's active one
// (another channel or none is active, filament at the switch) runs the idle control, which is not
// braked here: below 30% its PID pushes the buffer back towards 50%, at up to 800 PWM (and the DM
// autoload in it pushes at 900 PWM for a channel not marked loaded).
// The release rules above are unchanged. A resume with before_on_use releases the latch at once if
// the buffer has been back at JAM_TRIP_PCT since the trip (a person fed filament or lifted the
// buffer), and hold_load runs from that pass on. If it has not, the channel stays latched and
// braked, its state LED stays red, and the printer's on_use after it gets 0xF06F: the printer's
// before_on_use replies carry the fixed pressure set_motion() gives them (bambu_bus_ams.cpp), so
// the jam reaches the printer in on_use, as before.
static inline bool jam_latch_brakes(jam_ctrl_t ctrl, uint8_t brake, uint8_t jam)
{
    if (ctrl == JAM_CTRL_ON_USE) return brake != 0u;
    if (ctrl == JAM_CTRL_BEFORE_ON_USE) return jam != 0u;
    return false;
}
