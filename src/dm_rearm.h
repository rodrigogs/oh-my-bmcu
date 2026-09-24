#pragma once
// DM autoload (BMCU_DM_TWO_MICROSWITCH) of Motion_control.cpp: when a channel counts as loaded
// (dm_loaded), and so when the autoload's Stage-2 push may run again. Hardware-free, so the
// decision is tested on the host (test/test_dm_rearm). motor_motion_run calls dm_rearm_pass() once
// per main-loop pass for every channel whose switches are wired, with the printer's command for it
// mapped by dm_host_from_motion().
//
// In idle, a channel that is not loaded and whose key reads 'both' (both switches closed) gets
// Stage-2: 120 mm pushed open-loop at 900 PWM, to make freshly inserted filament print-ready. A
// loaded channel used to be marked unloaded by any 100 ms with the key away from 'both', whatever
// the filament did. So a key voltage that stayed below the 1.7 V 'both' threshold for 100 ms
// (noise, supply ripple, a vibrating lever) got the parked filament another 120 mm pushed into the
// printer's filament path as soon as the key read 'both' again. A printer load that started from
// 'external only' (e.g. after a boot with the key there) left the channel unloaded through the
// whole print, so the same push followed the unload.
//
// A loaded channel is now marked unloaded only by a real event: the filament left both switches
// (key 'none'), or the gear retracted it by DM_REARM_RETRACT_CNT in one excursion of the key away
// from 'both' that lasted DM_REARM_AWAY_MS, while the printer did not move the channel. The printer
// treating the filament as loaded with the key at 'both' marks the channel loaded.
#include <stdbool.h>
#include <stdint.h>

#include "ams.h"

// MC_ONLINE_key_stu of the DM board (dm_key_to_state): 0 none, 1 both, 2 external only, 3 other.
#define DM_KEY_NONE 0u
#define DM_KEY_BOTH 1u

// Unchanged: the key must stay away from 'both' this long.
#define DM_REARM_AWAY_MS 100u

// And by then the gear must have retracted the filament 10 mm since the key left 'both' (1739
// counts of 5.75 um = 10.003 mm), the audit's suggestion. The gear position (as5600_count) rises
// while the motor retracts and falls while it pushes. A key flicker by itself moves nothing, and
// the AS5600 jitters by +-2 counts at rest. The BMCU's own retract out of 'both', the auto-unload
// (850 PWM), keeps pulling for 1.5 s once the key has left 'both' (AUTO_UNLOAD_EMPTY_MS) unless
// the buffer falls below 35%: 18 mm even at 12 mm/s, the pull back's end speed (PWM floor 400).
#define DM_REARM_RETRACT_CNT 1739u

// Who moves the channel, from the printer's command for it and the BMCU's unload.
typedef enum
{
    DM_HOST_IDLE = 0,  // the printer commands idle (or another channel is active): only the BMCU's
                       // idle control, its auto-unload or the autoload itself move the filament
    DM_HOST_LOADED,    // the printer treats the filament as loaded: before_on_use, on_use or
                       // stop_on_use (the BMCU saves the channel as loaded on the first two)
    DM_HOST_MOVING,    // the printer loads or unloads the channel (send_out, before_pull_back,
                       // pull_back), or the BMCU's pull back / redetect of an unload still runs
} dm_host_t;

// active: the printer's active channel (A.now_filament_num) is this one; motion: what it commands
// for it (A.filament[ch].motion); unloading: the channel is in filament_pulling_back or
// filament_redetect.
static inline dm_host_t dm_host_from_motion(bool active, _filament_motion motion, bool unloading)
{
    if (unloading) return DM_HOST_MOVING;
    if (!active) return DM_HOST_IDLE;
    switch (motion)
    {
    case _filament_motion::idle:          return DM_HOST_IDLE;
    case _filament_motion::before_on_use:
    case _filament_motion::on_use:
    case _filament_motion::stop_on_use:   return DM_HOST_LOADED;
    default:                              return DM_HOST_MOVING;
    }
}

typedef enum
{
    DM_REARM_NONE = 0,   // loaded unchanged
    DM_REARM_EMPTY,      // key 'none': not loaded, the filament is out (as before)
    DM_REARM_RETRACTED,  // loaded -> not loaded: retracted out of 'both'; Stage-2 is armed again
    DM_REARM_LOADED,     // not loaded -> loaded: the printer treats it as loaded, key at 'both'
} dm_rearm_event;

// One main-loop pass for one channel. loaded: dm_loaded[ch]. away_t0_ms / away_cnt: the current
// excursion of a loaded channel's key away from 'both' (dm_loaded_drop_t0_ms / dm_loaded_drop_cnt):
// its first pass and the gear position then; away_t0_ms == 0 means none, and away_cnt is only read
// while one runs. ks: MC_ONLINE_key_stu[ch]; pos_cnt: as5600_count[ch] (wraps; only differences are
// used). On any event other than DM_REARM_NONE the caller restarts the autoload from IDLE.
static inline dm_rearm_event dm_rearm_pass(uint8_t *loaded, uint64_t *away_t0_ms, uint32_t *away_cnt,
                                           uint8_t ks, dm_host_t host, uint64_t now_ms, uint32_t pos_cnt)
{
    if (ks == DM_KEY_NONE)
    {
        // both switches open: the filament is out (as before)
        *loaded     = 0u;
        *away_t0_ms = 0u;
        return DM_REARM_EMPTY;
    }

    if (*loaded == 0u)
    {
        *away_t0_ms = 0u;
        if ((ks == DM_KEY_BOTH) && (host == DM_HOST_LOADED))
        {
            *loaded = 1u;
            return DM_REARM_LOADED;
        }
        return DM_REARM_NONE; // Stage-1 / Stage-2 decide in idle, as before
    }

    // Back at 'both', or the printer / the unload moves the filament: the excursion is over. A
    // retract by the printer does not count, so a key misread during its unload cannot arm the push
    // that would follow in idle.
    if ((ks == DM_KEY_BOTH) || (host != DM_HOST_IDLE))
    {
        *away_t0_ms = 0u;
        return DM_REARM_NONE;
    }

    if (*away_t0_ms == 0u)
    {
        *away_t0_ms = now_ms;
        *away_cnt   = pos_cnt;
        return DM_REARM_NONE;
    }

    const int32_t retracted = (int32_t)(pos_cnt - *away_cnt);
    if (((now_ms - *away_t0_ms) >= DM_REARM_AWAY_MS) && (retracted >= (int32_t)DM_REARM_RETRACT_CNT))
    {
        *loaded     = 0u;
        *away_t0_ms = 0u;
        return DM_REARM_RETRACTED;
    }
    return DM_REARM_NONE;
}
