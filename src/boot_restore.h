#pragma once
// Boot restore of the loaded channel (main.cpp): the channel the printer had loaded into the
// extruder at power-off (NVM STA record, g_loaded_ch) and what the BMCU reports for it until the
// printer takes it over. Hardware-free, so the decisions are tested on the host
// (test/test_boot_restore).
//
// Since V8 the BMCU boots with that channel in use, as the printer left it: now_filament_num = ch,
// filament_use_flag 0x04, motion on_use. This is what lets a print resume after a power cut, and
// the printer's unload after it get the BMCU's retract (before_pull_back, pull back). The idle poll
// (0xFF, statu 0x01) never clears it. Upstream #148 (A1 01.08.01.00: after a power cycle with a
// channel loaded, the printer no longer finds the AMS) may come from this in-use report at
// power-up; that is not proven, hence the A/B build option below.
#include <stdbool.h>
#include <stdint.h>

#include "ams.h"

// platformio.ini: -DBMCU_BOOT_RESTORE_LOADED=0 (A/B image for #148,
// env:a1_solo_autoload_rgboff_no_boot_restore).
// 1: report the restored channel in use from boot (the behaviour since V8).
// 0: keep g_loaded_ch (set_motion's gating of stop_on_use/before_pull_back needs it), but boot with
//    every channel idle, use_flag 0 and now_filament_num 0xFF. The in-use state is applied only when
//    the printer first references the channel (boot_restore_references), right before set_motion
//    acts on that command, so set_motion then acts as with 1. Until then the channel's motor runs
//    the idle control instead of the on_use control that 1 starts at the first heartbeat.
#ifndef BMCU_BOOT_RESTORE_LOADED
#define BMCU_BOOT_RESTORE_LOADED 1
#endif

// restore_at_boot, in every call below: BMCU_BOOT_RESTORE_LOADED != 0 (a constant in the firmware,
// so each image only carries its own mode's code).
typedef struct
{
    uint8_t ch;  // restored channel the printer has not referenced yet; 0xFF: none
} boot_restore_t;

// The in-use state restored for channel ch, as main.cpp has always set it at boot.
static inline void boot_restore_seed(_ams *a, uint8_t ch)
{
    a->now_filament_num  = ch;
    a->filament_use_flag = 0x04;
    a->pressure          = 0x2B00;

    for (uint8_t i = 0; i < 4u; i++)
        a->filament[i].motion = _filament_motion::idle;

    a->filament[ch].motion = _filament_motion::on_use;
}

// At boot, with the channel read from the STA record (g_loaded_ch). ch >= 4: none was loaded.
static inline void boot_restore_init(boot_restore_t *r, _ams *a, uint8_t ch, bool restore_at_boot)
{
    r->ch = 0xFFu;
    if (ch >= 4u) return;

    r->ch = ch;
    if (restore_at_boot) boot_restore_seed(a, ch);
}

// Does a printer motion command (set_motion's read_num, statu_flags, motion flag) reference
// channel ch? Every command for ch does, and 0xFF/0x03/0x00, which unloads the AMS's current
// channel (ch, once restored). The idle poll 0xFF/0x01, the other 0xFF commands and commands for
// other channels do not.
static inline bool boot_restore_references(uint8_t ch, uint8_t read_num, uint8_t statu_flags, uint8_t motion_flag)
{
    if (read_num < 4u) return read_num == ch;
    return (read_num == 0xFFu) && (statu_flags == 0x03u) && (motion_flag == 0x00u);
}

// set_motion, before it acts on a command. The first command that references the restored channel
// hands it to the printer; a deferred restore (option 0) is applied first.
static inline void boot_restore_on_command(boot_restore_t *r, _ams *a, bool restore_at_boot, uint8_t read_num,
                                           uint8_t statu_flags, uint8_t motion_flag)
{
    if (r->ch >= 4u) return;
    if (!boot_restore_references(r->ch, read_num, statu_flags, motion_flag)) return;

    if (!restore_at_boot) boot_restore_seed(a, r->ch);
    r->ch = 0xFFu;
}

// ams_state_set_unloaded, right after it has cleared g_loaded_ch, on any path (Motion_control_run
// when the channel's switches read empty, a send_out, ...). A restored state that the printer has
// not referenced yet is stale then: it is undone, so a channel the BMCU knows is empty is not
// reported in use. now_filament_num is only reset if it is still the restored channel (a send_out
// for another channel has already moved it). Once the printer has referenced the channel, its own
// commands own the state: after a runout mid-print, 0xFF/0x03 must still find now_filament_num on
// that channel to pull the rest back, so that state is left alone, as before.
static inline void boot_restore_on_unloaded(boot_restore_t *r, _ams *a, bool restore_at_boot)
{
    if (r->ch >= 4u) return;

    const uint8_t ch = r->ch;
    r->ch = 0xFFu;
    if (!restore_at_boot) return;  // option 0: nothing was restored into RAM

    if (a->now_filament_num == ch)
    {
        a->now_filament_num  = 0xFFu;
        a->filament_use_flag = 0x00u;
    }
    a->filament[ch].motion = _filament_motion::idle;
}

// A restored channel the printer has not referenced yet.
static inline bool boot_restore_pending(const boot_restore_t *r)
{
    return r->ch < 4u;
}

// The AHUB host sets every channel's state itself (all_filament_stu) and never goes through
// set_motion: the restore has nothing left to hand over.
static inline void boot_restore_drop(boot_restore_t *r)
{
    r->ch = 0xFFu;
}
