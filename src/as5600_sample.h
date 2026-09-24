#pragma once
// Which AS5600 angle reads may be used, for the distance/speed tracker (AS5600_distance_updata) and
// the motor direction test (MOTOR_get_dir). Hardware-free, tested on the host (test/test_as5600_sample).
//
// On a NACK the driver (many_soft_AS5600.cpp, updata_angle) sets raw_angle = 0 and online = false.
// The health gate only drops a channel after kAS5600_FAIL_TRIP (3) failed reads in a row, so failures
// 1 and 2 used to be consumed as a real angle of 0: a fake move of up to half a turn (2048 counts,
// 11.8 mm) and back, seen by every distance consumer (pull back, DM autoload, send limit). Taken as
// the MOTOR_get_dir baseline, the same 0 made the first good read look like a move in a random
// direction, which was then saved to flash.
#include <stdbool.h>
#include <stdint.h>

#define AS5600_COUNTS_PER_TURN 4096
#define AS5600_HALF_TURN       2048

// RAW ANGLE is 12 bits: the top 4 bits of its high byte always read 0. The soft-I2C driver only sees
// a NACK on the address and register bytes, never a corrupted data byte, so an answered read above
// 4095 (for example 0xFFFF when SDA opens during the data phase) is garbage: it is a failed read like
// a NACK, for the health gate (AS5600_distance_updata), the tracker and the direction test alike.
#define AS5600_RAW_MAX 4095u

static inline bool as5600_raw_ok(uint16_t raw)
{
    return raw <= AS5600_RAW_MAX;
}

// Shortest signed move from `last` to `now` in counts, -2048..2048 (the wrap the firmware always used).
static inline int32_t as5600_angle_diff(int32_t now, int32_t last)
{
    int32_t d = now - last;
    if (d > AS5600_HALF_TURN) d -= AS5600_COUNTS_PER_TURN;
    if (d < -AS5600_HALF_TURN) d += AS5600_COUNTS_PER_TURN;
    return d;
}

// ===== distance/speed tracker =====

// Fastest believable gear speed: 150 counts per ms. One count is pi * 7.5 mm / 4096 = 5.75 um of
// filament (kAS5600_MM_PER_CNT), so this is 863 mm/s, 2200 rpm of the 7.5 mm drive wheel. The
// firmware never commands more than 60 mm/s (PULL_V_FAST and the send speed, about 10 counts/ms), a
// 14x margin. The open-loop moves (PWM 850-1000: DM autoload, redetect, auto unload, kicks) are not
// speed-controlled and their speed is not known here. The limit is high on purpose, since rejecting
// real motion would make the distance undercount: even a 1000 rpm gear output (393 mm/s) keeps a 2x
// margin. A read corrupted into a random angle still lands within +-150 counts of the right one only
// 301/4096 = 7.3% of the time, and then adds at most 0.86 mm instead of up to 11.8 mm.
#define AS5600_MAX_COUNTS_PER_MS 150u

// After this many implausible reads in a row the baseline itself is suspect (the read it was taken
// from may have been corrupted), so tracking restarts from the current read. Same count as the health
// gate's kAS5600_FAIL_TRIP. It loses at most 3 reads of real motion (about 3 ms, 0.18 mm at 60 mm/s)
// instead of adding a fake jump, and the distance can never stop advancing for good.
#define AS5600_JUMP_REBASE 3u

// Largest believable move over `elapsed_us` since the last accepted read. It grows with the elapsed
// time, so a skipped read or a slow main-loop pass does not turn real motion into a jump. At least one
// 1 ms poll period's worth; at most half a turn (from 13.65 ms on), which is all the wrap can resolve.
static inline int32_t as5600_max_step(uint32_t elapsed_us)
{
    const uint32_t full_us = ((uint32_t)AS5600_HALF_TURN * 1000u) / AS5600_MAX_COUNTS_PER_MS;
    if (elapsed_us >= full_us) return AS5600_HALF_TURN;

    uint32_t n = (elapsed_us * AS5600_MAX_COUNTS_PER_MS + 999u) / 1000u;
    if (n < AS5600_MAX_COUNTS_PER_MS) n = AS5600_MAX_COUNTS_PER_MS;
    return (int32_t)n;
}

typedef struct
{
    int32_t  angle; // raw angle of the last accepted read: the next read is diffed against it
    uint32_t t;     // SysTick of that read
    uint8_t  valid; // angle and t are set
    uint8_t  jumps; // implausible reads in a row
} as5600_track_t;

// as5600_track_sample() results.
enum
{
    AS5600_TRACK_OFF = 0, // channel fails the health gate: baseline dropped, speed 0
    AS5600_TRACK_SKIP,    // failed read: baseline kept, no distance, speed held
    AS5600_TRACK_JUMP,    // implausible move: baseline kept, no distance, speed held
    AS5600_TRACK_REBASE,  // new baseline from this read: no distance, speed 0
    AS5600_TRACK_MOVE     // moved *diff counts in *dt ticks since the last accepted read
};

static inline void as5600_track_reset(as5600_track_t *s)
{
    s->angle = 0;
    s->t     = 0u;
    s->valid = 0u;
    s->jumps = 0u;
}

// One poll of one channel. good: the channel passes the health gate; ok: this read was answered
// (online, magnet present). now: SysTick of the read; ticks_per_us: time_hw_tpus.
static inline uint8_t as5600_track_sample(as5600_track_t *s, bool good, bool ok, uint16_t raw,
                                          uint32_t now, uint32_t ticks_per_us,
                                          int32_t *diff, uint32_t *dt)
{
    *diff = 0;
    *dt   = 0u;

    if (!good)
    {
        s->valid = 0u;
        s->jumps = 0u;
        return AS5600_TRACK_OFF;
    }

    // raw is 0 (NACK) or above 12 bits (corrupted) here, not an angle, and never a baseline
    if (!ok || !as5600_raw_ok(raw)) return AS5600_TRACK_SKIP;

    if (!s->valid)
    {
        s->angle = (int32_t)raw;
        s->t     = now;
        s->valid = 1u;
        s->jumps = 0u;
        return AS5600_TRACK_REBASE;
    }

    const uint32_t elapsed = (uint32_t)(now - s->t);
    if (elapsed == 0u) return AS5600_TRACK_SKIP;

    if (!ticks_per_us) ticks_per_us = 1u;

    const int32_t d = as5600_angle_diff((int32_t)raw, s->angle);
    const int32_t lim = as5600_max_step(elapsed / ticks_per_us);

    if ((d > lim) || (d < -lim))
    {
        s->jumps++;
        if (s->jumps < AS5600_JUMP_REBASE) return AS5600_TRACK_JUMP;

        s->angle = (int32_t)raw;
        s->t     = now;
        s->jumps = 0u;
        return AS5600_TRACK_REBASE;
    }

    s->angle = (int32_t)raw;
    s->t     = now;
    s->jumps = 0u;
    *diff = d;
    *dt   = elapsed;
    return AS5600_TRACK_MOVE;
}

// ===== motor direction test (MOTOR_get_dir) =====
// The motor runs at +PWM from rest until the gear has turned AS5600_DIR_MOVE_COUNTS from a baseline
// read; the sign of that move is the direction saved to flash. A corrupted read can only decide it if
// it repeats: the baseline needs two agreeing reads and the move three agreeing reads, all answered
// and within 12 bits.

// 163 counts = 0.94 mm of filament (14 degrees of the gear), unchanged: far above the sensor noise.
#define AS5600_DIR_MOVE_COUNTS 163

// Answered reads in a row, with the motor stopped, that must agree within AS5600_DIR_BASE_TOL counts
// before the baseline is taken (the newer one is used). A corrupted baseline read would make every
// later read look like a move in a random direction, even with no motion at all.
#define AS5600_DIR_BASE_AGREE 2u

// 32 counts (0.18 mm): 16x the +-2 count noise of a gear at rest, and a fifth of the move threshold.
// A baseline that is off by up to 32 counts only shifts where the move is seen (after 131 to 195
// counts of real travel instead of 163); it can neither invert the sign nor see a move where there is
// none. Two corrupted reads in a row that land within 32 counts of each other are needed to fool it.
#define AS5600_DIR_BASE_TOL 32

// Reads that must all see the move, with the same sign, before the direction is accepted: the one
// that first sees it (with the motor on, more than AS5600_DIR_MOVE_COUNTS), then two more with the
// motor stopped (as before, the motor stops at the first one), which see the gear at rest. Stopping
// first keeps the gear as close to the baseline as the old one-read test did (far from the half-turn
// wrap), and costs 20 ms.
#define AS5600_DIR_AGREE 3u

// The two reads at rest only have to show the move by more than half the threshold, 81 counts
// (0.47 mm, still 40x the noise), with the same sign. With the H-bridge braked the gear is held only
// by friction, and the buffer spring or filament held at the toolhead may push it part of the way back
// toward the baseline, where it rested before the test (it is not expected to go past it). The sign is
// what gets saved; the size of the move says little about a corrupted read: a random 12-bit value is
// more than 81 counts away on a given side 48% of the time, and more than 163 counts 46% of the time.
#define AS5600_DIR_HOLD_COUNTS (AS5600_DIR_MOVE_COUNTS / 2)

// Passes (10 ms apart) without a baseline that may fail (a failed read, or an answered read that
// disagrees with the one before) before the channel is given up and left untested (dir stays 0,
// nothing saved). The main loop calls a sensor bad after 3 failed reads in a row; 10 failures, at
// least 90 ms, is no transient.
#define AS5600_DIR_BASE_TRIES 10u

// Once the motor has started, failed reads in a row that abort the channel (nothing saved). Same count
// as the main-loop health gate (kAS5600_FAIL_TRIP): 1 or 2 are a transient, during which the motor is
// stopped and nothing is counted toward the result.
#define AS5600_DIR_FAIL_TRIP 3u

enum
{
    AS5600_DIR_BASE = 0, // motor off: waiting for AS5600_DIR_BASE_AGREE reads to agree on a baseline
    AS5600_DIR_RUN,      // motor on: waiting for the gear to move
    AS5600_DIR_CONFIRM,  // motor off: the move is being re-read
    AS5600_DIR_DONE,     // direction found (dir = +1 or -1)
    AS5600_DIR_ABORT     // sensor failed: nothing may be saved for this channel
};

typedef struct
{
    int32_t base;  // baseline raw angle (motor at rest); in BASE, the candidate read
    uint8_t phase;
    uint8_t tries; // failed passes in BASE
    uint8_t agree; // reads in a row that agree: on the baseline (BASE), on the move with sign `sign`
    uint8_t fails; // failed reads in a row once the motor has started (RUN / CONFIRM)
    int8_t  sign;
    int8_t  dir;   // result once phase == AS5600_DIR_DONE
} as5600_dir_probe_t;

static inline void as5600_dir_probe_init(as5600_dir_probe_t *p)
{
    p->base  = 0;
    p->phase = AS5600_DIR_BASE;
    p->tries = 0u;
    p->agree = 0u;
    p->fails = 0u;
    p->sign  = 0;
    p->dir   = 0;
}

// The motor only runs while the gear is being watched: in RUN, and not after a failed read.
static inline bool as5600_dir_probe_motor_on(const as5600_dir_probe_t *p)
{
    return (p->phase == AS5600_DIR_RUN) && (p->fails == 0u);
}

// Feed one read (ok: online). Returns the new phase.
static inline uint8_t as5600_dir_probe_step(as5600_dir_probe_t *p, bool ok, uint16_t raw)
{
    // above 12 bits: corrupted data bytes, a failed read like a NACK
    if (!as5600_raw_ok(raw)) ok = false;

    switch (p->phase)
    {
    case AS5600_DIR_BASE:
        if (!ok)
        {
            // raw is 0 or garbage here: never a baseline, and it breaks the run of agreeing reads
            p->agree = 0u;
            p->tries++;
        }
        else if (p->agree == 0u)
        {
            p->base  = (int32_t)raw; // first candidate
            p->agree = 1u;
        }
        else
        {
            const int32_t d = as5600_angle_diff((int32_t)raw, p->base);
            p->base = (int32_t)raw;
            if ((d <= AS5600_DIR_BASE_TOL) && (d >= -AS5600_DIR_BASE_TOL))
            {
                p->agree++;
            }
            else
            {
                // one of the two is corrupted: the newer read is the new candidate
                p->agree = 1u;
                p->tries++;
            }
        }

        if (p->agree >= AS5600_DIR_BASE_AGREE)
        {
            p->agree = 0u;
            p->phase = AS5600_DIR_RUN;
        }
        else if (p->tries >= AS5600_DIR_BASE_TRIES)
        {
            p->phase = AS5600_DIR_ABORT;
        }
        break;

    case AS5600_DIR_RUN:
    case AS5600_DIR_CONFIRM:
    {
        if (!ok)
        {
            // No angle this pass: the motor is held stopped (as5600_dir_probe_motor_on) and nothing
            // is counted toward the result. Only a sensor that stays lost aborts the channel, and
            // then nothing is saved (as before).
            p->fails++;
            if (p->fails >= AS5600_DIR_FAIL_TRIP) p->phase = AS5600_DIR_ABORT;
            break;
        }
        p->fails = 0u;

        const int32_t d = as5600_angle_diff((int32_t)raw, p->base);
        const int32_t m = (d < 0) ? -d : d;
        const int8_t  s = (d > 0) ? (int8_t)1 : (int8_t)-1;

        if ((p->phase == AS5600_DIR_CONFIRM) && (s == p->sign) && (m > AS5600_DIR_HOLD_COUNTS))
        {
            p->agree++; // re-read at rest: the move is still there
        }
        else if (m > AS5600_DIR_MOVE_COUNTS)
        {
            // the first read that sees the move (or a move the other way): count again from it
            p->sign  = s;
            p->agree = 1u;
        }
        else
        {
            // no move (after all): run the motor again
            p->agree = 0u;
            p->phase = AS5600_DIR_RUN;
            break;
        }

        if (p->agree >= AS5600_DIR_AGREE)
        {
            p->dir   = p->sign;
            p->phase = AS5600_DIR_DONE;
        }
        else
        {
            p->phase = AS5600_DIR_CONFIRM;
        }
        break;
    }

    default:
        break;
    }

    return p->phase;
}
