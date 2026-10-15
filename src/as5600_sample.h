#pragma once
// Which AS5600 angle reads the distance/speed tracker (AS5600_distance_updata) may use.
// Hardware-free, tested on the host (test/test_as5600_sample).
//
// The driver (many_soft_AS5600.cpp, updata_angle) returns a mask of the valid reads (answered, and
// within 12 bits); on a failed read it sets online = false and leaves raw_angle at the last valid
// angle. The health gate only drops a channel after kAS5600_FAIL_TRIP (3) failed reads in a row, so
// failures 1 and 2 must be skipped, not consumed as an angle: V10.5's driver set raw_angle = 0 there,
// a fake move of up to half a turn (2048 counts, 11.8 mm) and back, seen by every distance consumer
// (pull back, DM autoload, send limit). The tracker skips them and also rejects an answered read that
// implies an impossible jump, which the driver's mask cannot see. The motor direction test
// (MOTOR_get_dir) checks its reads itself (V11's phases).
#include <stdbool.h>
#include <stdint.h>

#define AS5600_COUNTS_PER_TURN 4096
#define AS5600_HALF_TURN       2048

// RAW ANGLE is 12 bits: the top 4 bits of its high byte always read 0. The soft-I2C driver only sees
// a NACK on the address and register bytes, never a corrupted data byte, so an answered read above
// 4095 (for example 0xFFFF when SDA opens during the data phase) is garbage: it is a failed read like
// a NACK, for the health gate (AS5600_distance_updata) and the tracker alike. The driver already
// leaves such a read out of its valid mask; the tracker checks it again.
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

// One poll of one channel. good: the channel passes the health gate; ok: this read is valid (in the
// driver's mask, magnet present). now: SysTick of the read; ticks_per_us: time_hw_tpus.
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

    // a failed read (raw: the last valid angle, or above 12 bits): no new angle, never a baseline
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
