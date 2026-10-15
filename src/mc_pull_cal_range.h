#pragma once
// The low-side floor of the boot buffer calibration (MC_PULL_calibration.cpp). Hardware-free, so it
// is tested on the host (test/test_mc_pull_cal_range).
//
// Each inserted buffer is calibrated in two steps: move it to one end and back (this step also
// learns the Hall polarity), then to the other end and back. A step counts once the reading has
// left the idle centre by MC_PULL_CAL_PRESS_DELTA_V and has settled back within
// MC_PULL_CAL_CENTER_EPS_V for CAL_STABLE_MS; it gives up after 30 s. All values are normalised:
// MC_PULL_V_OFFSET puts the idle reading at 1.65 V and the polarity puts the first step's end
// below it. Since V11 a step that times out, or a range that fails its checks, fails the whole run:
// nothing is saved, every buffer stays invalid (MC_PULL_calibration_is_valid, steady red LED, no
// drive), and the next boot calibrates again.
//
// pull_v_to_percent_f() (Motion_control.cpp) maps the low side linearly: p % below 50 sits
// (50 - p) / 50 of the span below centre. V11 keeps any first step that counted, so a 100 mV low
// side put the on_use jam latch (< 40 %, jam_latch.h) 20 mV below centre, inside the window the
// calibration itself accepts as "back at rest", and let the recalibration gesture's 15 % rule fire
// 69 mV below centre, before its own absolute 100 mV rule (audit finding 8). A first step is kept
// only if its end spans at least MC_PULL_CAL_MIN_LOW_SPAN_V: the jam latch then sits at least
// 1.5 x MC_PULL_CAL_CENTER_EPS_V below centre, and the 15 % rule (pct rounds, so < 15.5 %:
// 0.69 x span) needs at least 131 mV. The second step counts only past MC_PULL_CAL_PRESS_DELTA_V,
// so a stored high side spans more than that and the load stops (90/95/97 %) trip within it.

#include <stdbool.h>
#include <stdint.h>

#define MC_PULL_CAL_PRESS_DELTA_V  0.10f  // smallest move a calibration step accepts
#define MC_PULL_CAL_CENTER_EPS_V   0.025f // a step is back at rest within this of centre (V11)
#define MC_PULL_CAL_MIN_LOW_SPAN_V 0.19f  // narrowest first-step end that is kept: 0.2 x it >= 1.5 x EPS

// The first step's end is kept: the step did not time out and its end spans the low-side floor.
static inline bool mc_pull_cal_low_ok(float center_v, bool ok_min, float cap_min)
{
    return ok_min && ((center_v - cap_min) >= MC_PULL_CAL_MIN_LOW_SPAN_V);
}
