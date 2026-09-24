#pragma once
// Idle averaging of the boot buffer calibration (MC_PULL_calibration_boot): the mean of
// MC_PULL_CAL_IDLE_SAMPLES filtered ADC readings (ADC_DMA_get_value, volts) per inserted channel,
// for the Hall centre (-> MC_PULL_V_OFFSET) and the DM key idle level (-> MC_DM_KEY_NONE_THRESH).
// Hardware-free, tested on the host (test/test_mc_pull_cal_mean).
//
// The sums are float. They used to be double, and on this FPU-less core that loop alone linked the
// soft-double routines __adddf3, __divdf3, __truncdfsf2 and __extendsfdf2 (3,752 B of flash).
// float is enough: a reading is 0..3.3 V, so a sum stays below 90 x 3.3 V = 297 V, where one float
// step is 2^-15 V (30.5 uV). The first addition is exact and each of the other 89 is off by at most
// half a step, so the mean is off by at most 89 x 15.3 uV / 90 = 15.1 uV, plus 0.12 uV for the
// division: under 1/50 of an ADC LSB (3.3 V / 4095 = 806 uV). The DM key threshold, rounded up
// to 10 mV steps, can only come out one step off the double result for an idle level that close
// to a step.

#define MC_PULL_CAL_IDLE_SAMPLES 90 // readings per channel, about 15 ms apart

// Mean of the n (> 0) readings summed in float into sum.
static inline float mc_pull_cal_mean(float sum, int n)
{
    return sum / (float)n;
}
