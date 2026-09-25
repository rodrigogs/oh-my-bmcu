// Host tests for src/mc_pull_cal_mean.h: the boot buffer calibration now averages its idle readings
// in float instead of double (which linked 3,752 B of soft-double code). For representative sets of
// readings the float mean must match what the old double code stored within half an ADC LSB, and in
// fact within the worst-case float error derived in the header; and the DM key threshold derived
// from it may only differ where the idle level sits within that error of one of its 10 mV steps.
//
// MC_PULL_calibration.cpp itself is not built on the host. Its loop sums each reading with
// float += (as sum_readings() does here) and divides with mc_pull_cal_mean(); the old result is
// reproduced by sum_readings_double(). The host's float arithmetic is IEEE single with
// round-to-nearest, like the target's soft-float library, so the float sums are the target's.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unity.h>

#include "mc_pull_cal_mean.h"

#define N MC_PULL_CAL_IDLE_SAMPLES

// ---- adapted from ADC_DMA.cpp: the full-filter reading scale ----
// A full-filter reading is the sum of 128 samples of both 12-bit
// ADCs (0..1,048,320 counts, 256 per ADC LSB) times kScale128.
#define ACC_MAX     1048320u
#define ACC_PER_LSB 256
static float adc_reading(uint32_t acc)
{
    return (float)acc * (3.3f / (8190.0f * 128.0f));
}

#define ADC_LSB_V  (3.3 / 4095.0)
#define HALF_LSB_V (ADC_LSB_V / 2.0)

// Largest |float mean - old double result| (mc_pull_cal_mean.h): 89 float additions off by at most
// 2^-16 V each (sums < 297 V < 512 V), plus the float division and the old code's rounding of its
// double mean to float, at most 2^-23 V each (means < 4 V), plus 1e-12 V for the double sum itself.
#define BOUND_V ((N - 1) * (1.0 / 65536.0) / N + 2.0 * (1.0 / 8388608.0) + 1e-12)

// ---- MC_PULL_calibration.cpp at this commit: dm_key_round_up_to_centi and dm_key_none_threshold_from_idle, verbatim ----
static inline uint8_t dm_key_round_up_to_centi(float v)
{
    if (v <= 0.0f) return 0u;

    float x = v * 100.0f - 0.0001f;
    int iv = (int)x;
    if ((float)iv < x) iv++;

    if (iv < 0) iv = 0;
    if (iv > 255) iv = 255;
    return (uint8_t)iv;
}

static inline float dm_key_none_threshold_from_idle(float key_value)
{
    const uint8_t key_cv = dm_key_round_up_to_centi(key_value);

    uint8_t thr_cv = (uint8_t)(key_cv + 10u);
    if (thr_cv < 60u) thr_cv = 60u;
    if (thr_cv > 139u) thr_cv = 139u;

    return 0.01f * (float)thr_cv;
}
// ---- end of the MC_PULL_calibration.cpp copy ----

static double abs_d(double a)
{
    return (a < 0.0) ? -a : a;
}

// Distance in volts from v to the nearest idle level where dm_key_round_up_to_centi() steps
// (k + 0.0001) / 100 V.
static double dist_to_key_step(double v)
{
    const double x = v * 100.0 - 0.0001;
    if (x <= 0.0) return abs_d(x) / 100.0;
    double f = x - (double)(long)x;
    if (f > 0.5) f = 1.0 - f;
    return f / 100.0;
}

// Deterministic xorshift64, so a failure always reproduces.
static uint64_t rng_state;
static uint32_t rng(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return (uint32_t)(rng_state >> 16);
}

// Reading at acc +/- up to noise_lsb ADC LSBs, clamped to the ADC range.
static float noisy_reading(uint32_t acc, uint32_t noise_lsb)
{
    int64_t a = (int64_t)acc;
    if (noise_lsb > 0u)
    {
        const uint32_t span = 2u * noise_lsb * ACC_PER_LSB + 1u;
        a += (int64_t)(rng() % span) - (int64_t)noise_lsb * ACC_PER_LSB;
    }
    if (a < 0) a = 0;
    if (a > (int64_t)ACC_MAX) a = ACC_MAX;
    return adc_reading((uint32_t)a);
}

static uint32_t acc_for_volts(double v)
{
    return (uint32_t)(v / 3.3 * (double)ACC_MAX + 0.5);
}

// ---- adapted from MC_PULL_calibration.cpp: MC_PULL_calibration_boot's idle sums, float and (before) double ----
static float sum_readings(const float *r)
{
    float sum = 0.0f;
    for (int k = 0; k < N; k++) sum += r[k];
    return sum;
}

// What the old code stored: a double sum and division, then rounded to float.
static float sum_readings_double(const float *r)
{
    double sum = 0.0;
    for (int k = 0; k < N; k++) sum += r[k];
    return (float)(sum / (double)N);
}

static double worst_err_v;
static uint32_t sets_checked;
static uint32_t key_step_flips;

// One set of N readings: the float mean matches the old result within BOUND_V (and so within half
// an ADC LSB), and the key threshold only differs next to one of its steps.
static void check_set(const float *r)
{
    const float mean_f = mc_pull_cal_mean(sum_readings(r), N);
    const float mean_d = sum_readings_double(r);
    const double err = abs_d((double)mean_f - (double)mean_d);

    TEST_ASSERT_TRUE(err <= HALF_LSB_V);
    TEST_ASSERT_TRUE(err <= BOUND_V);
    if (err > worst_err_v) worst_err_v = err;
    sets_checked++;

    // The offset (MC_PULL_V_OFFSET = 1.65 V - centre) moves by the same amount.
    TEST_ASSERT_TRUE(abs_d((double)(1.65f - mean_f) - (double)(1.65f - mean_d)) <= BOUND_V);

    if (dm_key_none_threshold_from_idle(mean_f) != dm_key_none_threshold_from_idle(mean_d))
    {
        // 1e-6 V covers the float arithmetic of dm_key_round_up_to_centi() (< 0.4 uV).
        TEST_ASSERT_TRUE(dist_to_key_step((double)mean_d) <= err + 1e-6);
        key_step_flips++;
    }
}

static void fill_noisy(float *r, uint32_t acc, uint32_t noise_lsb)
{
    for (int k = 0; k < N; k++) r[k] = noisy_reading(acc, noise_lsb);
}

void setUp(void)
{
    rng_state = 0x9E3779B97F4A7C15ull;
}

void tearDown(void) {}

// The mean returns float (a double here would bring the soft-double library back), over the 90
// readings the bound in mc_pull_cal_mean.h assumes.
static void test_mean_is_float_over_90_readings(void)
{
    TEST_ASSERT_EQUAL_UINT32(sizeof(float), sizeof(mc_pull_cal_mean(0.0f, N)));
    TEST_ASSERT_EQUAL_INT(90, N);
}

// Readings with an exact float sum give their exact mean.
static void test_exact_sums_give_the_exact_mean(void)
{
    float r[N];

    for (int k = 0; k < N; k++) r[k] = 1.5f;
    TEST_ASSERT_EQUAL_FLOAT(1.5f, mc_pull_cal_mean(sum_readings(r), N));
    TEST_ASSERT_TRUE(mc_pull_cal_mean(sum_readings(r), N) == 1.5f);

    for (int k = 0; k < N; k++) r[k] = (k & 1) ? 2.0f : 1.0f;
    TEST_ASSERT_TRUE(mc_pull_cal_mean(sum_readings(r), N) == 1.5f);

    for (int k = 0; k < N; k++) r[k] = 0.0f;
    TEST_ASSERT_TRUE(mc_pull_cal_mean(sum_readings(r), N) == 0.0f);

    for (int k = 0; k < N; k++) r[k] = (k < 45) ? 0.0f : 3.0f;
    TEST_ASSERT_TRUE(mc_pull_cal_mean(sum_readings(r), N) == 1.5f);
}

// Hall idle centres 1.40..1.90 V (nominal 1.65 V) in 1 mV steps, noise up to +/-4 LSB.
static void test_hall_idle_centres_match_the_double_mean(void)
{
    float r[N];
    for (int mv = 1400; mv <= 1900; mv++)
    {
        for (uint32_t noise = 0u; noise <= 4u; noise += 2u)
        {
            fill_noisy(r, acc_for_volts(mv / 1000.0), noise);
            check_set(r);
        }
    }
}

// DM key idle levels from 0 V to full scale, clean and noisy.
static void test_key_idle_levels_match_the_double_mean(void)
{
    static const double levels[] = {0.0, 0.003, 0.05, 0.30, 0.50, 0.60, 1.00, 1.29, 1.39, 1.65, 2.50, 3.30};
    static const uint32_t noises[] = {0u, 1u, 16u, 63u};
    float r[N];

    for (unsigned i = 0; i < sizeof(levels) / sizeof(levels[0]); i++)
    {
        for (unsigned j = 0; j < sizeof(noises) / sizeof(noises[0]); j++)
        {
            fill_noisy(r, acc_for_volts(levels[i]), noises[j]);
            check_set(r);
        }
    }
}

// Full scale gives the largest sums, where float steps are coarsest.
static void test_full_scale_readings_stay_within_the_bound(void)
{
    float r[N];

    fill_noisy(r, ACC_MAX, 0u);
    check_set(r);
    TEST_ASSERT_FLOAT_WITHIN(BOUND_V, 3.3f, mc_pull_cal_mean(sum_readings(r), N));

    for (int t = 0; t < 1000; t++)
    {
        fill_noisy(r, ACC_MAX - (rng() % (8u * ACC_PER_LSB)), 1u + (rng() % 8u));
        check_set(r);
    }
}

// Idle levels 0.5 uV below or above a step of the DM key threshold ((k + 0.0001) / 100 V): only
// there may the float and the double mean give thresholds 10 mV apart (check_set() asserts that
// every difference is this close to a step). 89 readings at +/-8 LSB, the last one sets the mean.
static void test_key_threshold_only_differs_next_to_a_step(void)
{
    static const int steps_cv[] = {50, 65, 100, 128}; // visible steps: thresholds 0.60..1.39 V
    float r[N];
    char msg[80];

    key_step_flips = 0u;
    for (unsigned i = 0; i < sizeof(steps_cv) / sizeof(steps_cv[0]); i++)
    {
        for (int t = 0; t < 500; t++)
        {
            const double target = (steps_cv[i] + 0.0001) / 100.0 + ((t & 1) ? 0.5e-6 : -0.5e-6);
            double sum = 0.0;
            for (int k = 0; k < N - 1; k++)
            {
                r[k] = noisy_reading(acc_for_volts(target), 8u);
                sum += r[k];
            }
            r[N - 1] = (float)(target * N - sum);
            check_set(r);
        }
    }
    snprintf(msg, sizeof(msg), "2000 sets next to a step: key threshold differs in %u", (unsigned)key_step_flips);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE(key_step_flips > 0u);
}

// 200,000 sets anywhere in the ADC range with up to +/-63 LSB of noise, and 100,000 sets of
// arbitrary floats in 0..3.3 V (not on the ADC grid, e.g. before the filter is full).
static void test_random_sets_stay_within_the_bound(void)
{
    float r[N];
    char msg[160];

    worst_err_v = 0.0;
    sets_checked = 0u;
    key_step_flips = 0u;

    for (int t = 0; t < 200000; t++)
    {
        fill_noisy(r, rng() % (ACC_MAX + 1u), rng() % 64u);
        check_set(r);
    }
    for (int t = 0; t < 100000; t++)
    {
        for (int k = 0; k < N; k++) r[k] = (float)((double)rng() / 4294967296.0 * 3.3);
        check_set(r);
    }

    snprintf(msg, sizeof(msg),
             "%u sets: worst |float - double| %.3f uV (%.4f ADC LSB, bound %.2f uV), key threshold differs in %u",
             (unsigned)sets_checked, worst_err_v * 1e6, worst_err_v / ADC_LSB_V, BOUND_V * 1e6,
             (unsigned)key_step_flips);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE(worst_err_v <= BOUND_V);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_mean_is_float_over_90_readings);
    RUN_TEST(test_exact_sums_give_the_exact_mean);
    RUN_TEST(test_hall_idle_centres_match_the_double_mean);
    RUN_TEST(test_key_idle_levels_match_the_double_mean);
    RUN_TEST(test_full_scale_readings_stay_within_the_bound);
    RUN_TEST(test_key_threshold_only_differs_next_to_a_step);
    RUN_TEST(test_random_sets_stay_within_the_bound);
    return UNITY_END();
}
