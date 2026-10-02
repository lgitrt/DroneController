/*
 * test_imu_kf.c — Host-side characterization tests for the attitude estimators
 * in IMU_KF.c (the fixed-gain roll/pitch Kalman filter and the complementary
 * filter actually wired up in main.c).
 *
 * Author: Luca Obwegs
 *
 * Builds and runs natively (no ARM toolchain or hardware needed): IMU_KF.c
 * only depends on <math.h>/<stdio.h>/<stdlib.h>, not the HAL. See
 * tests/Makefile.
 *
 * Both filters keep their state in file-scope globals rather than an
 * explicit init/reset function, so each test below drives enough samples
 * for the filter to converge from whatever state the previous test left it
 * in, instead of assuming a fresh start.
 */
#include "test_util.h"
#include "IMU_KF.h"

/* Both callKF() and callCompFilter() derive phiS/thetaS purely from the
 * accelerometer via atan2(a_y, a_z) and atan2(a_x, a_z); picking
 * ax = sin(angle), az = cos(angle) makes that reading exactly `angle`,
 * independent of the library's own trig rounding. */
static void accel_for_tilt(double angle, double *a_level_axis, double *a_z)
{
    *a_level_axis = sin(angle);
    *a_z = cos(angle);
}

static void test_level_accel_holds_zero(void)
{
    struct est kf, cf;
    for (int i = 0; i < 50; i++) {
        kf = callKF(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.01);
        cf = callCompFilter(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.01);
    }
    CHECK_NEAR(kf.phi, 0.0, 1e-9, "KF roll must stay exactly level with no tilt or rotation");
    CHECK_NEAR(kf.theta, 0.0, 1e-9, "KF pitch must stay exactly level with no tilt or rotation");
    CHECK_NEAR(cf.phi, 0.0, 1e-9, "comp. filter roll must stay exactly level with no tilt or rotation");
    CHECK_NEAR(cf.theta, 0.0, 1e-9, "comp. filter pitch must stay exactly level with no tilt or rotation");
}

/* ax = sin(angle), az = cos(angle), ay = 0 is a pure-pitch tilt reading
 * (phiS stays 0 since atan2(0, az) == 0); the mirror image (ay = sin(angle),
 * ax = 0) is a pure-roll tilt. Both filters' phiS/thetaS calculations share
 * a single az, so only one axis can be driven away from zero at a time with
 * a physically meaningful reading. */
static void test_kf_converges_to_static_tilt(void)
{
    const double roll_angle = 0.30;
    double ay, az;
    accel_for_tilt(roll_angle, &ay, &az);

    struct est kf = {0};
    for (int i = 0; i < 4000; i++) {
        kf = callKF(0.0, ay, az, 0.0, 0.0, 0.0, 0.01);
    }
    CHECK_NEAR(kf.phi, roll_angle, 1e-6, "KF roll should converge to a sustained accelerometer tilt");
    CHECK_NEAR(kf.theta, 0.0, 1e-6, "KF pitch must stay level while only roll is tilted");
}

static void test_comp_filter_converges_to_static_tilt(void)
{
    const double pitch_angle = 0.25;
    double ax, az;
    accel_for_tilt(pitch_angle, &ax, &az);

    struct est cf = {0};
    for (int i = 0; i < 4000; i++) {
        cf = callCompFilter(ax, 0.0, az, 0.0, 0.0, 0.0, 0.01);
    }
    CHECK_NEAR(cf.theta, pitch_angle, 1e-6, "comp. filter pitch should converge to a sustained accelerometer tilt");
    CHECK_NEAR(cf.phi, 0.0, 1e-6, "comp. filter roll must stay level while only pitch is tilted");
}

/* callCompFilter() never sets estComp.psi, so the struct returned to the
 * caller (which in main.c forces estimate.psi = 0 itself) must not have a
 * stale/garbage yaw value carried over between calls. */
static void test_comp_filter_leaves_yaw_untouched(void)
{
    struct est cf = callCompFilter(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.01);
    CHECK_NEAR(cf.psi, 0.0, 1e-9, "comp. filter must not populate a yaw estimate (no magnetometer input)");
}

int main(void)
{
    test_level_accel_holds_zero();
    test_kf_converges_to_static_tilt();
    test_comp_filter_converges_to_static_tilt();
    test_comp_filter_leaves_yaw_untouched();
    TEST_SUMMARY();
}
