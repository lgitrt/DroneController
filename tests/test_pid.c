/*
 * test_pid.c — Host-side characterization tests for the attitude/thrust PID
 * loop and motor-mixing law in PID.c.
 *
 * Author: Luca Obwegs
 *
 * Builds and runs natively (no ARM toolchain or hardware needed): PID.c only
 * depends on <math.h>, not the HAL. See tests/Makefile.
 *
 * callPID() keeps its error/derivative history in file-scope globals, but
 * with roll == rollRef (etc.) on the very first call the derivative term is
 * zero (e_*_old starts at 0 too), so these tests can reason about a single
 * deterministic call instead of needing to converge over many iterations.
 */
#include "test_util.h"
#include "PID.h"

#define DT 0.001f

static void test_hover_gives_four_equal_motor_speeds(void)
{
    struct motSpeeds m = callPID(0.0f, 0.0f, 0.0f, 0.0f, 40.0f, 0.0f, 0.0f, DT);
    double expected = sqrt(40.0 / (4.0 * kt));

    CHECK_NEAR(m.w1, expected, 1e-2, "pure-hover thrust command should map to the textbook per-motor speed");
    CHECK(m.w1 == m.w2 && m.w2 == m.w3 && m.w3 == m.w4,
          "with zero roll/pitch/yaw error, all four motors must spin at the same speed");
}

static void test_zero_thrust_demand_clips_to_zero(void)
{
    struct motSpeeds m = callPID(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, DT);
    CHECK(m.w1 == 0.0 && m.w2 == 0.0 && m.w3 == 0.0 && m.w4 == 0.0,
          "zero thrust demand with no attitude error must not drive any motor below its zero floor");
}

static void test_huge_thrust_demand_clips_to_w_max(void)
{
    struct motSpeeds m = callPID(0.0f, 0.0f, 0.0f, 0.0f, 1.0e6f, 0.0f, 0.0f, DT);
    CHECK_NEAR(m.w1, w_max, 1e-6, "an over-range thrust demand must saturate at w_max, not exceed it");
    CHECK_NEAR(m.w4, w_max, 1e-6, "an over-range thrust demand must saturate at w_max, not exceed it");
}

/* A positive roll error (rollRef > roll) should speed up the motors on one
 * side of the roll axis and slow down the motors on the other, while leaving
 * the pitch-axis pair (w2/w4) balanced since pitch error is zero. */
static void test_positive_roll_error_differentiates_roll_motors(void)
{
    struct motSpeeds level = callPID(0.0f, 0.0f, 0.0f, 0.0f, 40.0f, 0.0f, 0.0f, DT);
    struct motSpeeds rolled = callPID(0.0f, 0.0f, 0.0f, 0.2f, 40.0f, 0.0f, 0.0f, DT);

    CHECK(rolled.w1 > level.w1, "positive roll error must increase w1's commanded speed");
    CHECK(rolled.w3 < level.w3, "positive roll error must decrease w3's commanded speed");
    CHECK_NEAR(rolled.w2, rolled.w4, 1e-9, "pure roll error must not unbalance the pitch-axis motor pair");
}

int main(void)
{
    test_hover_gives_four_equal_motor_speeds();
    test_zero_thrust_demand_clips_to_zero();
    test_huge_thrust_demand_clips_to_w_max();
    test_positive_roll_error_differentiates_roll_motors();
    TEST_SUMMARY();
}
