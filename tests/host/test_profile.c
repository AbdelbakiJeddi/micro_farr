// SRCS: main/tasks/motion/motion_profile.c
#include <math.h>

#include "motion_profile.h"
#include "test.h"

static const profile_limits_t L = { .v_max = 100, .accel = 1000, .decel = 1000, .v_min = 5 };

static void test_accel_limited(void)
{
    CHECK_NEAR(profile_next(0, 1000, 0.01f, &L), 10, 1e-4);
}

static void test_cruise_cap(void)
{
    CHECK_NEAR(profile_next(100, 1000, 0.01f, &L), 100, 1e-4);
}

static void test_brake_curve(void)
{
    CHECK_NEAR(profile_next(100, 2, 0.01f, &L), sqrtf(4000), 1e-3);
}

static void test_floor_always_finishes(void)
{
    CHECK_NEAR(profile_next(50, 0, 0.01f, &L), 5, 1e-4);
}

static void test_overshoot_no_nan(void)
{
    CHECK_NEAR(profile_next(50, -3, 0.01f, &L), 5, 1e-4);
}

int main(void)
{
    test_accel_limited();
    test_cruise_cap();
    test_brake_curve();
    test_floor_always_finishes();
    test_overshoot_no_nan();
    return test_result();
}
