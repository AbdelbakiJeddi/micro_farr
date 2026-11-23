// SRCS: main/tasks/motion/steering.c main/params.c
#include "app_types.h"
#include "params.h"
#include "steering.h"
#include "test.h"

#define TPM 5.6f   /* TICKS_PER_CELL 1400 / CELL_MM 250 */

static float align_mm(void)
{
    return params_get(P_FRONT_ALIGN_MM);
}

static void test_front_remaining_no_reading(void)
{
    params_reset();
    CHECK(steer_front_remaining(560, -1) == 560);
}

/* 100 mm left by encoder, ToF says 65 mm to the stop point: inside the window. */
static void test_front_remaining_uses_tof_inside_window(void)
{
    params_reset();
    int32_t tof = (int32_t)align_mm() + 65;
    CHECK(steer_front_remaining(560, tof) == (int32_t)(65 * TPM));
}

/* 250 mm left by encoder, ToF reads far too short: spurious, ignored. */
static void test_front_remaining_ignores_outside_window(void)
{
    params_reset();
    CHECK(steer_front_remaining(1400, 60) == 1400);
}

static sensor_snapshot_t corridor(int32_t left, int32_t right, float yaw)
{
    sensor_snapshot_t snap = { .left_tof = left, .right_tof = right, .front_tof = -1, .yaw = yaw };
    return snap;
}

static void test_realign_off_by_default(void)
{
    params_reset();
    steer_t s = { .heading_ref = 0 };
    sensor_snapshot_t snap = corridor(85, 85, 2.0f);

    steer_realign(&s, &snap, 0.01f);
    CHECK_NEAR(s.heading_ref, 0, 1e-6);
}

static void test_realign_drifts_toward_yaw_between_walls(void)
{
    params_reset();
    params_step(P_REALIGN_RATE, +1);
    steer_t s = { .heading_ref = 0 };
    sensor_snapshot_t snap = corridor(85, 85, 2.0f);

    steer_realign(&s, &snap, 0.01f);
    CHECK(s.heading_ref > 0 && s.heading_ref < 2);
}

static void test_realign_needs_both_walls(void)
{
    params_reset();
    params_step(P_REALIGN_RATE, +1);
    steer_t s = { .heading_ref = 0 };
    sensor_snapshot_t snap = corridor(85, -1, 2.0f);

    steer_realign(&s, &snap, 0.01f);
    CHECK_NEAR(s.heading_ref, 0, 1e-6);
}

int main(void)
{
    test_front_remaining_no_reading();
    test_front_remaining_uses_tof_inside_window();
    test_front_remaining_ignores_outside_window();
    test_realign_off_by_default();
    test_realign_drifts_toward_yaw_between_walls();
    test_realign_needs_both_walls();
    return test_result();
}
