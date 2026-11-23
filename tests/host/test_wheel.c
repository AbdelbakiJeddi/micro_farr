// SRCS: main/tasks/motion/wheel_ctrl.c main/params.c
#include "params.h"
#include "test.h"
#include "wheel_ctrl.h"

/* Wheel standing still (no ticks), large positive reference: output saturates high and
 * the error pushes further, so the integrator must not wind up. */
static void test_integrator_frozen_when_saturated(void)
{
    wheel_ctrl_t w;

    params_reset();
    wheel_reset(&w, 0, 0, 1.0f);
    float i0 = w.i_term;
    for (int t = 10; t <= 500; t += 10) {
        wheel_duty(&w, 5000.0f, 0, (uint32_t)t, 0.01f);
    }
    CHECK_NEAR(w.i_term, i0, 1e-6);
}

/* Small reference, output well inside the limit: the integrator accumulates the error. */
static void test_integrator_runs_when_not_saturated(void)
{
    wheel_ctrl_t w;

    params_reset();
    wheel_reset(&w, 0, 0, 1.0f);
    float i0 = w.i_term;
    wheel_duty(&w, 100.0f, 0, 10, 0.01f);
    CHECK(w.i_term > i0);
}

/* Pivot turns drive one wheel backward: its friction pre-load must point backward too. */
static void test_preload_follows_direction(void)
{
    wheel_ctrl_t fwd, back;

    params_reset();
    wheel_reset(&fwd, 0, 0, 1.0f);
    wheel_reset(&back, 0, 0, -1.0f);
    CHECK(fwd.i_term > 0.0f);
    CHECK_NEAR(back.i_term, -fwd.i_term, 1e-6);
}

int main(void)
{
    test_preload_follows_direction();
    test_integrator_frozen_when_saturated();
    test_integrator_runs_when_not_saturated();
    return test_result();
}
