// SRCS: main/params.c
#include <string.h>

#include "params.h"
#include "test.h"

static void test_default_value(void)
{
    params_reset();
    CHECK_NEAR(params_get(P_RUN_KFF), 0.012f, 1e-6);
}

static void test_step_stays_on_grid(void)
{
    params_reset();
    for (int i = 0; i < 37; i++) {
        params_step(P_RUN_KFF, +1);
    }
    CHECK_NEAR(params_get(P_RUN_KFF), 0.049f, 1e-6);
}

static void test_step_clamps_at_min(void)
{
    params_reset();
    for (int i = 0; i < 1000; i++) {
        params_step(P_TURN_SETTLE_MS, -1);
    }
    CHECK(params_get(P_TURN_SETTLE_MS) == params_info(P_TURN_SETTLE_MS)->min);
}

static void test_step_clamps_at_max(void)
{
    params_reset();
    for (int i = 0; i < 1000; i++) {
        params_step(P_TICKS_PER_CELL, +1);
    }
    CHECK(params_get(P_TICKS_PER_CELL) == params_info(P_TICKS_PER_CELL)->max);
}

static void test_define_lines_are_valid_c(void)
{
    char b[64];

    params_reset();
    for (int i = 0; i < 37; i++) {
        params_step(P_RUN_KFF, +1);
    }
    for (int i = 0; i < 1000; i++) {
        params_step(P_TURN_SETTLE_MS, -1);
    }

    params_format_define(P_TICKS_PER_CELL, b, sizeof b);
    CHECK(strcmp(b, "#define TICKS_PER_CELL 1400") == 0);
    params_format_define(P_RUN_KFF, b, sizeof b);
    CHECK(strcmp(b, "#define RUN_KFF 0.049f") == 0);
    params_format_define(P_TURN_SETTLE_MS, b, sizeof b);
    CHECK(strcmp(b, "#define TURN_SETTLE_MS 0") == 0);
}

static void test_labels_fit_oled(void)
{
    for (int i = 0; i < P_COUNT; i++) {
        CHECK(strlen(params_info((param_id_t)i)->label) <= 8);
    }
}

int main(void)
{
    test_default_value();
    test_step_stays_on_grid();
    test_step_clamps_at_min();
    test_step_clamps_at_max();
    test_define_lines_are_valid_c();
    test_labels_fit_oled();
    return test_result();
}
