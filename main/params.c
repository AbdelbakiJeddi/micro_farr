#include "params.h"

#include <math.h>
#include <stdio.h>

#include "app_config.h"

#define PARAM(id_, label_, name_, step_, min_, max_, dec_) \
    [id_] = { label_, #name_, (float)(name_), step_, min_, max_, dec_ }

static const param_info_t s_info[P_COUNT] = {
    PARAM(P_TICKS_PER_CELL,     "TICKS/C",  TICKS_PER_CELL,     10.0f,   1000.0f, 2000.0f,  0),
    PARAM(P_FRONT_ALIGN_MM,     "F_ALIGN",  FRONT_ALIGN_MM,     1.0f,    0.0f,    200.0f,   0),
    PARAM(P_FRONT_ALIGN_WIN_MM, "F_WIN",    FRONT_ALIGN_WIN_MM, 5.0f,    0.0f,    250.0f,   0),
    PARAM(P_YAW_KP,             "YAW_KP",   YAW_KP,             0.5f,    0.0f,    100.0f,   1),
    PARAM(P_TOF_CENTER_KP,      "TOF_KP",   TOF_CENTER_KP,      0.1f,    0.0f,    20.0f,    1),
    PARAM(P_YAW_MAX_RATE_DPS,   "YAW_MAX",  YAW_MAX_RATE_DPS,   5.0f,    5.0f,    720.0f,   0),
    PARAM(P_REALIGN_RATE,       "REALIGN",  REALIGN_RATE,       0.1f,    0.0f,    5.0f,     1),
    PARAM(P_RUN_KFF,            "RUN_KFF",  RUN_KFF,            0.001f,  0.0f,    0.2f,     3),
    PARAM(P_RUN_KP,             "RUN_KP",   RUN_KP,             0.001f,  0.0f,    0.2f,     3),
    PARAM(P_RUN_KI,             "RUN_KI",   RUN_KI,             0.005f,  0.0f,    1.0f,     3),
    PARAM(P_RUN_MAX_DUTY,       "MAX_DUTY", RUN_MAX_DUTY,       1.0f,    0.0f,    100.0f,   0),
    PARAM(P_EXP_CRUISE,         "E_CRUISE", EXP_CRUISE_MM_S,    10.0f,   10.0f,   3000.0f,  0),
    PARAM(P_EXP_ACCEL,          "E_ACCEL",  EXP_ACCEL_MM_S2,    100.0f,  100.0f,  20000.0f, 0),
    PARAM(P_EXP_DECEL,          "E_DECEL",  EXP_DECEL_MM_S2,    100.0f,  100.0f,  20000.0f, 0),
    PARAM(P_FAST_CRUISE,        "F_CRUISE", FAST_CRUISE_MM_S,   10.0f,   10.0f,   3000.0f,  0),
    PARAM(P_FAST_ACCEL,         "F_ACCEL",  FAST_ACCEL_MM_S2,   100.0f,  100.0f,  20000.0f, 0),
    PARAM(P_FAST_DECEL,         "F_DECEL",  FAST_DECEL_MM_S2,   100.0f,  100.0f,  20000.0f, 0),
    PARAM(P_TURN_KP,            "TURN_KP",  TURN_KP,            0.05f,   0.0f,    10.0f,    2),
    PARAM(P_TURN_FF_DUTY,       "TURN_FF",  TURN_FF_DUTY,       0.5f,    0.0f,    100.0f,   1),
    PARAM(P_TURN_MAX_DUTY,      "TURN_MAX", TURN_MAX_DUTY,      1.0f,    0.0f,    100.0f,   0),
    PARAM(P_TURN_SLEW,          "SLEW",     TURN_ACCEL_SLEW,    0.05f,   0.0f,    10.0f,    2),
    PARAM(P_TURN_SETTLE_MS,     "SETTL_MS", TURN_SETTLE_MS,     10.0f,   0.0f,    1000.0f,  0),
};

static float s_value[P_COUNT];

void params_reset(void)
{
    for (int i = 0; i < P_COUNT; i++) {
        s_value[i] = s_info[i].def;
    }
}

float params_get(param_id_t id)
{
    return s_value[id];
}

/* One step up or down, snapped to the step grid so repeated steps never drift. */
void params_step(param_id_t id, int dir)
{
    const param_info_t *p = &s_info[id];
    float v = roundf((s_value[id] + (float)dir * p->step) / p->step) * p->step;

    s_value[id] = fminf(fmaxf(v, p->min), p->max);
}

const param_info_t *params_info(param_id_t id)
{
    return &s_info[id];
}

int params_format_value(param_id_t id, char *buf, size_t len)
{
    return snprintf(buf, len, "%.*f", (int)s_info[id].decimals, s_value[id]);
}

/* Integers without suffix, floats with a decimal point and 'f': valid C either way. */
int params_format_define(param_id_t id, char *buf, size_t len)
{
    const param_info_t *p = &s_info[id];

    if (p->decimals == 0) {
        return snprintf(buf, len, "#define %s %.0f", p->define, s_value[id]);
    }
    return snprintf(buf, len, "#define %s %.*ff", p->define, (int)p->decimals, s_value[id]);
}

void params_dump(void)
{
    char line[64];

    printf("\n--- TUNED PARAMS ---\n");
    for (int i = 0; i < P_COUNT; i++) {
        params_format_define((param_id_t)i, line, sizeof line);
        printf("%s\n", line);
    }
    printf("--------------------\n");
    fflush(stdout);
}

float params_ticks_per_mm(void)
{
    return s_value[P_TICKS_PER_CELL] / CELL_MM;
}
