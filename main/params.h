#pragma once

/* RAM parameter table: every tunable value with its limits. Seeded from app_config.h,
 * changed by the tuner, read by the motion task. Not saved to flash: DUMP prints valid
 * #define lines to paste back into app_config.h. */

#include <stddef.h>
#include <stdint.h>

typedef enum {
    P_TICKS_PER_CELL, P_FRONT_ALIGN_MM, P_FRONT_ALIGN_WIN_MM,
    P_YAW_KP, P_TOF_CENTER_KP, P_YAW_MAX_RATE_DPS, P_REALIGN_RATE,
    P_RUN_KFF, P_RUN_KP, P_RUN_KI, P_RUN_MAX_DUTY,
    P_EXP_CRUISE, P_EXP_ACCEL, P_EXP_DECEL,
    P_FAST_CRUISE, P_FAST_ACCEL, P_FAST_DECEL,
    P_TURN_MAX_RATE, P_TURN_ACCEL, P_TURN_DECEL,
    P_TURN_SETTLE_MS,
    P_COUNT
} param_id_t;

typedef struct {
    const char *label;    /* OLED, <= 8 chars */
    const char *define;   /* app_config.h name printed by DUMP */
    float       def, step, min, max;
    uint8_t     decimals; /* 0 -> printed as integer */
} param_info_t;

void                params_reset(void);
float               params_get(param_id_t id);
void                params_step(param_id_t id, int dir);
const param_info_t *params_info(param_id_t id);
int                 params_format_value(param_id_t id, char *buf, size_t len);
int                 params_format_define(param_id_t id, char *buf, size_t len);
void                params_dump(void);

/* Encoder ticks per mm from the tuned TICKS_PER_CELL and the fixed CELL_MM. */
float               params_ticks_per_mm(void);
