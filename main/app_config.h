#pragma once

#define BENCH_OLED_TEST         0
#define BENCH_TELEMETRY_TEST    0

#define BENCH_MOTOR_TEST        0
#define BENCH_MOTOR_SPEED       30
#define BENCH_MOTOR_TIME_MS     2000

#define BENCH_ENCODER_TEST      0
#define BENCH_ENCODER_PERIOD_MS 100

#define BENCH_IMU_TEST          0
#define BENCH_IMU_PERIOD_MS     100

#define BENCH_TOF_TEST          0
#define BENCH_TOF_PERIOD_MS     50

#define BENCH_CELL_CAL_TEST      0
#define BENCH_CELL_CAL_PERIOD_MS 200

#define BENCH_MAZE_TEST          1   /* full maze solve; press a button to start */

#define BENCH_STRAIGHT_TEST      0
#define BENCH_STRAIGHT_CELLS     2
#define BENCH_STRAIGHT_REPEATS   1
#define BENCH_STRAIGHT_DELAY_MS  500

#define BUTTON_ACTIVE_LEVEL      0

#define ENCODER_HIGH_LIMIT      30000
#define ENCODER_LOW_LIMIT      -30000
#define ENCODER_GLITCH_NS       1000

#define I2C_PORT_NUM            0
#define I2C_SPEED_HZ            400000
#define I2C_TIMEOUT_MS          100

#define OLED_ADDR_PRIMARY       0x3C
#define OLED_ADDR_ALT           0x3D
#define OLED_WIDTH              128
#define OLED_HEIGHT             64
#define OLED_PAGES              (OLED_HEIGHT / 8)
#define OLED_COLS               (OLED_WIDTH / 6)
#define OLED_I2C_TIMEOUT_MS     1000
#define OLED_I2C_SPEED_HZ       400000
#define OLED_COL_OFFSET         2

#define IMU_ADDR_PRIMARY        0x68
#define IMU_ADDR_ALT            0x69
#define IMU_CALIB_SAMPLES       1000
#define IMU_CALIB_DELAY_MS      2
#define GYRO_LSB_PER_DPS        65.5f

#define TOF_DEFAULT_ADDR        0x29
#define TOF_ADDR_LEFT           0x30
#define TOF_ADDR_FRONT          0x31
#define TOF_ADDR_RIGHT          0x32
#define TOF_BOOT_MS             10
#define TOF_ADDR_SETTLE_MS      100
#define TOF_PROBE_MS            20
#define TOF_MEASURE_TIMEOUT_MS  100

#define SENSOR_TASK_HZ          100
#define MOTION_TASK_HZ          200
#define TELEMETRY_TASK_HZ       20
#define TELEMETRY_TASK_CORE     0
#define TELEMETRY_TASK_PRIO     2
#define TELEMETRY_TASK_STACK    4096
#define SENSOR_TASK_CORE        1
#define MOTION_TASK_CORE        1
#define SENSOR_TASK_PRIO        9
#define MOTION_TASK_PRIO        10
#define SENSOR_TASK_STACK       4096
#define MOTION_TASK_STACK       4096

#define CELL_TICKS              1400
#define ENCODER_LEFT_SIGN       1
#define ENCODER_RIGHT_SIGN      1

#define STRAIGHT_FF_DUTY        0.0f
#define DIST_KP                 0.06f
#define MOTION_MIN_DUTY         15.0f
#define MOTION_MAX_DUTY         24.0f
#define MOTION_ACCEL_SLEW       0.6f

#define YAW_KP                  0.5f
#define YAW_MAX_CORR            12.0f
#define YAW_STEER_SIGN          1

/* Straight-line steering source */
#define STEER_MODE_YAW          0   /* IMU yaw hold only */
#define STEER_MODE_TOF          1   /* side-ToF centering only */
#define STEER_MODE_FUSED        2   /* IMU heading hold + ToF centering offset */
#define STEER_MODE              STEER_MODE_FUSED
#define TOF_WALL_MAX_MM         170    /* side reading below this counts as a wall */
#define TOF_SIDE_TARGET_MM      85      /* (250 mm cell - 100 mm robot) / 2 */
#define TOF_CENTER_KP           0.10f   /* duty per mm of lateral error */
#define TOF_CENTER_MAX_CORR     14.0f

/* Fused mode: ToF error -> heading offset (deg) fed to the IMU yaw loop.
 * TOF_YAW_GAIN * YAW_KP ~= TOF_CENTER_KP, so both modes respond with similar strength. */
#define TOF_YAW_GAIN            0.2f    /* deg of heading offset per mm of lateral error */
#define TOF_YAW_MAX_OFFSET_DEG  15.0f
#define TOF_OFFSET_ALPHA        0.1f    /* low-pass per 5 ms loop (~50 ms time constant) */

#define SEGMENT_TIMEOUT_MS      5000

/* Front wall: emergency stop of a straight move when the front ToF reads below this */
#define FRONT_STOP_MM           50

/* Pivot turns (positive yaw = left / counter-clockwise) */
#define TURN_QUARTER_DEG        90.0f
#define TURN_KP                 0.5f    /* duty per degree of heading error */
#define TURN_TOL_DEG            2.0f
#define TURN_SETTLE_LOOPS       10      /* motion loops inside tolerance before "done" */
#define TURN_TIMEOUT_MS         3000
/* Heading reference carried between moves; re-captured after this idle time */
#define HEADING_REF_TIMEOUT_MS  2000

/* Maze (x grows east, y grows north; directions 0=N 1=E 2=S 3=W) */
#define MAZE_W                  3
#define MAZE_H                  3
#define MAZE_START_X            0
#define MAZE_START_Y            0
#define MAZE_START_DIR          0
#define MAZE_GOAL_X             1
#define MAZE_GOAL_Y             1

/* Navigation */
#define NAV_FRONT_WALL_MM       TOF_WALL_MAX_MM  /* square robot: same wall threshold as the sides */
#define NAV_SENSE_SETTLE_MS     100     /* wait after stopping before reading walls */
#define NAV_START_DELAY_MS      500
#define NAV_MOVE_WAIT_MS        (SEGMENT_TIMEOUT_MS + 2000)
#define NAV_MAX_STEPS           60
#define NAV_TASK_CORE           0
#define NAV_TASK_PRIO           5
#define NAV_TASK_STACK          4096
