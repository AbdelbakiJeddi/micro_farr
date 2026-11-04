# Progress Notes — micro_farr

ESP32-S3 micromouse. Tracks milestones, active files, and bench tests.

## Milestones
| # | Milestone | Status |
|---|-----------|--------|
| M1 | Bring-up & I2C bus | Done |
| M2 | ToF re-addressing | Done |
| M3 | Motors & encoders | Done |
| M4 | IMU yaw stability | Done |
| M5 | Straight-cell PID | Implemented — CELL_TICKS=1495, five-cell bench armed; hardware run pending |
| M6 | Yaw hold & centering | Partial — yaw-hold correction is implemented; side-ToF centering pending |
| M7 | ±90° turns | Not started |
| M8 | Wall recording | Not started |
| M9 | Flood fill | Not started |

## Files (`main/`)
| File | Role |
|------|------|
| `CMakeLists.txt` | Sources + component deps |
| `idf_component.yml` | Managed dep `pkolt/vl53l0x` |
| `pins.h` | GPIO map |
| `app_types.h` | `robot_state_t`, `motion_cmd_t`, `sensor_snapshot_t`, `motion_status_t` |
| `app_config.h` | Bench flags + tuning params |
| `app_ipc.*` | Snapshot mailbox, motion command queue, status mailbox, event group |
| `tasks/sensor_task.*` | 100 Hz encoder ticks + yaw + non-blocking ToF |
| `tasks/motion_control_task.*` | 200 Hz tick-distance P + yaw-hold P, publishes status |
| `tasks/telemetry_task.*` | 20 Hz OLED telemetry, core 0 |
| `drivers/i2c_bus.*` | I2C master, SDA 8 / SCL 9 |
| `drivers/motor.*` | TB6612, 20 kHz LEDC |
| `drivers/encoder.*` | PCNT x4 quadrature |
| `drivers/imu.*` | MPU6050/6500/9250, gyro-Z yaw |
| `drivers/tof.*` | 3x VL53L0X, XSHUT re-address, cached readings |
| `drivers/oled.*` | SH1106-compatible 128x64 text driver, page-addressed flush |
| `bench.*` | Standalone & integrated bench tests runner |
| `main.c` | Minimal entry point & task bootstrapper |

## Bench Tests
| Test | Flag | Result |
|------|------|--------|
| Motor | `BENCH_MOTOR_TEST` | OK — 30% duty, fwd/rev both wheels |
| Encoder | `BENCH_ENCODER_TEST` | OK — PCNT delta/total both wheels |
| IMU | `BENCH_IMU_TEST` | OK — MPU6500, drift <0.1°/12 s, ±90° tracks |
| ToF | `BENCH_TOF_TEST` | OK — 3x VL53L0X re-addressed, L/F/R mm @ 10 Hz |
| Cell cal | `BENCH_CELL_CAL_TEST` | OK — hand-push one cell: L=1489 R=1502 avg=1495 |
| OLED | `BENCH_OLED_TEST` | OK — SH1106-compatible, 0x3C, full white/black/text |
| Telemetry | `BENCH_TELEMETRY_TEST` | OK — 4-line IDLE screen, yaw + ticks update live |
| Straight | `BENCH_STRAIGHT_TEST` | Armed — 5 cells (7475 ticks, ~90 cm), soft-start slew |

## M5 notes
- `CELL_TICKS` is set to the measured average, 1495.
- `BENCH_STRAIGHT_TEST` is enabled with `BENCH_STRAIGHT_CELLS=5` (~90 cm).
- Smooth acceleration slew rate `MOTION_ACCEL_SLEW=0.5` prevents torque spikes.
- Safe testing duty capped at `MOTION_MAX_DUTY=30.0f`.
- Gentle yaw heading correction `YAW_KP=0.5f`, `YAW_MAX_CORR=8.0f`.
- Straight screen: `RUN`, `P [ticks]/7475`, `Y [yaw]`, `L[ticks] R[ticks]`.
- Cell-cal screen: `CELL CAL`, `L`, `R`, `AVG`.

## Firmware integration notes
- `app_main` now initializes IPC, OLED telemetry, IMU calibration, ToF sensors, encoders,
  and motors before starting the sensor and motion tasks.
- Sensor snapshots are published at 100 Hz and motion control runs at 200 Hz on core 1;
  OLED telemetry runs at 20 Hz on core 0.
- Motion commands and status/events are routed through `app_ipc`; turn commands are
  intentionally rejected until the turn controller is implemented.
- The next hardware checkpoint is to run `BENCH_STRAIGHT_TEST`, verify five-cell
  travel and yaw correction, then tune `CELL_TICKS`, `DIST_KP`, and `YAW_KP`.
