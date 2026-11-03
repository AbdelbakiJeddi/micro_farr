# Build Plan — ESP32-S3 + FreeRTOS (Micromouse)

Hardware and pin map: see `hardware.md`.

## Scope
- Cell-to-cell maze navigation with **flood fill**.
- **No speed run**, **no diagonal moves** at this stage.
- Distance and cell size are measured in **encoder ticks** (no mm conversion).
- Maze size, start pose, and goal are runtime-configurable.

## FreeRTOS Tasks
| Task | Priority | Role |
|------|----------|------|
| motion_control | Highest | Wheel-tick distance PID, yaw PID, ToF centering; 100–200 Hz via `vTaskDelayUntil` |
| sensor | High | PCNT tick deltas, MPU6050 yaw, 3x VL53L0X ranges → overwrite queue |
| navigation | Medium | Update maze walls, run flood fill, issue next move |
| supervisor | Medium | State machine, fault handling, task watchdog |
| telemetry | Low | OLED + UART status |

- ESP32-S3 is dual-core: motion + sensor on core 1, navigation + telemetry on core 0.
- Task watchdog subscribed on motion and navigation; feed only on healthy loop.

## IPC
- **Overwrite queue (len 1):** latest sensor snapshot `{l_tof, r_tof, f_tof, yaw, ticks}`.
- **Command queue:** navigation → motion (`STRAIGHT_CELL`, `TURN_LEFT`, `TURN_RIGHT`, `STOP`).
- **Event group:** `CALIBRATED`, `SEGMENT_DONE`, `MAZE_SOLVED`, `FAULT`.
- **Mutex:** shared maze/wall map.

## Modules
```
main/
  main.c                  # app_main: init drivers, spawn tasks
  pins.h                  # from hardware.md
  app_config.h            # maze, ticks, ToF, PID, calibration
  app_types.h
  app_ipc.{c,h}           # queue/event/mutex handles
  drivers/
    motor.{c,h}           # TB6612 (LEDC PWM + direction + STBY)
    encoder.{c,h}         # PCNT quadrature, get_delta()/reset()
    imu.{c,h}             # MPU6050 yaw (gyro + complementary filter)
    tof.{c,h}             # VL53L0X, XSHUT re-address, range read
    oled.{c,h}            # SSD1306 status
  maze.{c,h}              # grid, walls, flood fill, path
  sensor_task.{c,h}
  motion_control_task.{c,h}
  navigation_task.{c,h}
  supervisor_task.{c,h}
  telemetry_task.{c,h}
```

## Sensing
- **VL53L0X x3** default address `0x29`; boot sequence holds all XSHUT low, releases one at a time, assigns: left `0x30`, front `0x31`, right `0x32` (configurable).
- Single I2C master = sensor task at 400 kHz; publish timestamped snapshot; never block the motion loop on I2C.
- **MPU6050:** gyro-bias calibration at rest, then integrate yaw with light complementary correction.
- **PCNT:** hardware quadrature decode, one unit per wheel; expose signed tick delta per loop.

## Motion Control (tick-based)
- **Distance PID:** target = `CELL_TICKS`; error = target − accumulated PCNT delta.
- **Yaw PID:** hold heading setpoint (0° straight, ±90° turns) from MPU6050.
- **Centering:** `error = left_tof − right_tof` biases the yaw setpoint to stay mid-corridor.
- **Primitives:** `straight_one_cell()`, `turn(±90°)`, `stop()`.
- Segment completes when ticks reached and lateral error within threshold → raise `SEGMENT_DONE`.
- Motor deadband + wheel-balance compensation in `app_config.h`.

## Navigation (Flood Fill)
- Grid cells store 4 wall bits (N/E/S/W) + `known` flag.
- **Distance grid:** BFS from goal; known walls block; unknown cells assumed open during exploration.
- **EXPLORE:** move to the lowest-distance cell that still has unknown walls, until map complete or goal found.
- **SOLVE:** flood fill from goal on known map, extract shortest path (no diagonals), replay cell-to-cell.
- Recompute flood after each newly detected wall.
- Goal = configurable cell(s); start pose and heading configurable.

## Configuration (`app_config.h`)
- `MAZE_W`, `MAZE_H`, `START_X`, `START_Y`, `START_HEADING`, goal cell(s).
- `CELL_TICKS` (ticks per cell), turn target ticks.
- ToF addresses + per-sensor offsets/thresholds.
- PID gains (distance, yaw, centering), motor deadband, speed limits, loop rate.

## State Machine (supervisor)
```
BOOT -> CALIBRATE -> EXPLORE -> SOLVE -> STOP
                        \_______ FAULT ____/
```

## Faults / Safety
- Wall-loss on a side during centering.
- Front crash / wall step detection.
- Stall: ticks not advancing vs PWM command.
- Segment timeout, I2C NACK, low battery.

## Milestones
1. Build/flash + I2C scan shows all devices + OLED alive.
2. VL53L0X re-addressed via XSHUT; L/R/F ranges sane.
3. Motors both directions; PCNT tick sign correct per wheel.
4. Yaw stable at rest.
5. Straight one cell using tick PID.
6. Add yaw-hold + side-ToF centering.
7. Accurate ±90° turns.
8. Cell-to-cell traversal with wall recording.
9. Flood fill explore + solve + path replay.
