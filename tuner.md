# Tuning Guide

Practical guide for tuning micro_farr. Tune in order — each block depends on the previous one.

## Setup

1. Build and flash with ESP-IDF v6.1:

   ```bash
   source ~/.espressif/v6.1/esp-idf/export.sh
   idf.py build
   idf.py -p /dev/ttyUSB0 flash monitor
   ```

2. Enable **one** `BENCH_*` test in `main/app_config.h` at a time, then rebuild.
3. Test on a stand first, then on the floor in a clear area.
4. `main.c` silences logs by default; bench tests turn their own tag back on.

## Tuning order

```
encoders -> cell ticks -> IMU -> ToF -> motor limits -> straight -> yaw hold
```

## 1. Encoders

Bench: `BENCH_ENCODER_TEST`

| Param | Current | What to do |
|---|---|---|
| `ENCODER_LEFT_SIGN` | `1` | Push robot forward. If left total goes down, set `-1`. |
| `ENCODER_RIGHT_SIGN` | `1` | Same for right wheel. |
| `ENCODER_GLITCH_NS` | `1000` | Raise if ticks jitter at rest; lower if ticks are missed at speed. |

Pass: both totals increase when pushing forward.

## 2. Cell ticks

Bench: `BENCH_CELL_CAL_TEST`

| Param | Current | What to do |
|---|---|---|
| `CELL_TICKS` | `1495` | Hand-push exactly one cell, read the average, repeat 3x. |

Pass: the three averages agree within ~2%.

After a motorized run, refine with:

```
new_CELL_TICKS = old_CELL_TICKS * target_distance / measured_distance
```

## 3. IMU

Bench: `BENCH_IMU_TEST`

| Param | Current | What to do |
|---|---|---|
| `IMU_CALIB_SAMPLES` | `500` | Increase if yaw drifts at rest. Keep the robot still during boot. |
| `IMU_CALIB_DELAY_MS` | `2` | Time between calibration samples. Usually leave alone. |

Pass: yaw drift under ~1°/min with the robot still.
If drift is large, check the robot is not moving/vibrating during calibration.

## 4. ToF

Bench: `BENCH_TOF_TEST`

| Param | Current | What to do |
|---|---|---|
| `TOF_ADDR_LEFT/FRONT/RIGHT` | `0x30/0x31/0x32` | Change only if an address conflicts. |
| `TOF_MEASURE_TIMEOUT_MS` | `100` | Raise if readings time out; lower only if the loop feels slow. |

Pass: all three sensors report valid, sane distances in mm.
Note: sensors are read one at a time, so each updates at roughly 10 Hz.

## 5. Motor limits

The motor bench is currently not used; verify motors with `BENCH_STRAIGHT_TEST`.

| Param | Current | What to do |
|---|---|---|
| `MOTION_MIN_DUTY` | `15` | Raise until both wheels start reliably, then lower 1–2 points. |
| `MOTION_MAX_DUTY` | `30` | Safety cap. Raise only after straight runs are stable. |
| `MOTION_ACCEL_SLEW` | `0.5` | Duty added per loop (200 Hz → ~100%/s). Too high = wheel slip; too low = slow start. |

Pass: both wheels move together, no stall, no current spike at start.

## 6. Straight motion

Bench: `BENCH_STRAIGHT_TEST`

| Param | Current | What to do |
|---|---|---|
| `STRAIGHT_FF_DUTY` | `0.0` | Base duty. Raise if the robot is sluggish at the start. |
| `DIST_KP` | `0.06` | Higher = faster correction. Too high = overshoot/oscillation; too low = undershoot. |
| `SEGMENT_TIMEOUT_MS` | `5000` | Time allowed per cell. Raise if slow runs fault; lower if a stuck robot runs away. |

How to read a run: the bench logs `run done: L=... R=... yaw=...`, and the OLED shows `P progress/target`.

Pass: 5 cells stop within ~2% of `5 * CELL_TICKS`, no timeout.

## 7. Yaw hold

| Param | Current | What to do |
|---|---|---|
| `YAW_KP` | `0.5` | Higher = stronger heading correction. Too high = weaving. |
| `YAW_MAX_CORR` | `8.0` | Max steering duty. Too high = sharp corrections; too low = weak correction. |
| `YAW_STEER_SIGN` | `1` | Drive straight and watch yaw. If the error grows, flip to `-1`. |

Pass: heading stays within ~±3° during a straight run.

## Quick troubleshooting

| Symptom | Likely fix |
|---|---|
| Ticks decrease when pushing forward | Flip that wheel's `ENCODER_*_SIGN` |
| Distance consistently short/long | Recalibrate `CELL_TICKS` |
| Robot stalls at start | Raise `MOTION_MIN_DUTY` |
| Overshoots the target | Lower `DIST_KP` or `MOTION_MAX_DUTY` |
| Weaves left/right | Lower `YAW_KP` or `YAW_MAX_CORR` |
| Turns harder instead of correcting | Flip `YAW_STEER_TY` | `12.0` | Base duty to get moving. Raise if the robot struggles to start. |
| `TURN_MIN_DUTY` | `12.0` | Minimum turning duty; set just above stiction. |
| `TURN_MAX_DUTY` | `25.0` | Speed cap. Lower it if turns overshoot. |
| `TURN_ACCEL_SLEW` | `0.5` | Duty added per loop. Too high = wheel slip. |
| `TURN_COAST_MS` | `120` | Predicted coast time after braking. Raise if turns overshoot, lower if they stop short. |
| `TURN_SETTLE_MS` | `250` | Wait after braking before checking the final angle. |
| `TURN_TIMEOUT_MS` | `3000` | Time allowed per 90°. Raise if slow turns fault. |

How to read a run: the bench logs `yaw before -> after` and the expected delta (90° left, -90° right with `TURN_YAW_SIGN=1`).

Pass: each turn stops within `TURN_TOL_DEG` of 90°, no timeout, no back-and-forth buzzing.

## Quick troubleshooting

| Symptom | Likely fix |
|---|---|
| Ticks decrease when pushing forward | Flip that wheel's `ENCODER_*_SIGN` |
| Distance consistently short/long | Recalibrate `CELL_TICKS` |
| Robot stalls at start | Raise `MOTION_MIN_DUTY` |
| Overshoots the target | Lower `DIST_KP` or `MOTION_MAX_DUTY` |
| Weaves left/right | Lower `YAW_KP` or `YAW_MAX_CORR` |
| Turns harder instead of correcting | Flip `YAW_STEER_SIGN` |
| Turn goes the wrong way | Flip `TURN_YAW_SIGN` |
| Turn overshoots or buzzes | Raise `TURN_COAST_MS`, lower `TURN_KP` / `TURN_MAX_DUTY` |
| Turn stops short | Lower `TURN_COAST_MS` |
| Turn won't start | Raise `TURN_MIN_DUTY` / `TURN_FF_DUTY` |
| Turn timeout | Raise `TURN_TIMEOUT_MS`, check `TURN_YAW_SIGN` and IMU |
| Segment timeout | Raise `SEGMENT_TIMEOUT_MS` or check encoders |
| Yaw drifts at rest | Increase `IMU_CALIB_SAMPLES`, keep robot still |
| ToF readings invalid | Check address, XSHUT wiring, raise timeout |

## Tuning log

Copy this for each test:

```
Date:
Battery:
Surface:
CELL_TICKS:
DIST_KP:        YAW_KP:
MOTION_MIN/MAX: MOTION_ACCEL_SLEW:
Result (ticks / yaw / overshoot):
Notes:
```
