# Robot FSM — Phase 1

**Scope:** explore the maze and stop at the target cell. No return, no speed run.

## Mission

```
BOOT -> CALIBRATE -> IDLE -> EXPLORE -> GOAL -> IDLE
any state -> STOP   (ESTOP)
any state -> FAULT  (unrecoverable) -> CALIBRATE (recovered)
```

- `BOOT`: init drivers, ToF addresses, IMU, encoders, OLED.
- `CALIBRATE`: gyro bias, encoder direction, ToF sanity.
- `IDLE`: motors off, wait for start.
- `EXPLORE`: run the navigation loop until the target cell is reached.
- `GOAL`: motors off, hold final pose, wait for reset.
- `STOP`: motors disabled immediately by button ISR.
- `FAULT`: motors off; supervisor recovery.

## Navigation (inside EXPLORE)

```
SETTLE -> READ_WALLS -> UPDATE_MAP -> CHOOSE -> PLAN -> MOVE -> ARRIVE -> SETTLE
                \-> RELOCALIZE (invalid/timeout/mismatch) -> FAULT after N retries
CHOOSE: target reached -> GOAL | no path -> FAULT
```

- `MOVE` = `TURN` then `DRIVE`.
- Pose (odometry + heading correction) is computed continuously in
  `motion_control_task` and published to `navigation_task`.
- `navigation_task` = outer loop (heading/distance). `motion_control_task` =
  inner wheel-speed PID, heading, and wall centering.
- ESTOP path: button ISR -> `motion_control_task` directly.

## Owners

| Machine | Owner |
|---|---|
| Mission + navigation FSM | `navigation_task` |
| Wheel speed, odometry, heading, pose | `motion_control_task` |
| Walls + IMU | `sensor_task` |
| Health / restart + OLED display | `supervisor_task` |
