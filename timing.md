# Task Timing, Deadlines and Blocking — Phase 1

Baseline: 4 tasks (`sensor`, `motion_control`, `navigation`, `supervisor`).
Tick: **1 kHz** (1 ms).

## 1. Timing table

| Task | Core | Period | Deadline | WCET target | Prio | Offset | Blocking |
|---|---|---|---|---|---|---|---|
| `motion_control_task` | 1 | 10 ms | 10 ms (hard) | ≤ 2 ms | 20 | 0 ms | command/IMU/ToF queues (0 timeout) |
| `sensor_task` | 0 | 10 ms | 10 ms activation; ToF ≤ 50 ms | ≤ 2 ms | 15 | 0 ms | I2C driver semaphore |
| `supervisor_task` | 0 | 20 ms | 20 ms | ≤ 0.5 ms typical; ≤ 3.5 ms on an OLED chunk | 10 | 5 ms | I2C mutex (OLED) |
| `navigation_task` | 0 | event-driven | 50 ms from event | ≤ 5 ms | 5 | — | queues / notification |

- **Offset** = initial delay before the `vTaskDelayUntil` loop.
- `motion_control` is alone on Core 1; the others share Core 0.
- `navigation` has no period: it blocks and wakes on a ToF update, a pose update,
  or a step timeout.

### Motion-control workload (expanded)

Each 10 ms activation, `motion_control_task`:

1. Reads left/right encoder counts; updates wheel speeds and odometry.
2. Consumes the latest IMU sample; estimates and corrects heading.
3. Consumes the latest left/right ToF samples; applies wall centering when a
   suitable parallel wall pair is detected.
4. Combines heading and centering corrections into left/right wheel-speed
   targets.
5. Runs the two wheel PID controllers.
6. Writes PWM and direction to the TB6612FNG.
7. Publishes pose + movement status to `navigation_task`.

### Supervisor workload

- Monitors task heartbeats, sensor timeouts, and motor faults.
- Requests emergency stops and latches the fault state.
- Drives the OLED for debug: system config and the current system state,
  refreshed at 5–10 Hz (not every 20 ms activation).
- Battery monitoring is **removed for now**.

## 2. Deadlines

Separate two meanings:

- **Activation deadline** — the scheduler guarantee. `motion_control` and
  `sensor` must start within their 10 ms period; `supervisor` within 20 ms.
- **Data-freshness deadline** — how old data may be. IMU must be ≤ 10 ms old;
  ToF ≤ 50 ms old. `navigation` must react within 50 ms of an event.

## 3. Rates and data

| Data | Rate | Producer | Consumer | Mechanism |
|---|---|---|---|---|
| IMU | 100 Hz (every 10 ms) | sensor | motion_control | latest-value queue |
| ToF left/right | 20–50 Hz | sensor | motion_control | latest-value queue |
| ToF frame (3 sensors) | 20–50 Hz | sensor | navigation | latest-value queue |
| Pose + movement status | 100 Hz | motion_control | navigation | snapshot + notification |
| Movement command | on event | navigation | motion_control | queue |
| System state (display) | 5–10 Hz | navigation/motion | supervisor | latest-value queue |
| ESTOP / fault | immediate | supervisor | motion_control | notification + latched flag |

ToF plan: start all 3 measurements, poll one per activation, read when ready.
This keeps each activation short and gives a full frame every 20–50 ms.

Because a FreeRTOS queue has one consumer, the ToF data is published to **two**
latest-value queues: `tof_motion_q` (left/right) and `tof_nav_q` (all three).
`sensor_task` overwrites both; each consumer reads only its own queue.

Centering uses the **left and right** ToF only. The front ToF is for navigation
and wall detection, not lateral centering.

## 4. Blocking behaviour

- **sensor**: owns I2C for IMU/ToF. It shares the bus with `supervisor` for the
  OLED through `i2c_mutex`. Every wait has a timeout. Publishing uses overwrite
  queues, so it never blocks on a full queue.
- **motion_control**: reads the command queue with a **0 timeout** and peeks the
  IMU and left/right ToF latest-value queues with a **0 timeout**. It must never
  block on another task. No I2C, no dynamic allocation.
- **navigation**: may block indefinitely on its input queues; that is correct for
  an event-driven task.
- **supervisor**: takes `i2c_mutex` only to refresh the OLED at 5–10 Hz, writes
  in chunks, and releases between chunks. It blocks on the mutex with a timeout
  and never touches motors.

**Shared resource:** `i2c_mutex` between `sensor_task` and `supervisor_task`. It
is a real mutex (priority inheritance) and is never held across a whole OLED
frame. If a chunk still delays the IMU too often, the fallback is a second I2C
bus for the OLED.

## 5. Priorities

Baseline order kept: `motion_control` (20) > `sensor` (15) > `supervisor` (10) >
`navigation` (5).

Why `sensor` above `motion_control` is **not** needed here: they are on
different cores. On Core 0, `sensor` > `supervisor` > `navigation` is
rate-monotonic. `motion_control` at 20 is safe from system tasks at 22 because
it runs on Core 1.

## 6. How WCET will be measured

- `esp_timer_get_time()` at task entry and exit; keep the maximum.
- `uxTaskGetStackHighWaterMark()` per task; target < 75%.
- A spare GPIO toggled around the motor loop for scope verification.
- Runtime stats enabled to check CPU share per task.

## 7. Open items

1. ToF mode (single-shot polling vs continuous) and final frame rate.
2. Wheel diameter, encoder CPR, gear ratio (needed before PID).
3. Centering: minimum parallel-wall length and correction gain (left/right ToF).
4. OLED refresh rate and chunk size; fallback to a second I2C bus if the mutex
   delays the IMU too often.
