# Maze-Solving Robot — Plan

**Target:** ESP32-S3 (ESP-IDF + FreeRTOS), project `freertos_lab`
**Maze:** 16x16, center 2x2 goal

## Hardware

- ESP32-S3-Zero, TB6612FNG, 2 DC motors, 2 quadrature encoders
- 3x VL53L0X ToF (front/left/right), SSD1306 128x64 OLED
- Pins: user-defined in `config/pins.h`; no hard-coded pins

## RTOS Design

### How the tasks were chosen

1. List the jobs: motor update, sensor read, maze decision, display/log.
2. Give each blocking resource one owner task. Never share a driver between tasks.
3. Set priority by **deadline urgency**, not by how "important" the job feels.
4. Make each period a multiple of the 10 ms tick.
5. Connect tasks only with queues or notifications.

### Task set

| Task | Prio | Period | Owns | Why this priority |
|------|------|--------|------|-------------------|
| `motor_task` | 10 | 10 ms | motors, encoders | Hard deadline. A late PID/PWM update causes speed ripple and unstable control. Must preempt everything. |
| `sensor_task` | 6 | 20 ms | ToF / I2C | Produces the data maze consumes. Runs before maze so each cycle gets a fresh frame. Short I2C reads keep it cheap. |
| `maze_task` | 5 | 20 ms | grid, pose | Decision work is soft real-time; 20 ms late changes path quality, not stability. Consumes sensor output. |
| `ui_task` | 1 | 200 ms | UART, OLED / I2C | Diagnostics only. OLED transfer is slow (~25 ms); must never delay control or sensing. |

Priority rule: shorter deadline = higher priority. Motor (10 ms) > sensor/maze (20 ms) > UI (200 ms). Sensor sits above maze because it is the producer; maze must not run on stale data. Priority is not "importance": maze is the brain, but it can wait 20 ms while the motor cannot.

### Data flow

```
sensor_task --wall_queue(depth 1, overwrite)--> maze_task
maze_task --motion_queue(depth 4)--> motor_task
motor_task --telemetry_queue(depth 1, overwrite)--> ui_task
stop event: task notification / event group -> motor_task
```

### Rules and timing

- `vTaskDelayUntil` for every periodic task; no `vTaskDelay` for loops.
- One owner per peripheral; only `motor_task` touches motors/encoders, only `sensor_task` touches ToF, only `ui_task` touches the OLED.
- No heap allocation after init; no mutex around queue operations.
- Motor WCET < 10 ms; sensor + maze < 20 ms; UI fits in its 200 ms slot.
- Verify jitter with `esp_timer_get_time()` and stacks with `uxTaskGetStackHighWaterMark()`.

### Maze convention

Origin bottom-left, heading `0=N,1=E,2=S,3=W`, wall bits `N=1,E=2,S=4,W=8`, goal cells `(7,7),(7,8),(8,7),(8,8)`.

## Phases

0. **Environment + project** — export ESP-IDF, `idf.py --version`, target `esp32s3`, tick 100 Hz, `idf.py build/flash`.
1. **FreeRTOS fundamentals** — two periodic tasks, priorities, `vTaskDelayUntil`, execution time. Output: task framework.
2. **Motor control** — PWM, direction, encoders, PID. Output: straight driving + controlled turns.
3. **Wall detection** — read/filter ToF, publish via queue. Output: stable wall frame.
4. **Coordination** — sensor/maze/motor/ui tasks with queues and priorities. Output: responsive architecture.
5. **Maze solver** — grid, pose, flood fill, explore and solve. Output: integrated solver.
6. **Integration** — combine, test faults, tune, real maze. Output: final buildable project.

## Progress

- [ ] Phase 0 — Environment + project
- [ ] Phase 1 — FreeRTOS fundamentals
- [ ] Phase 2 — Motor control
- [ ] Phase 3 — Wall detection
- [ ] Phase 4 — Coordination
- [ ] Phase 5 — Maze solver
- [ ] Phase 6 — Integration

## Open Values

Motor constants (wheel diameter, CPR, gear ratio), start cell/heading, final tuning. Pins are learner-owned.
