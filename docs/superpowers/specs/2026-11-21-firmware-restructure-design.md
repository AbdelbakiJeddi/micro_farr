# micro_farr firmware restructure — design

Date: 2026-11-21
Status: draft, awaiting approval

## 1. Intent

micro_farr is a competition micromouse (ESP32-S3, 3 ToF: left/front/right, MPU6050 gyro,
quadrature encoders, OLED 128x64, 4 buttons). The firmware grew bench by bench: a compile-time
switch in `bench.c` picks a mode, the tuner and telemetry both draw on the OLED, navigation and
tuner both read the buttons, motion commands carry no data (cell count leaks through a static),
and straights and turns use two unrelated control paths.

Goal: one firmware with a top-level robot FSM that holds two programs — the micromouse program
(explore to goal, then speed run on the memorised shortest path) and a tuner program (tune each
capability separately) — built on one consistent control cascade, with clean task boundaries.

Success: at the competition, without reflashing, the operator can explore, carry the robot back,
and speed-run from the saved map (even after a power cycle); at the bench, every capability is
tuned on its own page with its result shown on the OLED.

## 2. Goals

- Top-level robot FSM: MENU, EXPLORE, SPEED_RUN, TUNER, FAULT.
- EXPLORE: start cell → goal cell, mapping walls, continuous running through straight cells
  (current decide-ahead + extend behaviour kept). Stops at the goal, no return trip.
- On reaching the goal the map is saved to flash automatically, overwriting the previous one.
  A failed exploration saves nothing (previous good map stays).
- SPEED_RUN: always loads the saved map; plans the shortest path through **visited cells only**
  (unknown walls treated as closed); drives it with straights + 90°/180° pivots. "No maze" on the
  OLED if nothing is saved.
- Two speed profiles: EXPLORE and FAST, each a trapezoid (cruise, accel, decel). No per-distance
  speed table.
- One control cascade for all moves (choice A, below). Turns are an angle profile through the
  same wheel speed loops as straights.
- New corrections: heading re-alignment from side walls; front-wall distance alignment.
- Tuner pages in the operator's order: DISTANCE → CENTERING → SPEED → TURNS → DUMP. Each test's
  result is shown on the OLED. INC on `[RUN]` starts, DEC stops.
- One parameter table: every tunable gain with name, step, min, max, decimals. Tuner pages and
  DUMP both read it. DUMP prints valid `#define` lines.

## 3. Non-goals

- Diagonal runs, smooth (curved) turns.
- Exploring back to the start / second exploration pass.
- Saving tuning values to flash. Gains live in RAM, seeded from `app_config.h`; the operator
  copies DUMP output into `app_config.h` and reflashes. `app_config.h` stays the single source of
  truth.
- Per-distance speed tables.
- Standalone bench tests (`bench.c` and all `BENCH_*` flags are removed).
- A separate UI/telemetry task.

## 4. Anti-goals

- More moving parts than today for the same behaviour. One knob per real problem.
- Breaking the working exploration logic (decide-ahead at `NAV_DECIDE_FRAC`, run extension,
  re-plan when a wall appears after a turn).
- Losing current tuned straight/speed values: today's `#define` values become the defaults.

## 5. Constraints

- ESP-IDF v6.1, ESP32-S3-WROOM-1-N16R8. `idf.py build` must pass after each step.
- AGENTS.md: drivers → tasks → app layering; one responsibility per module; small functions;
  structs + pointers over switch/branching; minimal public headers, internals `static`;
  configuration constants in `app_config.h`; tasks communicate only through `app_ipc`.
- Buttons: BOOT GPIO0, SEL GPIO21, INC GPIO47, DEC GPIO48.
- Default partition table already has an `nvs` partition. `idf.py flash` does not erase NVS, so
  the saved maze survives reflashing.

## 6. Approaches considered

### 6.1 Control structure

- **A. Unified cascade (chosen).** Every move produces a body reference (v, ω) from a motion
  profile; corrections add to it; both wheels follow their speed references through one shared
  FF+PI wheel loop. Turns become an angle trapezoid (max spin rate, spin accel, spin decel).
  Removes `TURN_MIN_DUTY`, `TURN_FF_DUTY`, `TURN_KP`, `TURN_ACCEL_SLEW`, `TURN_MAX_DUTY`.
  Turns become independent of battery voltage. Cost: turns must be retuned; they depend on the
  wheel loop being tuned first (matches the operator's tuning order).
- B. Keep duty-based pivot turns, fix only timing/re-align/front-wall. Less retuning but two
  control styles remain and turns drift with battery and floor. Rejected by the operator.

### 6.2 Task split

- **Option 1 (chosen): three tasks** — sensor, motion, app. The app task runs the robot FSM and
  owns OLED + buttons; only one FSM state is active at a time, so the active state draws its own
  screen. Fewest IPC links, no OLED/button sharing.
- Option 2: four tasks with a separate UI task. Rejected: two extra IPC links and every FSM state
  split across two tasks, for a screen nobody reads while the robot drives.
- Single super-loop: rejected, loses the fixed-rate motion loop on core 1.

## 7. Architecture

### 7.1 Tasks

| Task | Core | Prio | Rate | Owns | Responsibility |
|---|---|---|---|---|---|
| sensor | 1 | 9 | 100 Hz | ToF, IMU, encoders | read sensors, integrate yaw, publish snapshot, signal "new sample" |
| motion | 1 | 10 | once per new snapshot (100 Hz) | motors | execute one move at a time through the cascade, publish status/result |
| app | 0 | 3 | state-driven, ~20 Hz UI poll | OLED, buttons, maze, path | robot FSM: menu, explore, speed run, tuner, fault |

The motion task blocks on the "new sample" signal instead of running its own 200 Hz timer, so
every control step sees fresh data exactly once (fixes the double-integration on repeated
snapshots). Sensor-wait timeout → motion faults the current move.

`telemetry_task`, `navigation_task`, `tuning_task`, `bench.c` are removed; their useful logic
moves into app states.

### 7.2 Shared information (all through `app_ipc`)

| Channel | Type | Writer → reader(s) | Content |
|---|---|---|---|
| sensor snapshot | mailbox (overwrite/peek) | sensor → motion, app | ToF L/F/R mm (−1 = invalid), yaw, yaw rate, ticks L/R, timestamp |
| sensor-new signal | event bit / notification | sensor → motion | one per published snapshot |
| motion command | queue (len 4) | app → motion | `motion_cmd_t` struct, below |
| motion status | mailbox | motion → app | state, progress, target, last result, below |
| events | event group | motion → app | `MOVE_DONE`, `MOVE_FAULT`, `CALIBRATED` |
| parameters | `params` module (RAM table) | app writes, motion reads | see 7.5 — read-mostly floats, 32-bit aligned writes are atomic on Xtensa |

`params` is the one sanctioned exception to "IPC only": it is a configuration store, not a task
channel. Its API is `params_get(id)` / `params_step(id, ±1)`; nobody touches globals directly.

```c
typedef enum { MOVE_STRAIGHT, MOVE_EXTEND, MOVE_TURN, MOVE_STOP } move_type_t;
typedef enum { PROFILE_EXPLORE, PROFILE_FAST } speed_profile_t;

typedef struct {
    move_type_t     type;
    int16_t         cells;      /* MOVE_STRAIGHT: cells to drive; MOVE_EXTEND: cells to add */
    int16_t         angle_deg;  /* MOVE_TURN: +90 left, -90 right, 180 */
    speed_profile_t profile;
} motion_cmd_t;

typedef enum { RESULT_NONE, RESULT_DONE, RESULT_TIMEOUT, RESULT_WALL_STOP,
               RESULT_SENSOR_LOST, RESULT_STOPPED } move_result_t;

typedef struct {
    move_result_t result;
    int32_t  distance_ticks;    /* straight: ticks covered */
    float    heading_err_deg;   /* final heading vs reference */
    float    lat_err_end_mm;    /* straight: last lateral error (0 if no walls) */
    float    lat_err_max_mm;    /* straight: worst |lateral error| */
    float    peak_speed_mm_s;
    uint32_t duration_ms;
} move_report_t;

typedef struct {
    motion_state_t state;       /* IDLE, RUNNING, DONE, FAULT */
    int32_t        progress_ticks;
    int32_t        target_ticks;
    move_report_t  last;        /* filled when a move ends */
} motion_status_t;
```

The command carries its own data — no static set from another task. `robot_state_t` in
`app_types.h` is replaced by the FSM's own state enum in the app layer. The `note[]` string in
status is dropped (the app formats text from `result`).

A thin client API (`motion_client.{h,c}` in the app layer) wraps the queue:
`motion_straight(cells, profile)`, `motion_extend(cells)`, `motion_turn(deg, profile)`,
`motion_stop()`, `motion_wait(timeout, &report)`. It clears the done/fault events before sending.

### 7.3 Control cascade (motion task)

```
           STRAIGHT                                   TURN
distance trapezoid on remaining ticks      angle trapezoid on remaining angle
   remaining = min(encoder, front-wall)       → ω_ref ; v_ref = 0
   → v_ref ; ω_ref = steering
                 │                                        │
steering (straight only):                                 │
   ω_ref = clamp(YAW_KP·heading_err + TOF_KP·lat_err, ±YAW_MAX_RATE)
   heading re-align: while both walls seen, heading_ref drifts toward yaw at REALIGN_RATE
                 └──────────────┬─────────────────────────┘
                     body (v_ref, ω_ref)
                     v_L = v_ref − ω_ref·B/2 ,  v_R = v_ref + ω_ref·B/2   (ticks/s)
                                │
              wheel loop L  ─── shared FF + PI ───  wheel loop R
              duty = KFF·v_wheel_ref + KP·err + I   (I clamped, anti-windup when duty saturates)
                                │
                             motors
```

- **Trapezoid** is one function used twice (distance in ticks, angle in degrees):
  `v = min(v_prev + a·dt, v_max, sqrt(2·d·remaining))`, floored at a creep value so the move
  always finishes. Profile parameters come from the selected speed profile (straight) or the
  turn parameters (turn).
- **Straight end:** progress ≥ target. **Turn end:** |angle error| ≤ `TURN_TOL_DEG`, then brake
  and wait `TURN_SETTLE_MS`; residual error is not re-driven (heading reference absorbs it).
- **Heading reference** is carried across moves (absolute, so turn errors don't accumulate),
  re-captured after `HEADING_REF_TIMEOUT_MS` idle, as today.
- **Heading re-align (new):** when both side walls are present, `heading_ref += REALIGN_RATE ·
  (yaw − heading_ref) · dt`. Under centering the robot's equilibrium heading is the corridor's
  direction, so this slowly removes gyro drift and the steady lateral offset (~5 mm/deg today).
  `REALIGN_RATE = 0` disables it.
- **Front-wall distance (new):** when the front ToF sees a wall within `FRONT_ALIGN_WINDOW_MM`
  of where the encoder says the target is, the remaining distance used by the profile becomes
  `front_tof − FRONT_ALIGN_MM`. The robot stops a fixed distance from the wall, re-zeroing
  accumulated encoder error. Readings outside the window are ignored (bad reads can't cause an
  early stop). The existing `FRONT_STOP_MM` emergency stop stays.
- **Removed:** `YAW_RATE_KP` gyro correction (one knob per problem), all `TURN_*` duty params,
  `MOTION_MIN_DUTY/MAX_DUTY` if unused, `TURN_MIN_DUTY` clamp.
- **Wheel loop is the only thing that writes motor duty** (plus brake at move end/stop).

Module split inside `tasks/motion/`:
- `motion_task.c` — task loop, command dispatch, move lifecycle, status/report.
- `motion_profile.c` — the trapezoid.
- `wheel_ctrl.c` — speed estimate + FF/PI per wheel.
- `steering.c` — heading hold, ToF centering, re-align, front-wall remaining distance.

### 7.4 Robot FSM (app task)

Each state is a struct entry `{ name, run }` where `run(ctx)` executes the state (blocking,
polling buttons/status as needed) and returns the next state. The FSM loop is
`state = states[state].run(&ctx)`. No switch over states.

```
BOOT ─► (init, IMU calibrate) ─► MENU
MENU ── EXPLORE ──► arm ─► explore ─► goal ─► save map ─► MENU
     ── SPEED RUN ─► load map ─► (none → "No maze") ─► plan ─► arm ─► run ─► MENU
     ── TUNER ─────► tuner pages (BOOT cycles pages; BOOT on DUMP) ─► MENU
any state: move fault / no path / sensor lost ─► FAULT (motors stopped, reason on OLED,
           any button) ─► MENU. RAM map kept, flash map untouched.
```

- **Arm:** "Press INC to start" → release → `START_DELAY_MS` so the hand clears → go.
  DEC during a run = stop → FAULT("STOPPED") → MENU.
- **EXPLORE** starts from a fresh RAM map, robot placed at the start cell facing
  `MAZE_START_DIR`. It runs the current navigation logic (record walls, flood fill, face, run with decide-ahead
  and extend, re-plan if wall after turn), plus marking each cell **visited**. Uses
  `PROFILE_EXPLORE`. On goal: `maze_store_save()`.
- **SPEED_RUN:** robot placed at the start cell facing `MAZE_START_DIR`; `maze_store_load()` → `path_plan()` → move list → execute with `PROFILE_FAST`,
  each straight a single command of N cells, each turn a pivot (robot stops between moves).
- **Screens during runs:** one status line (state, cell x,y, last result), redrawn ~10 Hz.

Button roles (my call, consistent across all screens):

| Button | Menu / lists | On `[RUN]` row | During a run |
|---|---|---|---|
| SEL | next row | next row | — |
| INC | enter / increment | start test | — |
| DEC | decrement | stop test | stop (emergency) |
| BOOT | next page / back to menu from page DUMP | — | — |

### 7.5 Parameters (`params` module, RAM only)

One table; tuner pages and DUMP are generated from it.

```c
typedef struct {
    const char *label;     /* OLED, ≤ 8 chars */
    const char *define;    /* DUMP name, e.g. "RUN_KFF" */
    float       def, step, min, max;
    uint8_t     decimals;  /* 0 → printed as integer, else "%.Nff" */
    float       value;
} param_t;
```

DUMP prints `#define NAME value` with valid C: integers without suffix (`1400`), floats with
decimal point and `f` (`0.012f`). Values are clamped to [min, max] and snapped to the step grid
on every change.

### 7.6 Tuner pages

Each page: header, parameter rows, `CELLS` (where relevant), `[RUN]` row(s), and a result line
filled from `move_report_t` after a test. 7 visible rows, scrolling beyond that (as today).

| Page | Parameters | Test | Result shown |
|---|---|---|---|
| DISTANCE | `TICKS_PER_CELL`, `FRONT_ALIGN_MM`, `FRONT_ALIGN_WINDOW_MM`, CELLS | straight N cells, EXPLORE profile | ticks covered / target, end reason |
| CENTERING | `YAW_KP`, `TOF_CENTER_KP`, `YAW_MAX_RATE_DPS`, `REALIGN_RATE`, CELLS | straight N cells | lateral err end / max, heading err |
| SPEED | `RUN_KFF`, `RUN_KP`, `RUN_KI`, `RUN_MAX_DUTY`, PROFILE (EXP/FAST), cruise/accel/decel of selected profile, CELLS | straight N cells, selected profile | distance, peak speed, time |
| TURNS | `TURN_MAX_RATE_DPS`, `TURN_ACCEL_DPS2`, `TURN_DECEL_DPS2`, `TURN_SETTLE_MS` | `[RUN R]`, `[RUN L]`, `[RUN 180]` | heading error, time |
| DUMP | — | `[DUMP]` prints all params over UART | "See UART" |

`TICKS_PER_CELL` replaces the fixed `CELL_TICKS` everywhere (motion targets, nav decide point).

### 7.7 Maze, store, path

- `maze.{h,c}` (existing) gains a `visited` bit per cell.
- `maze_store.{h,c}`: NVS blob `{ magic, version, w, h, goal_x, goal_y, walls[], visited[] }`.
  Load rejects a blob whose magic/version/size/goal don't match `app_config.h`
  (→ "No maze"). Save overwrites.
- `path.{h,c}`: BFS from start to goal over visited cells with unknown walls treated as closed;
  compress the cell path into `{STRAIGHT n | TURN ±90 | TURN 180}` moves.

### 7.8 File layout

```
main/
  main.c                 init drivers + IPC, start the 3 tasks
  app_config.h  pins.h
  app_types.h            shared IPC types (7.2)
  app_ipc.{h,c}
  params.{h,c}
  drivers/               unchanged: button, encoder, i2c_bus, imu, motor, oled, tof
  tasks/
    sensor_task.{h,c}
    motion/ motion_task.{h,c} motion_profile.{h,c} wheel_ctrl.{h,c} steering.{h,c}
  app/
    app_task.{h,c}       FSM loop + state table
    ui.{h,c}             button edges, row list / scroll helpers, text formatting
    motion_client.{h,c}
    state_menu.c state_explore.c state_speed_run.c state_tuner.c state_fault.c
    maze.{h,c} maze_store.{h,c} path.{h,c}
```

Deleted: `bench.{h,c}`, `tuning_task.*`, `tuning_params.*`, `tasks/telemetry_task.*`,
`tasks/navigation_task.*`, all `BENCH_*` in `app_config.h`.

## 8. Migration / tuning impact

- Straight and speed gains keep their current values as defaults.
- Turn tuning starts over (new parameters). Suggested defaults: max rate 360 °/s,
  accel/decel 3000 °/s² — to be tuned on the robot.
- First session order: DISTANCE (at EXPLORE speed) → CENTERING → SPEED (wheel loop, then
  profiles) → TURNS.

## 9. Left to the builder

- Exact OLED text layouts and abbreviations.
- Signal mechanism for "new sample" (event bit vs task notification).
- NVS namespace/key names.
- Default values for new parameters (`REALIGN_RATE`, `FRONT_ALIGN_*`, turn profile) — pick
  conservative ones, mark them "tune me".
- Order of the implementation steps, as long as the build passes after each.

## 10. Calls made by Claude (operator to check)

- `params` as the one non-IPC shared module (config store).
- Motion runs once per new sensor sample (100 Hz) instead of a free 200 Hz timer.
- Dropping `YAW_RATE_KP`.
- Heading re-align and front-wall distance alignment added as part of choice A.
- 180° turn as one angle-profile move (not two 90°s).
- Speed run stops between moves (pivots need a standstill).
- Button roles table in 7.4.
- Failed exploration saves nothing; FAULT keeps the RAM map.
- Goal is the single cell `MAZE_GOAL_X/Y` from `app_config.h` (a 2x2 centre goal can be added
  later by changing `at_goal()`).
