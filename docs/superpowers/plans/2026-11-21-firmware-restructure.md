# Firmware Restructure Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restructure micro_farr into 3 tasks (sensor, motion, app) with a robot FSM (MENU / EXPLORE / SPEED_RUN / TUNER / FAULT), one control cascade for all moves, a parameter table, and a saved maze for speed runs.

**Architecture:** Sensor task publishes snapshots and signals motion; motion task runs a trapezoid profile → steering → per-wheel FF+PI cascade for straights and turns; app task owns OLED + buttons and runs the FSM as a table of state functions. Tasks talk only through `app_ipc`; gains live in a RAM `params` table.

**Tech Stack:** C, ESP-IDF v6.1, FreeRTOS, NVS; host-side unit tests with system `gcc` for pure modules.

**Spec:** `docs/superpowers/specs/2026-11-21-firmware-restructure-design.md`

## Global Constraints

- Host-testable modules (`params.c`, `motion_profile.c`, `steering.c`, `maze.c`, `path.c`, `maze_store.c` under `-DMAZE_STORE_HOST`) include no ESP-IDF/FreeRTOS headers.
- Build env: `source ~/.espressif/v6.1/esp-idf/export.sh`; every task ends with `idf.py build` → `Project build complete`.
- Target ESP32-S3-WROOM-1-N16R8. Buttons: BOOT GPIO0, SEL GPIO21, INC GPIO47, DEC GPIO48.
- Layering drivers → tasks → app; one responsibility per module; internals `static`; minimal public headers.
- Configuration constants (defaults, rates, sizes) live in `app_config.h`.
- Tasks communicate only through `app_ipc`; `params` is the single sanctioned config store.
- Prefer struct tables + function pointers over switch on state/page.
- No standalone bench tests, no NVS for gains, no diagonals, no per-distance speed tables.
- Do not over-engineer: one knob per real problem.
- Commit after each task on branch `restructure`, message ending with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

1. Sensor snapshot stops arriving mid-move → motion brakes and reports `RESULT_SENSOR_LOST` within `MOTION_SENSOR_TIMEOUT_MS` (Task 2 robot check: suspend the sensor task from a temporary tuner action, confirm brake + FAULT, then remove the action).
2. Saved maze from a different config (size or goal list changed, then reflashed) → speed run shows "No maze", never drives (Task 6 host test `test_store_rejects_mismatch` on the blob validator).
3. Single spurious short front-ToF read mid-straight → robot does not stop early (Task 5 host test `test_front_remaining_ignores_outside_window`).
4. Param stepped many times / to its limits → value stays on the step grid, inside [min,max]; DUMP lines compile (Task 1 host tests).
5. DEC pressed during any run → motors stop at once, FAULT("STOPPED"), back to MENU (Task 7 robot check, Task 8/9 reuse the same run-guard helper).

---

## File map (end state)

```
main/main.c                              init drivers + IPC, start 3 tasks
main/app_config.h  main/pins.h
main/app_types.h  main/app_ipc.{h,c}     IPC types and channels
main/params.{h,c}                        RAM parameter table
main/drivers/*                           unchanged
main/tasks/sensor_task.{h,c}
main/tasks/motion/motion_task.{h,c}      loop, dispatch, move lifecycle, report
main/tasks/motion/motion_profile.{h,c}   trapezoid
main/tasks/motion/wheel_ctrl.{h,c}       speed estimate + FF/PI
main/tasks/motion/steering.{h,c}         heading hold, centering, re-align, front-wall remaining
main/app/app_task.{h,c}                  FSM loop + state table
main/app/ui.{h,c}                        button edges, row list/scroll, result line
main/app/motion_client.{h,c}             queue wrappers
main/app/state_menu.c state_fault.c state_tuner.c state_explore.c state_speed_run.c
main/app/app_states.h                    state ids + run() prototypes (app-internal)
main/app/maze.{h,c} maze_store.{h,c} path.{h,c}
tests/host/run.sh  tests/host/test_*.c   host unit tests (gcc)
```

---

### Task 1: Host test harness + `params` table

**Files:**
- Create: `main/params.h`, `main/params.c`, `tests/host/run.sh`, `tests/host/test_params.c`, `tests/host/test.h`
- Modify: `main/tasks/motion_control_task.c` (replace `g_*` with `params_get`), `main/tuning_task.c` (page entries reference `param_id_t`; drop `dump_all` body for `params_dump`), `main/app_config.h` (add new defaults), `main/CMakeLists.txt`
- Delete: `main/tuning_params.{h,c}`

**Interfaces:**
- Produces:
```c
typedef enum {
    P_TICKS_PER_CELL, P_FRONT_ALIGN_MM, P_FRONT_ALIGN_WIN_MM,
    P_YAW_KP, P_TOF_CENTER_KP, P_YAW_MAX_RATE_DPS, P_REALIGN_RATE,
    P_RUN_KFF, P_RUN_KP, P_RUN_KI, P_RUN_MAX_DUTY,
    P_EXP_CRUISE, P_EXP_ACCEL, P_EXP_DECEL,
    P_FAST_CRUISE, P_FAST_ACCEL, P_FAST_DECEL,
    P_TURN_KP, P_TURN_FF_DUTY, P_TURN_MAX_DUTY, P_TURN_SLEW, /* removed in Task 4 */
    P_TURN_SETTLE_MS,
    P_COUNT
} param_id_t;

typedef struct {
    const char *label;   /* ≤ 8 chars */
    const char *define;
    float def, step, min, max;
    uint8_t decimals;    /* 0 → integer */
} param_info_t;

void                params_reset(void);                 /* all values = defaults */
float               params_get(param_id_t id);
void                params_step(param_id_t id, int dir); /* dir = +1/-1; clamp + snap */
const param_info_t *params_info(param_id_t id);
int                 params_format_value(param_id_t id, char *buf, size_t len);
int                 params_format_define(param_id_t id, char *buf, size_t len);
void                params_dump(void);                   /* printf all define lines */
```
- Defaults come from `app_config.h` names equal to `define` (e.g. `RUN_KFF`). New defines: `TICKS_PER_CELL 1400` (replaces `CELL_TICKS`; keep `#define CELL_TICKS TICKS_PER_CELL` until Task 3 removes users), `FRONT_ALIGN_MM 40`, `FRONT_ALIGN_WIN_MM 60`, `REALIGN_RATE 0.0f`, `EXP_CRUISE_MM_S 450.0f`, `EXP_ACCEL_MM_S2 2500.0f`, `EXP_DECEL_MM_S2 3000.0f`, `FAST_*` = same three values. `RUN_CRUISE/ACCEL/DECEL_*` removed.

- [ ] **Step 1: Write `tests/host/test.h`** — `CHECK(cond)` and `CHECK_NEAR(a,b,eps)` macros that print file:line and set a failure flag; `main()` returns nonzero on failure.
- [ ] **Step 2: Write `tests/host/run.sh`** — for each `tests/host/test_*.c`, compile with `gcc -std=c11 -Wall -Werror -I main -I main/app -I main/tasks/motion -o build_host/<name>` plus the module sources it lists in a `// SRCS:` first-line comment, then run it; exit nonzero on any failure; print `ALL HOST TESTS PASSED`.
- [ ] **Step 3: Write failing tests `tests/host/test_params.c`** (`// SRCS: main/params.c`):
```c
params_reset();
CHECK_NEAR(params_get(P_RUN_KFF), 0.012f, 1e-6);
for (int i = 0; i < 37; i++) params_step(P_RUN_KFF, +1);
CHECK_NEAR(params_get(P_RUN_KFF), 0.049f, 1e-6);          /* on grid, no drift */
for (int i = 0; i < 1000; i++) params_step(P_TURN_SETTLE_MS, -1);
CHECK(params_get(P_TURN_SETTLE_MS) == params_info(P_TURN_SETTLE_MS)->min);
char b[64];
params_format_define(P_TICKS_PER_CELL, b, sizeof b);  CHECK(strcmp(b, "#define TICKS_PER_CELL 1400") == 0);
params_format_define(P_RUN_KFF, b, sizeof b);         CHECK(strcmp(b, "#define RUN_KFF 0.049f") == 0);
params_format_define(P_TURN_SETTLE_MS, b, sizeof b);  CHECK(strcmp(b, "#define TURN_SETTLE_MS 0") == 0); /* at min 0, integer form */
for (int i = 0; i < P_COUNT; i++) CHECK(strlen(params_info(i)->label) <= 8);
```
- [ ] **Step 4: Run `tests/host/run.sh`** — expected: compile error (params.h missing).
- [ ] **Step 5: Implement `params.c`** — static `param_info_t` table indexed by `param_id_t` + static `float s_value[P_COUNT]`; `params_step`: `v = roundf((v + dir*step)/step)*step`, then clamp. Min/max per param: gains ≥ 0, settle 0..1000, ticks/cell 1000..2000, duties 0..100, speeds/accels ≥ step. `params_format_define`: decimals 0 → `"%.0f"`, else `"%.*ff"`.
- [ ] **Step 6: Run `tests/host/run.sh`** — expected `ALL HOST TESTS PASSED`.
- [ ] **Step 7: Switch callers** — motion task uses `params_get(P_...)` (EXP profile for all straights for now); tuner page entries hold `param_id_t` and call `params_step` / `params_format_value`; DUMP calls `params_dump`. Delete `tuning_params.*`; update CMake SRCS; call `params_reset()` in `app_main` before tasks start.
- [ ] **Step 8: `idf.py build`** → `Project build complete`.
- [ ] **Step 9: Commit** `refactor(params): single RAM parameter table with limits and valid DUMP`.

---

### Task 2: Commands with payload, move report, per-sample motion loop

**Files:**
- Modify: `main/CMakeLists.txt` (add `app` to INCLUDE_DIRS), `main/app_types.h`, `main/app_ipc.{h,c}`, `main/tasks/sensor_task.c`, `main/tasks/motion_control_task.{h,c}`, `main/tasks/navigation_task.c`, `main/tasks/telemetry_task.c`, `main/tuning_task.c`, `main/app_config.h`
- Create: `main/app/motion_client.{h,c}`

**Interfaces:**
- Produces (types exactly as spec §7.2: `move_type_t`, `speed_profile_t`, `motion_cmd_t`, `move_result_t`, `move_report_t`, `motion_status_t` with `last`; `note[]` and `robot_state_t` removed). Event bits renamed: `APP_EVENT_MOVE_DONE`, `APP_EVENT_MOVE_FAULT`, `APP_EVENT_CALIBRATED`; `APP_EVENT_MAZE_SOLVED` removed.
```c
/* app_ipc.h additions */
bool app_ipc_send_motion(const motion_cmd_t *cmd, TickType_t wait);
bool app_ipc_recv_motion(motion_cmd_t *cmd, TickType_t wait);
void app_ipc_signal_sensor(void);                 /* sensor task, after publish */
bool app_ipc_wait_sensor(TickType_t wait);        /* motion task only */

/* motion_client.h */
esp_err_t motion_straight(int cells, speed_profile_t profile);
esp_err_t motion_extend(int cells);
esp_err_t motion_turn(int angle_deg, speed_profile_t profile);  /* +left, -right */
esp_err_t motion_stop(void);
bool      motion_wait(TickType_t timeout, move_report_t *out);  /* true = RESULT_DONE */
bool      motion_status(motion_status_t *out);
```
- `motion_straight/turn` clear DONE|FAULT events before sending; `motion_extend` does not.
- Motion task: `app_ipc_wait_sensor(pdMS_TO_TICKS(MOTION_SENSOR_TIMEOUT_MS))` (new define `50`) replaces `vTaskDelayUntil`; on timeout while a move is active → brake, `RESULT_SENSOR_LOST`, FAULT event. `MOTION_TASK_HZ` removed; `MOTION_DT_S` computed from snapshot timestamps (clamped 1..50 ms).
- Report filled at every move end: `distance_ticks`, `heading_err_deg`, `lat_err_end_mm`, `lat_err_max_mm`, `peak_speed_mm_s`, `duration_ms`, `result`.
- `straight_cells()` etc. and the `s_target_ticks` static are deleted; target comes from `cmd.cells * params_get(P_TICKS_PER_CELL)`.

- [ ] **Step 1: Change types + IPC**, implement `app_ipc_signal_sensor/wait_sensor` with a dedicated event bit `APP_EVENT_SENSOR_NEW` (wait clears on exit).
- [ ] **Step 2: Sensor task** calls `app_ipc_signal_sensor()` after each publish.
- [ ] **Step 3: Motion task** — dispatch on `cmd.type` via a `static const` handler table indexed by `move_type_t`; build report; sensor-timeout fault.
- [ ] **Step 4: `motion_client.c`** and switch navigation, telemetry (format text from `status.last.result`), tuner (STRAIGHT `[RUN]` → `motion_straight(1, PROFILE_EXPLORE)`, TURN `[RUN]` → `motion_turn(-90, PROFILE_EXPLORE)`, SPEED `[RUN]` → `motion_straight(cells, ...)`).
- [ ] **Step 5: `idf.py build`** → `Project build complete`.
- [ ] **Step 6: Robot check (tuner build):** SPEED `[RUN]` CELLS=3, then STRAIGHT `[RUN]` → drives exactly 1 cell (stale-cells bug gone). Sensor-lost check per Review Focus 1.
- [ ] **Step 7: Commit** `refactor(ipc): motion commands carry payload; motion runs once per sensor sample`.

---

### Task 3: Split motion into profile / wheel / steering modules

**Files:**
- Create: `main/tasks/motion/motion_task.{h,c}`, `motion_profile.{h,c}`, `wheel_ctrl.{h,c}`, `steering.{h,c}`, `tests/host/test_profile.c`
- Delete: `main/tasks/motion_control_task.{h,c}`
- Modify: `main/CMakeLists.txt` (add `tasks/motion` include dir), `main/main.c`, `main/app_config.h`

**Interfaces:**
```c
/* motion_profile.h — units agnostic (ticks or degrees) */
typedef struct { float v_max, accel, decel, v_min; } profile_limits_t;
float profile_next(float v_prev, float remaining, float dt, const profile_limits_t *lim);
/* = max(min(v_prev + accel*dt, v_max, sqrt(2*decel*max(remaining,0))), v_min) */

/* wheel_ctrl.h */
typedef struct { int32_t ref_ticks; uint32_t ref_ms; float v; float i_term; } wheel_ctrl_t;
void  wheel_reset(wheel_ctrl_t *w, int32_t ticks, uint32_t now_ms);
float wheel_duty(wheel_ctrl_t *w, float v_ref, int32_t ticks, uint32_t now_ms, float dt);
/* FF+PI from params; integrator frozen when |output| ≥ P_RUN_MAX_DUTY and err pushes further (anti-windup) */

/* steering.h */
typedef struct { float heading_ref; float lat_err; } steer_t;
float steer_rate_ref(steer_t *s, const sensor_snapshot_t *snap);  /* deg/s, clamped */
float steer_wheel_half_diff(float rate_dps);                       /* ticks/s */
```
- `YAW_RATE_KP` removed (steer_wheel_half_diff takes only the reference). `CELL_TICKS` alias removed; all users read `P_TICKS_PER_CELL`. Straight behaviour otherwise unchanged.

- [ ] **Step 1: Failing tests `test_profile.c`** (`// SRCS: main/tasks/motion/motion_profile.c`):
```c
profile_limits_t L = { .v_max = 100, .accel = 1000, .decel = 1000, .v_min = 5 };
CHECK_NEAR(profile_next(0, 1000, 0.01f, &L), 10, 1e-4);     /* accel limited */
CHECK_NEAR(profile_next(100, 1000, 0.01f, &L), 100, 1e-4);  /* cruise cap */
CHECK_NEAR(profile_next(100, 2, 0.01f, &L), sqrtf(4000), 1e-3); /* brake curve */
CHECK_NEAR(profile_next(50, 0, 0.01f, &L), 5, 1e-4);        /* floor, always finishes */
CHECK_NEAR(profile_next(50, -3, 0.01f, &L), 5, 1e-4);       /* overshoot: no NaN */
```
- [ ] **Step 2: Run `tests/host/run.sh`** — FAIL (missing module).
- [ ] **Step 3: Implement the four modules** by moving existing code; `motion_task.c` keeps loop, dispatch, lifecycle, report.
- [ ] **Step 4: `tests/host/run.sh`** → PASS; **`idf.py build`** → complete.
- [ ] **Step 5: Robot check:** tuner STRAIGHT 3 cells in corridor — same behaviour as before the split.
- [ ] **Step 6: Commit** `refactor(motion): split into profile, wheel control and steering modules`.

---

### Task 4: Turns through the wheel loops (angle profile)

**Files:**
- Modify: `main/tasks/motion/motion_task.c`, `main/params.{h,c}`, `main/app_config.h`, `main/tuning_task.c`

**Interfaces:**
- Params: remove `P_TURN_KP, P_TURN_FF_DUTY, P_TURN_MAX_DUTY, P_TURN_SLEW`; add `P_TURN_MAX_RATE` (def `TURN_MAX_RATE_DPS 360`, step 10), `P_TURN_ACCEL` (`TURN_ACCEL_DPS2 3000`, step 100), `P_TURN_DECEL` (`TURN_DECEL_DPS2 3000`, step 100). Remove `TURN_MIN_DUTY`, `TURN_KP`, `TURN_FF_DUTY`, `TURN_MAX_DUTY`, `TURN_ACCEL_SLEW`, `MOTION_MIN_DUTY`, `MOTION_MAX_DUTY` from config. New `TURN_MIN_RATE_DPS 20` (profile floor).
- Turn: `ω = sign * profile_next(|ω_prev|, |angle_err|, dt, turn_limits)`, `v = 0`; wheels via `steer_wheel_half_diff(ω)` and `wheel_duty`. Done when `|err| ≤ TURN_TOL_DEG` → brake, settle `P_TURN_SETTLE_MS`, `RESULT_DONE` with `heading_err_deg`. `motion_turn(angle)` accepts ±90 and 180.

- [ ] **Step 1: Implement**; tuner TURN page shows new params and `[RUN R]` (−90), `[RUN L]` (+90), `[RUN 180]`.
- [ ] **Step 2: `idf.py build`** → complete.
- [ ] **Step 3: Robot check:** 4× `[RUN R]` and 4× `[RUN L]` each finish (no timeout), heading err printed per turn.
- [ ] **Step 4: Commit** `feat(motion): pivot turns as angle profile through wheel speed loops`.

---

### Task 5: Heading re-align + front-wall distance

**Files:**
- Modify: `main/tasks/motion/steering.{h,c}`, `main/tasks/motion/motion_task.c`
- Create: `tests/host/test_steering.c`

**Interfaces:**
```c
/* steering.h additions */
void    steer_realign(steer_t *s, const sensor_snapshot_t *snap, float dt);
/* both side walls seen → heading_ref += P_REALIGN_RATE * wrap180(yaw - heading_ref) * dt */
int32_t steer_front_remaining(int32_t enc_remaining_ticks, int32_t front_tof_mm);
/* if front_tof valid and |(front_tof - P_FRONT_ALIGN_MM) - enc_remaining_mm| ≤ P_FRONT_ALIGN_WIN_MM
   → return (front_tof - P_FRONT_ALIGN_MM) in ticks, else enc_remaining_ticks */
```
- Steering keeps `TOF_WALL_MAX_MM`, `TOF_SIDE_TARGET_MM`; snapshot ToF `-1` = invalid.

- [ ] **Step 1: Failing tests `test_steering.c`** (`// SRCS: main/tasks/motion/steering.c main/params.c`; stub `sensor_snapshot_t` via `app_types.h`):
```c
params_reset();  /* TICKS_PER_CELL 1400 → 5.6 ticks/mm, ALIGN 40, WIN 60 */
CHECK(steer_front_remaining(560, -1) == 560);                    /* no reading */
CHECK(steer_front_remaining(560, 150) == (int32_t)(110 * 5.6f)); /* 100 mm enc vs 110 mm tof: inside window */
/* test_front_remaining_ignores_outside_window */
CHECK(steer_front_remaining(1400, 60) == 1400);                  /* 250 mm enc vs 20 mm tof: spurious */
steer_t s = { .heading_ref = 0 };  /* REALIGN_RATE default 0 → no change */
sensor_snapshot_t snap = { .left_tof = 85, .right_tof = 85, .yaw = 2.0f };
steer_realign(&s, &snap, 0.01f);  CHECK_NEAR(s.heading_ref, 0, 1e-6);
```
  then set realign via `params_step` to a nonzero value and `CHECK(s.heading_ref > 0 && s.heading_ref < 2)`; with one wall missing (`right_tof = -1`) `heading_ref` unchanged.
- [ ] **Step 2: `tests/host/run.sh`** — FAIL; **Step 3: implement**; **Step 4:** PASS + `idf.py build` complete.
- [ ] **Step 5: Wire in motion_task** (straight remaining = `steer_front_remaining(...)`; realign each step); tuner DISTANCE page gets `TICKS_PER_CELL`, `FRONT_ALIGN_MM`, `FRONT_ALIGN_WIN_MM`, CENTERING page gets `REALIGN_RATE`.
- [ ] **Step 6: Robot check:** 1 cell toward a wall → stops at ~`FRONT_ALIGN_MM` from it.
- [ ] **Step 7: Commit** `feat(motion): heading re-align from side walls and front-wall distance stop`.

---

### Task 6: Maze (visited, goal group), store, path planner

**Files:**
- Move: `main/maze.{h,c}` → `main/app/maze.{h,c}`
- Create: `main/app/maze_store.{h,c}`, `main/app/path.{h,c}`, `tests/host/test_maze.c`, `tests/host/test_path.c`, `tests/host/test_store.c`
- Modify: `main/app_config.h`, `main/tasks/navigation_task.c`, `main/CMakeLists.txt` (REQUIRES `nvs_flash`)

**Interfaces:**
```c
/* app_config.h */
#define MAZE_GOAL_CELLS { {1, 1} }      /* any count ≥ 1 */
/* MAZE_GOAL_X / MAZE_GOAL_Y removed */

/* maze.h additions/changes */
typedef struct { uint8_t x, y; } maze_cell_t;
/* maze_t gains: uint8_t visited[MAZE_MAX_SIZE][MAZE_MAX_SIZE]; goals[MAZE_MAX_GOALS]; goal_count */
void maze_init(maze_t *m, uint8_t w, uint8_t h, const maze_cell_t *goals, uint8_t goal_count);
bool maze_is_goal(const maze_t *m, int x, int y);
void maze_flood(maze_t *m);                 /* multi-source BFS from all goals, unknown = open */
void maze_mark_visited(maze_t *m, int x, int y);
bool maze_visited(const maze_t *m, int x, int y);

/* maze_store.h — blob validation split out so it is host-testable */
bool      maze_blob_valid(const maze_t *saved, const maze_t *expected_shape); /* w, h, goals equal */
esp_err_t maze_store_save(const maze_t *m);
esp_err_t maze_store_load(maze_t *m);       /* ESP_ERR_NOT_FOUND / ESP_ERR_INVALID_VERSION on mismatch */

/* path.h */
typedef struct { move_type_t type; int16_t cells; int16_t angle_deg; } path_move_t;
int path_plan(const maze_t *m, int sx, int sy, maze_dir_t sdir,
              path_move_t *out, int max_moves);   /* returns count, -1 if no route */
/* BFS over visited cells, unknown walls = closed, target = nearest goal; compress to
   STRAIGHT n / TURN ±90 / TURN 180 (TURN 180 only possible as first move). */
```
- Blob: `{ uint32_t magic 'MZ01'; uint16_t version 1; maze_t maze; }` in NVS namespace `"maze"`, key `"map"`; `nvs_flash_init()` in `app_main` (erase + retry on `ESP_ERR_NVS_NO_FREE_PAGES`/`NEW_VERSION_FOUND`).

- [ ] **Step 1: Failing tests:**
  - `test_maze.c`: 4×4 maze, goals {(1,1),(1,2),(2,1),(2,2)} → `maze_dist` of each goal == 0, of (0,0) == 2; `maze_is_goal(2,2)` true, `(0,0)` false.
  - `test_path.c`: 3×3 maze, goal (2,2), all cells visited, walls forming an L corridor (0,0)→(0,2)→(2,2): expect `{STRAIGHT 2, TURN -90, STRAIGHT 2}` from (0,0) facing north. Unvisited shortcut cell present → path avoids it. Goal unreachable through visited cells → `-1`.
  - `test_store.c` (`test_store_rejects_mismatch`, SRCS `maze_store.c` built with `-DMAZE_STORE_HOST` so NVS calls are excluded): blob with w=3 vs expected w=4 → false; same size, different goal list → false; identical shape → true.
- [ ] **Step 2: `tests/host/run.sh`** — FAIL; **Step 3: implement**; **Step 4:** PASS.
- [ ] **Step 5: Update navigation_task** to new maze API (marks visited each cell, `maze_is_goal`). `idf.py build` complete.
- [ ] **Step 6: Commit** `feat(maze): goal cell group, visited cells, NVS store and speed-run path planner`.

---

### Task 7: App task, FSM skeleton, UI helpers, MENU / TUNER / FAULT

**Files:**
- Create: `main/app/app_task.{h,c}`, `main/app/app_states.h`, `main/app/ui.{h,c}`, `main/app/state_menu.c`, `main/app/state_tuner.c`, `main/app/state_fault.c`
- Delete: `main/bench.{h,c}`, `main/tuning_task.{h,c}`, `main/tasks/telemetry_task.{h,c}`, all `BENCH_*` defines
- Modify: `main/main.c`, `main/CMakeLists.txt`, `main/app_config.h` (APP_TASK_* , `UI_POLL_MS 50`, `START_DELAY_MS 500`)

**Interfaces:**
```c
/* app_states.h */
typedef enum { ST_MENU, ST_EXPLORE, ST_SPEED_RUN, ST_TUNER, ST_FAULT, ST_COUNT } app_state_id_t;
typedef struct {
    maze_t      maze;           /* RAM map, kept across states */
    bool        maze_valid;
    const char *fault_reason;   /* set before → ST_FAULT */
} app_ctx_t;
typedef app_state_id_t (*state_run_fn)(app_ctx_t *ctx);
app_state_id_t state_menu_run(app_ctx_t *ctx);   /* … one per state */

/* ui.h */
typedef enum { BTN_BOOT, BTN_SEL, BTN_INC, BTN_DEC, BTN_COUNT } ui_btn_t;
typedef struct { bool pressed[BTN_COUNT]; } ui_events_t;
ui_events_t ui_poll(void);                         /* edge-detected, primed so held-at-boot ≠ press */
void ui_draw_list(const char *title, const char *const *rows, int count, int cursor, const char *footer);
void ui_message(const char *title, const char *line1, const char *line2);
bool ui_arm_and_wait(const char *title);           /* "INC start / DEC back"; false = back */
bool ui_run_guard(void);                           /* call while a move runs: DEC → motion_stop(), returns false */
void ui_format_report(const move_report_t *r, char *buf, size_t len);
```
- App task (core 0, prio 3): `params_reset()` already done in main; loop `state = s_states[state](&ctx)` over `static const state_run_fn s_states[ST_COUNT]`.
- MENU rows: `EXPLORE`, `SPEED RUN`, `TUNER`; SEL moves, INC enters. Until Tasks 8/9, EXPLORE and SPEED RUN go to FAULT with reason "not ready".
- TUNER: pages table `{ title, const param_id_t *rows, n, const tuner_action_t *actions, n_actions }` where `tuner_action_t = { label, void (*run)(int cells) }`; pages DISTANCE, CENTERING, SPEED (incl. PROFILE EXP/FAST toggle row and cruise/accel/decel of the selected profile), TURNS, DUMP exactly as spec §7.6. Last report line under the header. BOOT = next page; BOOT on DUMP → `ST_MENU`. INC on action row runs, DEC on action row stops. While a test runs, poll `ui_run_guard()` and redraw progress at ~10 Hz.
- FAULT: shows `ctx->fault_reason`, motors already stopped; any button → `ST_MENU`.
- `main.c`: NVS init, IPC init, OLED init, IMU init + calibrate, ToF, encoders, motors; start sensor, motion, app tasks. Logging: global `ESP_LOG_WARN`, app tag `INFO`.

- [ ] **Step 1: Implement** the files above; delete bench/tuner/telemetry.
- [ ] **Step 2: `idf.py build`** → complete; `grep -rn BENCH_ main` → no hits.
- [ ] **Step 3: Robot check:** boot → MENU; TUNER pages cycle; each `[RUN]` shows a result line; DEC during a 5-cell run stops the robot immediately; DUMP prints lines that paste into `app_config.h` and build.
- [ ] **Step 4: Commit** `feat(app): robot FSM with menu, tuner and fault states; remove bench and telemetry tasks`.

---

### Task 8: EXPLORE state

**Files:**
- Create: `main/app/state_explore.c`
- Delete: `main/tasks/navigation_task.{h,c}`
- Modify: `main/app_config.h` (`NAV_TASK_*` removed)

**Interfaces:**
- Consumes `motion_client`, `maze`, `maze_store`, `ui_run_guard`, `ui_arm_and_wait`.
- `state_explore_run`: arm → fresh `maze_init` with `MAZE_GOAL_CELLS` → existing navigation algorithm (record walls, flood, face, run with decide-ahead at `NAV_DECIDE_FRAC * P_TICKS_PER_CELL` and `motion_extend(1)`, re-plan when wall after turn), `maze_mark_visited` on each cell entered, `PROFILE_EXPLORE`. On goal: `maze_store_save`, `ctx->maze_valid = true`, message "GOAL / saved" → `ST_MENU`. On failure: no save, reason (`"NO PATH"`, `"MOVE FAULT"`, `"STOPPED"`, `"STEP LIMIT"`) → `ST_FAULT`. Status line: cell x,y and heading, ~10 Hz.

- [ ] **Step 1: Implement** (port functions from `navigation_task.c`, keeping their names `record_walls`, `face`, `run`, `explore_step` as `static`).
- [ ] **Step 2: `idf.py build`** → complete.
- [ ] **Step 3: Robot check:** small maze, explore reaches goal, reboot, SPEED RUN entry finds the saved map (Task 9 shows it; here confirm "saved" message and no fault).
- [ ] **Step 4: Commit** `feat(app): explore state with automatic maze save at goal`.

---

### Task 9: SPEED_RUN state

**Files:**
- Create: `main/app/state_speed_run.c`
- Modify: `main/app_config.h` (`PATH_MAX_MOVES 64`)

**Interfaces:**
- `state_speed_run_run`: `maze_store_load` → fail → message "No maze" → `ST_MENU`. `path_plan` from `MAZE_START_X/Y/DIR` → `-1` → `ST_FAULT` "NO PATH". Show move count, arm, then execute each `path_move_t` with `motion_straight(cells, PROFILE_FAST)` / `motion_turn(angle, PROFILE_FAST)`, `motion_wait` + `ui_run_guard` per move. Done → message "DONE" + total time → `ST_MENU`; fault → `ST_FAULT`.

- [ ] **Step 1: Implement.** **Step 2: `idf.py build`** → complete.
- [ ] **Step 3: Robot check:** after Task 8 explore, power-cycle, SPEED RUN drives the stored path to the goal; with no saved map (erase via `idf.py erase-flash` once) shows "No maze".
- [ ] **Step 4: Commit** `feat(app): speed run from saved maze`.

---

### Task 10: Docs and repo cleanup

**Files:**
- Modify: `README.md` (status, architecture: 3 tasks + FSM + tuner pages; remove `progress_notes.md` link), `hardware.md` (add buttons SEL 21, INC 47, DEC 48, BOOT 0), `AGENTS.md` (host tests: `tests/host/run.sh`)
- Add to `.gitignore`: `build_host/`

- [ ] **Step 1: Update docs.** **Step 2:** `tests/host/run.sh` PASS and `idf.py build` complete. **Step 3:** `git status` clean except intended. **Step 4: Commit** `docs: update README and hardware for new architecture`.
