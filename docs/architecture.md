# micro_farr firmware architecture

Three FreeRTOS tasks, one robot FSM, one control cascade for every move.
Design rationale: [firmware restructure design](superpowers/specs/2026-11-21-firmware-restructure-design.md).

## 1. Hardware

![Hardware block diagram](diagrams/hardware.drawio.svg)

Editable: open `docs/diagrams/hardware.drawio.svg` in draw.io. Pin map: [hardware.md](../hardware.md).

## 2. Tasks and IPC

```mermaid
flowchart LR
    subgraph IN["sensors"]
        tof["ToF ×3"]:::hw
        imu["IMU"]:::hw
        enc["encoders"]:::hw
    end
    subgraph C1["core 1"]
        sensor["sensor · core 1 · prio 9 · 100 Hz"]:::task
        motion["motion · core 1 · prio 10 · once per sample"]:::task
    end
    subgraph C0["core 0"]
        app["app · core 0 · prio 3 · UI poll 20 Hz"]:::task
    end
    snap[("sensor mailbox")]:::ipc
    newbit{{"SENSOR_NEW bit"}}:::ipc
    cmdq[["motion queue (4)"]]:::ipc
    status[("status mailbox")]:::ipc
    events{{"MOVE_DONE / MOVE_FAULT bits"}}:::ipc
    params[/"params table"/]:::cfg
    motors["motors"]:::hw
    ui["OLED + buttons"]:::hw
    nvs["NVS: saved maze"]:::hw

    tof --> sensor
    imu --> sensor
    enc --> sensor
    sensor -- "sensor_snapshot_t" --> snap
    sensor -- "signal" --> newbit
    newbit --> motion
    snap -- "sensor_snapshot_t" --> motion
    snap -- "walls" --> app
    app -- "motion_cmd_t" --> cmdq
    cmdq --> motion
    motion -- "motion_status_t" --> status
    status --> app
    motion -- "MOVE_DONE" --> events
    events --> app
    app -- "params_step" --> params
    params -- "params_get" --> motion
    motion --> motors
    app <--> ui
    app <--> nvs

    subgraph LEGEND["Legend"]
        direction LR
        l1["task"]:::task
        l2["IPC channel"]:::ipc
        l3["hardware"]:::hw
        l4["config store"]:::cfg
    end

    classDef task fill:#dae8fc,stroke:#6c8ebf,color:#000
    classDef ipc fill:#e1d5e7,stroke:#9673a6,color:#000
    classDef hw fill:#f5f5f5,stroke:#666666,color:#000
    classDef cfg fill:#d5e8d4,stroke:#82b366,color:#000
```

| Task | Core | Prio | Rate | Owns | Reads | Writes |
| --- | --- | --- | --- | --- | --- | --- |
| sensor | 1 | 9 | 100 Hz | ToF, IMU, encoders | hardware | sensor mailbox, SENSOR_NEW |
| motion | 1 | 10 | once per snapshot | motors | queue, snapshot, params | status mailbox, MOVE_DONE / MOVE_FAULT |
| app | 0 | 3 | state-driven, UI poll 20 Hz | OLED, buttons, maze, NVS | status, events, snapshot | queue, params |

`params` is the only shared module outside `app_ipc`: a RAM table of tunable values (aligned
32-bit floats, written by the app, read by motion).

## 3. Robot FSM

```mermaid
stateDiagram-v2
    [*] --> MENU: boot, IMU calibrated
    MENU --> EXPLORE: INC on EXPLORE
    MENU --> SPEED_RUN: INC on SPEED RUN
    MENU --> TUNER: INC on TUNER
    EXPLORE --> MENU: goal, map saved
    SPEED_RUN --> MENU: done, or no maze
    TUNER --> MENU: BOOT on DUMP page
    EXPLORE --> FAULT: move fault, no path, DEC
    SPEED_RUN --> FAULT: move fault, no path, DEC
    FAULT --> MENU: any button
    note right of FAULT
        Legend: arrow label = trigger.
        FAULT: motors stopped, RAM map kept,
        saved map untouched.
    end note
```

Each state is a function `run(ctx)` that returns the next state; the app task loops
`state = s_states[state](&ctx)`.

## 4. Control cascade

```mermaid
flowchart LR
    prof["trapezoid profile<br>distance (straight) or angle (turn)"]:::ctrl
    steer["steering<br>heading hold + centering + re-align"]:::ctrl
    body["body to wheels<br>v ∓ ω·B/2"]:::ctrl
    wl["wheel loop L<br>FF + PI"]:::ctrl
    wr["wheel loop R<br>FF + PI"]:::ctrl
    ml["motor L"]:::hw
    mr["motor R"]:::hw
    gyro["gyro yaw"]:::hw
    side["side ToF"]:::hw
    front["front ToF"]:::hw
    encs["encoders"]:::hw

    prof -- "v_ref (straight) / ω_ref (turn)" --> body
    steer -- "ω_ref (straight)" --> body
    body -- "v_L" --> wl
    body -- "v_R" --> wr
    wl --> ml
    wr --> mr
    encs -.-> wl
    encs -.-> wr
    encs -. "progress" .-> prof
    front -. "remaining" .-> prof
    gyro -.-> steer
    side -.-> steer

    subgraph LEGEND["Legend"]
        direction LR
        k1["controller"]:::ctrl
        k2["hardware"]:::hw
        k3["solid = reference, dashed = feedback"]
    end

    classDef ctrl fill:#dae8fc,stroke:#6c8ebf,color:#000
    classDef hw fill:#f5f5f5,stroke:#666666,color:#000
```

- Straight: the profile runs on the remaining ticks (front ToF replaces the encoder value when
  they agree within `FRONT_ALIGN_WIN_MM`), steering adds ω.
- Turn: the profile runs on the remaining angle and gives ω; v = 0.
- Only the wheel loops write motor duty (plus brake at move end).

## 5. One explore step

```mermaid
sequenceDiagram
    participant A as app (EXPLORE)
    participant M as motion task
    participant S as sensor task
    A->>M: motion_straight(1, EXPLORE)
    loop every snapshot (100 Hz)
        S->>M: SENSOR_NEW + sensor_snapshot_t
        M->>M: profile, steering, wheel PI
    end
    A->>M: reads status: progress reached decide point
    A->>M: motion_extend(1) if the path goes straight on
    M-->>A: MOVE_DONE + move_report_t
    A->>A: mark visited, record walls, flood fill, face next cell
    Note over A,S: Legend: solid = command or signal, dashed = completion event
```
