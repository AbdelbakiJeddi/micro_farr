---
type: project
status: active
started: 2026-10-30
last-touched: 2026-10-30
next-action: Lab 1.1 (two blinkers)
tags: [project, freertos, esp32-s3, micromouse]
---

# Micromouse Firmware on FreeRTOS

## Goal
Build a micromouse robot whose firmware is a clean, real-time FreeRTOS architecture: a 1 ms control loop with under 100 µs jitter, reliable sensing, maze solving, and a supervisor that recovers from task failures.

## Short description
A maze-solving robot built on an ESP32-S3 with 3 ToF distance sensors, an MPU9250 IMU, 2 N20 motors with encoders, and a LiPo battery. The project is built in two stages. First, the task architecture runs with simulated sensor data to prove the timing and design. Then the real hardware is connected, and the robot explores a maze, maps it, and runs the shortest path.

## What I will learn
- **Tasks and scheduling:** priorities, core pinning (dual core), periodic tasks with `vTaskDelayUntil`, preventing starvation
- **Timing:** measuring jitter and designing a hard-timed control loop
- **Memory:** stack sizing with high water marks, heap use, static allocation
- **Inter-task communication:** queues, task notifications, event groups, gatekeeper pattern
- **Interrupts:** ISR-safe APIs and deferring work from encoders and buttons to tasks
- **Shared resources:** mutexes, priority inversion, deadlock avoidance, I2C bus ownership
- **Reliability:** watchdogs, heartbeat supervisor, restarting failed tasks
- **Debugging:** run time stats, panic decoding, crash analysis
- **Robotics:** PID speed control, sensor fusion (ToF + IMU), maze mapping and solving

## Success criteria
1. Control loop period error under 100 µs over 5 minutes
2. No task stack above 75% usage
3. Slow or blocked logger/solver tasks never affect the control loop
4. Supervisor detects and restarts a killed sensor task
5. No shared global written by more than one task
6. Run time stats show a stable CPU share for the control loop
7. Robot solves a test maze (target time: ___)

## Milestones
- [ ] Course Modules 1-3: task skeleton with fake data
- [ ] Modules 4-7: queues, ISRs, synchronization
- [ ] Modules 8-9: debugging tools, supervisor, state machine
- [ ] Capstone: real sensors, encoders and motors under the control loop
- [ ] Maze run and final measurements

## Next action
- [ ] Lab 1.1: two blinkers

## Log
- 2026-10-30: project started, FreeRTOS course in progress
