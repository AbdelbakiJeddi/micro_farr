# Project Progress & Development Notes — `micro_farr`

This document tracks all active files, milestone progression, and hardware driver tests for the `micro_farr` ESP32-S3 micromouse robot.

---

## 1. Milestone Status Dashboard

| Milestone | Status | Description | Notes |
|---|---|---|---|
| **M1: Bringup & Bus** | 🟢 **Complete** | Project structure, pin map, modern `i2c_bus` driver | `i2c_bus` using ESP-IDF v6 `driver/i2c_master.h` on GPIO 8/9 |
| **M2: ToF Re-addressing** | ⚪ Not Started | Sequential XSHUT toggling, addresses `0x30, 0x31, 0x32` | Next step after IMU verification |
| **M3: Motors & Encoders** | 🟢 **Complete** | TB6612 LEDC PWM + PCNT hardware quadrature | `motor.c` + `encoder.c` fully implemented and bench-tested |
| **M4: IMU Yaw Stability** | 🟢 **In Test** | MPU6050 probe, rest bias calibration, yaw integration | `imu.c` implemented; currently running `BENCH_IMU_TEST` |
| **M5: Straight Cell PID** | ⚪ Not Started | Distance PID driving forward `CELL_TICKS` | Requires combining motor + encoder PID |
| **M6: Yaw Hold & Centering** | ⚪ Not Started | Yaw lock + lateral ToF corridor centering | Requires M4 (IMU) + M2 (ToF) |
| **M7: ±90° Accurate Turns** | ⚪ Not Started | Pivot turns using gyro feedback | Uses M4 (IMU) |
| **M8: Wall Recording** | ⚪ Not Started | Cell-by-cell traversal with wall detection | Maze grid representation |
| **M9: Full Flood Fill** | ⚪ Not Started | BFS flood fill exploration & shortest path solve | Navigation logic |

*Legend: 🟢 Complete | 🟡 In Progress / In Test | ⚪ Not Started*

---

## 2. Codebase File Map (`main/`)

| File | Status | Description |
|---|---|---|
| [`main/CMakeLists.txt`](file:///home/abok-omen/Desktop/Abok/embedded/micro_farr/main/CMakeLists.txt) | 🟢 Active | Registers `main.c`, `i2c_bus.c`, `motor.c`, `encoder.c`, `imu.c`; requires `esp_driver_gpio`, `esp_driver_ledc`, `esp_driver_pcnt`, `esp_driver_i2c`, `freertos` |
| [`main/pins.h`](file:///home/abok-omen/Desktop/Abok/embedded/micro_farr/main/pins.h) | 🟢 Active | Hardware GPIO definitions matching `hardware.md` |
| [`main/app_types.h`](file:///home/abok-omen/Desktop/Abok/embedded/micro_farr/main/app_types.h) | 🟢 Active | Enums (`robot_state_t`, `motion_cmd_t`, `imu_model_t`) and structs |
| [`main/app_config.h`](file:///home/abok-omen/Desktop/Abok/embedded/micro_farr/main/app_config.h) | 🟢 Active | Bench test selectors (`BENCH_MOTOR_TEST`, `BENCH_ENCODER_TEST`, `BENCH_IMU_TEST`), I2C/PCNT/IMU parameters |
| [`main/drivers/i2c_bus.h`](file:///home/abok-omen/Desktop/Abok/embedded/micro_farr/main/drivers/i2c_bus.h) / [`.c`](file:///home/abok-omen/Desktop/Abok/embedded/micro_farr/main/drivers/i2c_bus.c) | 🟢 Active | Modern ESP-IDF v6 `i2c_master` bus master initialization on GPIO 8 (SDA) and 9 (SCL) |
| [`main/drivers/motor.h`](file:///home/abok-omen/Desktop/Abok/embedded/micro_farr/main/drivers/motor.h) / [`.c`](file:///home/abok-omen/Desktop/Abok/embedded/micro_farr/main/drivers/motor.c) | 🟢 Active | TB6612FNG dual motor driver via 20 kHz 10-bit LEDC PWM, standby gating, direction control |
| [`main/drivers/encoder.h`](file:///home/abok-omen/Desktop/Abok/embedded/micro_farr/main/drivers/encoder.h) / [`.c`](file:///home/abok-omen/Desktop/Abok/embedded/micro_farr/main/drivers/encoder.c) | 🟢 Active | Hardware Pulse Counter (`pulse_cnt`) 4x quadrature decoding with 1000 ns glitch filter |
| [`main/drivers/imu.h`](file:///home/abok-omen/Desktop/Abok/embedded/micro_farr/main/drivers/imu.h) / [`.c`](file:///home/abok-omen/Desktop/Abok/embedded/micro_farr/main/drivers/imu.c) | 🟢 Active | MPU6050/6500/9250 auto-probe, wake, DLPF (42 Hz), 500-sample rest bias calibration, yaw integration |
| [`main/main.c`](file:///home/abok-omen/Desktop/Abok/embedded/micro_farr/main/main.c) | 🟢 Active | Boot entrypoint and bench test runners (motor, encoder, IMU) |

---

## 3. Bench Test Log

- **Bench Motor Test (`BENCH_MOTOR_TEST`)**: Verified forward/reverse spin on left and right channels at 30% duty cycle for 2000 ms.
- **Bench Encoder Test (`BENCH_ENCODER_TEST`)**: Verified hardware PCNT delta and cumulative ticks on both wheels at 100 ms period.
- **Bench IMU Test (`BENCH_IMU_TEST`)**: Auto-probes `0x68`/`0x69`, reads `WHO_AM_I`, performs rest bias calibration across 500 samples, and logs yaw in degrees at 10 Hz.
