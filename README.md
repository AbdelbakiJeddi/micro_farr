# micro_farr

Autonomous micromouse robot powered by the ESP32-S3 microcontroller, designed for real-time maze exploration and solving.

---

## Overview

**micro_farr** is a compact, two-wheeled autonomous mobile robot built to solve standard grid mazes. Leveraging the dual-core processing power of the ESP32-S3, the robot combines real-time sensor feedback, closed-loop motion control, and autonomous decision-making to navigate maze environments without external assistance or remote intervention.

---

## Robot Mission

The primary objective of the robot is autonomous maze exploration and path resolution:

- **Autonomous Exploration**: Navigate an unknown grid maze from a designated starting cell, detecting and mapping walls in real time.
- **Cell-to-Cell Traversal**: Execute precise, discrete motion primitives—driving straight through cell units and executing accurate 90° pivot turns.
- **Distance & Heading Sensing**: Continuously measure distances to surrounding walls using three Time-of-Flight (ToF) sensors while maintaining heading accuracy using a 6-axis inertial measurement unit (IMU).
- **Goal Resolution & Shortest Path**: Map the maze topology and calculate the optimal shortest path to the goal cells using the flood-fill algorithm.

---

## Current Implementation Status

The current firmware baseline is focused on bring-up and straight-cell motion:

- Shared I2C bus at 400 kHz for the MPU6050, OLED, and three VL53L0X sensors.
- VL53L0X XSHUT sequencing and per-sensor address assignment (`0x30`–`0x32`).
- Quadrature encoder totals published through the sensor snapshot mailbox.
- 100 Hz sensor task for ToF servicing, gyro yaw integration, and encoder sampling.
- 200 Hz motion task for multi-cell straight segments, tick-distance control, acceleration slew, and yaw-hold correction.
- 20 Hz OLED telemetry task showing motion state, progress, yaw, and wheel ticks.
- Configurable bench tests for motors, encoders, IMU, ToF, cell calibration, OLED, telemetry, and straight motion.

Flood-fill navigation, wall recording, and accurate pivot turns remain planned work. See
[`progress_notes.md`](progress_notes.md) for the milestone dashboard and bench results.

## Firmware Architecture

Application code is organized into three layers:

| Layer | Location | Responsibility |
|---|---|---|
| Drivers | `main/drivers/` | Motor, encoder, I2C, IMU, ToF, and OLED hardware access |
| Tasks | `main/tasks/` | Periodic sensing, motion control, and telemetry |
| Application | `main/` | Configuration, IPC, bench selection, and startup |

Tasks communicate through the IPC interfaces in `main/app_ipc.*`; tuning constants and
bench flags are centralized in `main/app_config.h`.

## Build and Flash

The project targets the ESP32-S3-WROOM-1-N16R8 and uses ESP-IDF 6.1:

```bash
source ~/.espressif/v6.1/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Set the desired `BENCH_*` flag in `main/app_config.h` before building. The safe straight
bench defaults to five cells, a 30% duty cap, and a measured `CELL_TICKS` value of 1495.

---

## Hardware Architecture

| Component | Model / Type | Quantity | Purpose |
|---|---|:---:|---|
| **MCU** | ESP32-S3 (WROOM-1-N16R8) | 1 | Dual-core 240 MHz controller, 16MB Flash, 8MB Octal PSRAM |
| **Motor Driver** | TB6612FNG | 1 | Dual H-bridge motor driver with standby control |
| **Motors** | Micro Metal DC Gearmotors | 2 | Differential drive propulsion |
| **Encoders** | Magnetic Quadrature Encoders | 2 | Wheel rotation and travel feedback |
| **Distance Sensors** | VL53L0X Time-of-Flight (ToF) | 3 | Millimeter-accurate distance measurement (Front, Left, Right) |
| **IMU** | MPU6050 (6-axis) | 1 | Angular velocity (gyroscope) and heading tracking |
| **Display** | SSD1306 OLED (128x64) | 1 | Live diagnostic and status display |

---

## Pinout & Wiring Specification

All peripherals are interfaced with the **ESP32-S3-WROOM-1-N16R8** module according to the following pin configuration:

### Shared I2C Bus
| Signal | GPIO | Connected Devices |
|---|:---:|---|
| SDA** | `GPIO 8` | MPU6050, SSD1306 OLED, 3x VL53L0X ToF |
| **SCL** | `GPIO 9` | MPU6050, SSD1306 OLED, 3x VL53L0X ToF |

### Quadrature Wheel Encoders
| Signal | GPIO | Function |
|---|:---:|---|
| **ENC_L_CHA** | `GPIO 17` | Left Wheel Channel A |
| **ENC_L_CHB** | `GPIO 18` | Left Wheel Channel B |
| **ENC_R_CHA** | `GPIO 15` | Right Wheel Channel A |
| **ENC_R_CHB** | `GPIO 16` | Right Wheel Channel B |

### Distance Sensors (XSHUT Control)
| Signal | GPIO | Function |
|---|:---:|---|
| **XSHUT_L** | `GPIO 6` | Left ToF Address Select / Shutdown |
| **XSHUT_F** | `GPIO 5` | Front ToF Address Select / Shutdown |
| **XSHUT_R** | `GPIO 4` | Right ToF Address Select / Shutdown |

### Motor Driver (TB6612FNG)
| Signal | GPIO | Function |
|---|:---:|---|
| **PWMA** | `GPIO 1` | Left Motor Speed (PWM) |
| **AIN1** | `GPIO 2` | Left Motor Direction 1 |
| **AIN2** | `GPIO 42` | Left Motor Direction 2 |
| **PWMB** | `GPIO 38` | Right Motor Speed (PWM) |
| **BIN1** | `GPIO 39` | Right Motor Direction 1 |
| **BIN2** | `GPIO 40` | Right Motor Direction 2 |
| **STBY** | `GPIO 41` | Driver Enable / Standby (Active HIGH) |

### Reserved Pins (Do Not Reassign)
- **`GPIO 33–37`**: Dedicated internal bus for Octal PSRAM.
- **`GPIO 0, 3, 45, 46`**: Boot configuration & strapping pins.
- **`GPIO 19, 20`**: USB D- / D+ communication.
