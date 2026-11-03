# Hardware

| Part | Model | Qty | Role |
|------|-------|-----|------|
| MCU | ESP32-S3-WROOM-1-N16R8 | 1 | Controller |
| Motor driver | TB6612FNG | 1 | Dual H-bridge |
| Motor | DC + encoder | 2 | Drive |
| Encoder | Quadrature | 2 | Wheel feedback |
| ToF | VL53L0X | 3 | Front, left, right walls |
| IMU | MPU6050 | 1 | Heading |
| OLED | SSD1306 | 1 | Display |

# Pin Map

## I2C
| Signal | GPIO |
|--------|------|
| SDA | 8 |
| SCL | 9 |

## Encoders (PCNT)
| Signal | GPIO |
|--------|------|
| ENC_L_CHA | 17 |
| ENC_L_CHB | 18 |
| ENC_R_CHA | 15 |
| ENC_R_CHB | 16 |

## ToF XSHUT
| Signal | GPIO |
|--------|------|
| XSHUT_L | 6 |
| XSHUT_F | 5 |
| XSHUT_R | 4 |

## Motor Driver
| Signal | GPIO | Use |
|--------|------|-----|
| PWMA | 1 | Left PWM |
| AIN1 | 2 | Left dir |
| AIN2 | 42 | Left dir |
| PWMB | 38 | Right PWM |
| BIN1 | 39 | Right dir |
| BIN2 | 40 | Right dir |
| STBY | 41 | Enable |

## Reserved GPIO
- 33–37: octal PSRAM
- 0, 3, 45, 46: strapping
- 19, 20: USB
