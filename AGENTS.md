# Project Rules

## Code quality
- Code must be well structured and decomposed.
- One responsibility per module; clear layering (drivers -> tasks -> app).
- Small, single-purpose functions; no duplicated logic.
- Prefer structs + pointers over duplicating behavior with switch/branching.
- Public headers expose a minimal API; keep internals static.
- Configuration constants live in `app_config.h`.
- Tasks communicate only through `app_ipc`.

## Build / verify
- ESP-IDF v6.1. Environment: `source ~/.espressif/v6.1/esp-idf/export.sh`.
- After each change run `idf.py build` and confirm success.
- Target: ESP32-S3-WROOM-1-N16R8 (16 MB flash, 8 MB octal PSRAM).
