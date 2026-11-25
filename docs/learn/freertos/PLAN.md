# FreeRTOS, the Hard Way: plan

Built with course-maker v2. Local multi-page course: open `index.html`.

| # | Page | Interactive | Anchor | Status |
|---|---|---|---|---|
| 1 | 01-why-rtos | super-loop vs RTOS | main/main.c:74 | done |
| 2 | 02-tasks | — (context switch + states SVG) | sensor_task.c:90 | done |
| 3 | 03-scheduler | starvation | app_config.h:42 | done |
| 4 | 04-timing | vTaskDelay drift | sensor_task.c:71 | done |
| 5 | 05-queues | full queue | app_ipc.c:41 | done |
| 6 | 06-mailbox | — | app_ipc.c:58 | done |
| 7 | 07-events | stale bit | app_ipc.h:12 | done |
| 8 | 08-shared-data | race, priority inversion | params.c:35 | done |
| 9 | 09-dual-core | — | app_config.h:40 | done |
| 10 | 10-design | — | docs/architecture.md | done |
| 11 | 11-synthesis | — | — | done |

## Verified numbers (node / gcc / IDF source, 2026-11-25)
- Simulations: super-loop 4/6 missed, RTOS 0; polling 0 samples, blocking 4; delay 5 passes, until 6;
  queue timeout 0 → 12 lost, portMAX_DELAY → 0 lost / 27 ms blocked; stale bit returns at 17 ms vs 29 ms;
  race count 1 vs 2; inversion High blocked 15 ms vs 3 ms.
- sizeof: motion_cmd_t 12, sensor_snapshot_t 32, move_report_t 28, motion_status_t 40, maze_t 787.
- OLED frame 1024 × 9 / 400 kHz = 23.04 ms; 50 ms blind at 450 mm/s = 22.5 mm.
- IDF: configMAX_PRIORITIES 25, esp_timer prio 22, IPC prio 24, 24 event bits, CONFIG_FREERTOS_HZ=1000, WDT 5 s.

## Open points
- Committed on branch `restructure`. Not published as an artifact (multi-page, local).
