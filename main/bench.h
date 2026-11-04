#pragma once

#include <stdbool.h>

typedef void (*bench_init_fn_t)(void);

/**
 * @brief Checks if any bench test is enabled in app_config.h.
 *        If a standalone test is enabled, runs it directly.
 *        If an integrated test is enabled (e.g. straight run), calls init_fn first.
 * @param init_fn Function to initialize robot subsystems/tasks for integrated tests.
 * @return true if a bench test was enabled and executed; false if normal app should run.
 */
bool bench_run_if_enabled(bench_init_fn_t init_fn);
