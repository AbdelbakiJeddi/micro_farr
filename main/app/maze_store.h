#pragma once

/* The explored maze, saved to NVS at the goal and loaded for the speed run. */

#include <stdbool.h>

#include "maze.h"

/* A saved maze is usable only if its size and goal list match the current config. */
bool maze_blob_valid(const maze_t *saved, const maze_t *expected_shape);

#ifndef MAZE_STORE_HOST
#include "esp_err.h"

esp_err_t maze_store_save(const maze_t *m);   /* overwrites the previous map */

/* ESP_ERR_NOT_FOUND: nothing saved; ESP_ERR_INVALID_VERSION: format or shape mismatch. */
esp_err_t maze_store_load(maze_t *m);
#endif
