#pragma once

/* Speed-run path: shortest route from the start to the nearest goal cell through visited
 * cells only (unknown walls count as closed), as a list of straight runs and pivots. */

#include <stdint.h>

#include "app_types.h"
#include "maze.h"

typedef struct {
    move_type_t type;       /* MOVE_STRAIGHT or MOVE_TURN */
    int16_t     cells;      /* MOVE_STRAIGHT */
    int16_t     angle_deg;  /* MOVE_TURN: +90 left, -90 right, 180 */
} path_move_t;

/* Returns the number of moves written to out, or -1 if there is no route (or it does not
 * fit in max_moves). TURN 180 can only be the first move. */
int path_plan(const maze_t *m, int sx, int sy, maze_dir_t sdir,
              path_move_t *out, int max_moves);
