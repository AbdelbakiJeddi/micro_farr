#pragma once

#include <stdbool.h>
#include <stdint.h>

#define MAZE_MAX_SIZE    16
#define MAZE_UNREACHABLE 255

/* Compass directions; x grows east, y grows north. */
typedef enum {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3,
} maze_dir_t;

typedef struct {
    uint8_t w;
    uint8_t h;
    uint8_t walls[MAZE_MAX_SIZE][MAZE_MAX_SIZE]; /* bit d set = wall on side d */
    uint8_t dist[MAZE_MAX_SIZE][MAZE_MAX_SIZE];  /* flood-fill distance to goal */
} maze_t;

/* Empty maze of w x h cells with the outer boundary walled. */
void maze_init(maze_t *m, uint8_t w, uint8_t h);

/* Record (or clear) the wall on side d of cell (x,y); the neighbour side is kept in sync.
 * Boundary walls are permanent. */
void maze_set_wall(maze_t *m, int x, int y, maze_dir_t d, bool present);
bool maze_has_wall(const maze_t *m, int x, int y, maze_dir_t d);

/* BFS from the goal; unknown walls count as open. */
void    maze_flood(maze_t *m, int goal_x, int goal_y);
uint8_t maze_dist(const maze_t *m, int x, int y);

/* Open neighbour with the lowest flood value, preferring straight, left, right, back.
 * Returns the direction, or -1 if there is no neighbour closer to the goal. */
int maze_next_dir(const maze_t *m, int x, int y, maze_dir_t heading);

/* Direction after quarter_turns clockwise quarter turns (negative = counter-clockwise). */
maze_dir_t maze_turn_dir(maze_dir_t d, int quarter_turns);

/* Move (x,y) one cell in direction d. */
void maze_step(maze_dir_t d, int *x, int *y);
