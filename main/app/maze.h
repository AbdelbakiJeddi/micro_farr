#pragma once

#include <stdbool.h>
#include <stdint.h>

#define MAZE_MAX_SIZE    16
#define MAZE_MAX_GOALS   8
#define MAZE_UNREACHABLE 255

/* Compass directions; x grows east, y grows north. */
typedef enum {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3,
} maze_dir_t;

typedef struct {
    uint8_t x;
    uint8_t y;
} maze_cell_t;

typedef struct {
    uint8_t     w;
    uint8_t     h;
    uint8_t     walls[MAZE_MAX_SIZE][MAZE_MAX_SIZE];   /* bit d set = wall on side d */
    uint8_t     dist[MAZE_MAX_SIZE][MAZE_MAX_SIZE];    /* flood-fill distance to the goal */
    uint8_t     visited[MAZE_MAX_SIZE][MAZE_MAX_SIZE]; /* 1 = robot has been in the cell */
    maze_cell_t goals[MAZE_MAX_GOALS];                 /* goal = group of cells */
    uint8_t     goal_count;
} maze_t;

/* Empty maze of w x h cells with the outer boundary walled; goal_count >= 1. */
void maze_init(maze_t *m, uint8_t w, uint8_t h, const maze_cell_t *goals, uint8_t goal_count);

/* maze_init with the size and goal cells from app_config.h. */
void maze_init_configured(maze_t *m);

bool maze_is_goal(const maze_t *m, int x, int y);
void maze_mark_visited(maze_t *m, int x, int y);
bool maze_visited(const maze_t *m, int x, int y);

/* Record (or clear) the wall on side d of cell (x,y); the neighbour side is kept in sync.
 * Boundary walls are permanent. */
void maze_set_wall(maze_t *m, int x, int y, maze_dir_t d, bool present);
bool maze_has_wall(const maze_t *m, int x, int y, maze_dir_t d);

/* Multi-source BFS from every goal cell; unknown walls count as open. */
void    maze_flood(maze_t *m);
uint8_t maze_dist(const maze_t *m, int x, int y);

/* Open neighbour with the lowest flood value, preferring straight, left, right, back.
 * Returns the direction, or -1 if there is no neighbour closer to the goal. */
int maze_next_dir(const maze_t *m, int x, int y, maze_dir_t heading);

/* Direction after quarter_turns clockwise quarter turns (negative = counter-clockwise). */
maze_dir_t maze_turn_dir(maze_dir_t d, int quarter_turns);

/* Move (x,y) one cell in direction d. */
void maze_step(maze_dir_t d, int *x, int *y);

/* True if (x,y) is inside the maze. */
bool maze_contains(const maze_t *m, int x, int y);
