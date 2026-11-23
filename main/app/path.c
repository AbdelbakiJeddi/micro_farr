#include "path.h"

#include <string.h>

#define NO_PARENT 0xFF

typedef struct {
    int         count;
    int         max;
    path_move_t *out;
} move_list_t;

static bool can_step(const maze_t *m, int x, int y, maze_dir_t d, int *nx, int *ny)
{
    *nx = x;
    *ny = y;
    maze_step(d, nx, ny);
    return !maze_has_wall(m, x, y, d) && maze_visited(m, *nx, *ny);
}

/* BFS from the start through visited cells. Fills parent_dir (direction taken to enter each
 * cell) and returns the nearest goal reached as x * MAZE_MAX_SIZE + y, or -1. */
static int bfs_to_goal(const maze_t *m, int sx, int sy,
                       uint8_t parent_dir[MAZE_MAX_SIZE][MAZE_MAX_SIZE])
{
    uint16_t queue[MAZE_MAX_SIZE * MAZE_MAX_SIZE];
    bool     seen[MAZE_MAX_SIZE][MAZE_MAX_SIZE] = { { false } };
    int      head = 0;
    int      tail = 0;

    memset(parent_dir, NO_PARENT, MAZE_MAX_SIZE * MAZE_MAX_SIZE);
    seen[sx][sy]  = true;
    queue[tail++] = (uint16_t)(sx * MAZE_MAX_SIZE + sy);

    while (head < tail) {
        int x = queue[head] / MAZE_MAX_SIZE;
        int y = queue[head] % MAZE_MAX_SIZE;
        head++;

        if (maze_is_goal(m, x, y)) {
            return x * MAZE_MAX_SIZE + y;
        }
        for (int d = 0; d < 4; d++) {
            int nx, ny;
            if (!can_step(m, x, y, (maze_dir_t)d, &nx, &ny) || seen[nx][ny]) {
                continue;
            }
            seen[nx][ny]       = true;
            parent_dir[nx][ny] = (uint8_t)d;
            queue[tail++]      = (uint16_t)(nx * MAZE_MAX_SIZE + ny);
        }
    }
    return -1;
}

/* Directions from the start to (gx,gy), written in path order; returns their count. */
static int trace_dirs(const uint8_t parent_dir[MAZE_MAX_SIZE][MAZE_MAX_SIZE],
                      int gx, int gy, maze_dir_t *dirs)
{
    int n = 0;
    int x = gx;
    int y = gy;

    while (parent_dir[x][y] != NO_PARENT) {
        maze_dir_t d = (maze_dir_t)parent_dir[x][y];
        dirs[n++] = d;
        maze_step(maze_turn_dir(d, 2), &x, &y);
    }
    for (int i = 0; i < n / 2; i++) {
        maze_dir_t t   = dirs[i];
        dirs[i]         = dirs[n - 1 - i];
        dirs[n - 1 - i] = t;
    }
    return n;
}

static bool push(move_list_t *l, move_type_t type, int cells, int angle)
{
    if (l->count >= l->max) {
        return false;
    }
    l->out[l->count++] = (path_move_t){ .type = type, .cells = (int16_t)cells,
                                        .angle_deg = (int16_t)angle };
    return true;
}

/* Angle of the pivot from heading to d: +90 left, -90 right, 180. */
static int turn_angle(maze_dir_t heading, maze_dir_t d)
{
    static const int16_t ANGLE[4] = { 0, -90, 180, 90 };
    return ANGLE[((int)d - (int)heading + 4) & 3];
}

/* Compress cell-by-cell directions into straight runs and pivots. */
static int compress(const maze_dir_t *dirs, int n, maze_dir_t heading, move_list_t *l)
{
    int run = 0;

    for (int i = 0; i < n; i++) {
        if (dirs[i] != heading) {
            if ((run > 0 && !push(l, MOVE_STRAIGHT, run, 0)) ||
                !push(l, MOVE_TURN, 0, turn_angle(heading, dirs[i]))) {
                return -1;
            }
            run     = 0;
            heading = dirs[i];
        }
        run++;
    }
    if (run > 0 && !push(l, MOVE_STRAIGHT, run, 0)) {
        return -1;
    }
    return l->count;
}

int path_plan(const maze_t *m, int sx, int sy, maze_dir_t sdir,
              path_move_t *out, int max_moves)
{
    uint8_t     parent_dir[MAZE_MAX_SIZE][MAZE_MAX_SIZE];
    maze_dir_t  dirs[MAZE_MAX_SIZE * MAZE_MAX_SIZE];
    move_list_t list = { .count = 0, .max = max_moves, .out = out };

    if (!maze_contains(m, sx, sy)) {
        return -1;
    }
    int goal = bfs_to_goal(m, sx, sy, parent_dir);
    if (goal < 0) {
        return -1;
    }
    int n = trace_dirs(parent_dir, goal / MAZE_MAX_SIZE, goal % MAZE_MAX_SIZE, dirs);
    return compress(dirs, n, sdir, &list);
}
