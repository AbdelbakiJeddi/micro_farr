#include "maze.h"

#include <string.h>

#include "app_config.h"

static const int8_t DX[4] = {0, 1, 0, -1};
static const int8_t DY[4] = {1, 0, -1, 0};

/* Relative preference when costs tie: straight, left, right, back. */
static const int8_t PREFERENCE[4] = {0, -1, 1, 2};

bool maze_contains(const maze_t *m, int x, int y)
{
    return x >= 0 && y >= 0 && x < m->w && y < m->h;
}

static maze_dir_t opposite(maze_dir_t d)
{
    return maze_turn_dir(d, 2);
}

maze_dir_t maze_turn_dir(maze_dir_t d, int quarter_turns)
{
    return (maze_dir_t)(((int)d + quarter_turns + 4) & 3);
}

void maze_step(maze_dir_t d, int *x, int *y)
{
    *x += DX[d];
    *y += DY[d];
}

void maze_init(maze_t *m, uint8_t w, uint8_t h, const maze_cell_t *goals, uint8_t goal_count)
{
    memset(m, 0, sizeof(*m));
    m->w = w;
    m->h = h;

    if (goal_count > MAZE_MAX_GOALS) {
        goal_count = MAZE_MAX_GOALS;
    }
    memcpy(m->goals, goals, goal_count * sizeof(goals[0]));
    m->goal_count = goal_count;

    for (int x = 0; x < w; x++) {
        m->walls[x][h - 1] |= 1U << DIR_NORTH;
        m->walls[x][0]     |= 1U << DIR_SOUTH;
    }
    for (int y = 0; y < h; y++) {
        m->walls[0][y]     |= 1U << DIR_WEST;
        m->walls[w - 1][y] |= 1U << DIR_EAST;
    }
}

void maze_init_configured(maze_t *m)
{
    static const maze_cell_t goals[] = MAZE_GOAL_CELLS;

    maze_init(m, MAZE_W, MAZE_H, goals, (uint8_t)(sizeof(goals) / sizeof(goals[0])));
}

bool maze_is_goal(const maze_t *m, int x, int y)
{
    for (int i = 0; i < m->goal_count; i++) {
        if (m->goals[i].x == x && m->goals[i].y == y) {
            return true;
        }
    }
    return false;
}

void maze_mark_visited(maze_t *m, int x, int y)
{
    if (maze_contains(m, x, y)) {
        m->visited[x][y] = 1;
    }
}

bool maze_visited(const maze_t *m, int x, int y)
{
    return maze_contains(m, x, y) && m->visited[x][y] != 0;
}

bool maze_has_wall(const maze_t *m, int x, int y, maze_dir_t d)
{
    return (m->walls[x][y] & (1U << d)) != 0;
}

static void write_wall(maze_t *m, int x, int y, maze_dir_t d, bool present)
{
    if (present) {
        m->walls[x][y] |= (uint8_t)(1U << d);
    } else {
        m->walls[x][y] &= (uint8_t)~(1U << d);
    }
}

void maze_set_wall(maze_t *m, int x, int y, maze_dir_t d, bool present)
{
    int nx = x + DX[d];
    int ny = y + DY[d];

    if (!maze_contains(m, nx, ny)) {
        return; /* boundary wall: permanent */
    }

    write_wall(m, x, y, d, present);
    write_wall(m, nx, ny, opposite(d), present);
}

void maze_flood(maze_t *m)
{
    uint16_t queue[MAZE_MAX_SIZE * MAZE_MAX_SIZE];
    int head = 0;
    int tail = 0;

    memset(m->dist, MAZE_UNREACHABLE, sizeof(m->dist));
    for (int i = 0; i < m->goal_count; i++) {
        m->dist[m->goals[i].x][m->goals[i].y] = 0;
        queue[tail++] = (uint16_t)(m->goals[i].x * MAZE_MAX_SIZE + m->goals[i].y);
    }

    while (head < tail) {
        int x = queue[head] / MAZE_MAX_SIZE;
        int y = queue[head] % MAZE_MAX_SIZE;
        head++;

        for (int d = 0; d < 4; d++) {
            if (maze_has_wall(m, x, y, (maze_dir_t)d)) {
                continue;
            }
            int nx = x + DX[d];
            int ny = y + DY[d];
            if (!maze_contains(m, nx, ny) || m->dist[nx][ny] != MAZE_UNREACHABLE) {
                continue;
            }
            m->dist[nx][ny] = (uint8_t)(m->dist[x][y] + 1);
            queue[tail++] = (uint16_t)(nx * MAZE_MAX_SIZE + ny);
        }
    }
}

uint8_t maze_dist(const maze_t *m, int x, int y)
{
    return m->dist[x][y];
}

int maze_next_dir(const maze_t *m, int x, int y, maze_dir_t heading)
{
    int best     = -1;
    int best_val = m->dist[x][y];

    for (int k = 0; k < 4; k++) {
        maze_dir_t d = maze_turn_dir(heading, PREFERENCE[k]);
        if (maze_has_wall(m, x, y, d)) {
            continue;
        }
        int nx = x + DX[d];
        int ny = y + DY[d];
        if (!maze_contains(m, nx, ny)) {
            continue;
        }
        if (m->dist[nx][ny] < best_val) {
            best_val = m->dist[nx][ny];
            best     = d;
        }
    }
    return best;
}
