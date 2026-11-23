// SRCS: main/app/maze.c main/app/path.c
#include "maze.h"
#include "path.h"
#include "test.h"

static const maze_cell_t GOAL_22[] = { {2, 2} };
static const maze_cell_t GOAL_20[] = { {2, 0} };

static void visit(maze_t *m, const maze_cell_t *cells, int n)
{
    for (int i = 0; i < n; i++) {
        maze_mark_visited(m, cells[i].x, cells[i].y);
    }
}

static void visit_all(maze_t *m)
{
    for (int x = 0; x < m->w; x++) {
        for (int y = 0; y < m->h; y++) {
            maze_mark_visited(m, x, y);
        }
    }
}

static bool move_is(const path_move_t *mv, move_type_t type, int cells, int angle)
{
    return mv->type == type && mv->cells == cells && mv->angle_deg == angle;
}

/* 3x3, corridor (0,0) -> (0,2) -> (2,2), everything else walled off. */
static void l_corridor(maze_t *m)
{
    maze_init(m, 3, 3, GOAL_22, 1);
    maze_set_wall(m, 0, 0, DIR_EAST, true);
    maze_set_wall(m, 0, 1, DIR_EAST, true);
    maze_set_wall(m, 1, 2, DIR_SOUTH, true);
    maze_set_wall(m, 2, 2, DIR_SOUTH, true);
    visit_all(m);
}

static void test_l_corridor(void)
{
    maze_t      m;
    path_move_t mv[8];

    l_corridor(&m);
    CHECK(path_plan(&m, 0, 0, DIR_NORTH, mv, 8) == 3);
    CHECK(move_is(&mv[0], MOVE_STRAIGHT, 2, 0));
    CHECK(move_is(&mv[1], MOVE_TURN, 0, -90));
    CHECK(move_is(&mv[2], MOVE_STRAIGHT, 2, 0));
}

static void test_turn_180_first(void)
{
    maze_t      m;
    path_move_t mv[8];

    l_corridor(&m);
    CHECK(path_plan(&m, 0, 0, DIR_SOUTH, mv, 8) == 4);
    CHECK(move_is(&mv[0], MOVE_TURN, 0, 180));
    CHECK(move_is(&mv[1], MOVE_STRAIGHT, 2, 0));
}

/* 3x3 without inner walls; only the U (0,0)->(0,2)->(2,2)->(2,0) was visited. The short
 * route through unvisited (1,0) must not be used. */
static void test_avoids_unvisited_shortcut(void)
{
    static const maze_cell_t u[] = { {0, 0}, {0, 1}, {0, 2}, {1, 2}, {2, 2}, {2, 1}, {2, 0} };
    maze_t      m;
    path_move_t mv[8];

    maze_init(&m, 3, 3, GOAL_20, 1);
    visit(&m, u, 7);
    CHECK(path_plan(&m, 0, 0, DIR_NORTH, mv, 8) == 5);
    CHECK(move_is(&mv[0], MOVE_STRAIGHT, 2, 0));
    CHECK(move_is(&mv[1], MOVE_TURN, 0, -90));
    CHECK(move_is(&mv[2], MOVE_STRAIGHT, 2, 0));
    CHECK(move_is(&mv[3], MOVE_TURN, 0, -90));
    CHECK(move_is(&mv[4], MOVE_STRAIGHT, 2, 0));
}

static void test_unreachable_goal(void)
{
    static const maze_cell_t part[] = { {0, 0}, {0, 1}, {0, 2}, {1, 2}, {2, 2}, {2, 0} };
    maze_t      m;
    path_move_t mv[8];

    maze_init(&m, 3, 3, GOAL_20, 1);
    visit(&m, part, 6);
    CHECK(path_plan(&m, 0, 0, DIR_NORTH, mv, 8) == -1);
}

int main(void)
{
    test_l_corridor();
    test_turn_180_first();
    test_avoids_unvisited_shortcut();
    test_unreachable_goal();
    return test_result();
}
