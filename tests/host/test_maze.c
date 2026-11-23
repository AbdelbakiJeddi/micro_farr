// SRCS: main/app/maze.c
#include "maze.h"
#include "test.h"

static const maze_cell_t GOALS4[] = { {1, 1}, {1, 2}, {2, 1}, {2, 2} };

static void test_flood_seeds_every_goal(void)
{
    maze_t m;

    maze_init(&m, 4, 4, GOALS4, 4);
    maze_flood(&m);
    for (int i = 0; i < 4; i++) {
        CHECK(maze_dist(&m, GOALS4[i].x, GOALS4[i].y) == 0);
    }
    CHECK(maze_dist(&m, 0, 0) == 2);
}

static void test_is_goal(void)
{
    maze_t m;

    maze_init(&m, 4, 4, GOALS4, 4);
    CHECK(maze_is_goal(&m, 2, 2));
    CHECK(!maze_is_goal(&m, 0, 0));
}

static void test_visited(void)
{
    maze_t m;

    maze_init(&m, 4, 4, GOALS4, 4);
    CHECK(!maze_visited(&m, 3, 0));
    maze_mark_visited(&m, 3, 0);
    CHECK(maze_visited(&m, 3, 0));
    CHECK(!maze_visited(&m, 0, 3));
}

int main(void)
{
    test_flood_seeds_every_goal();
    test_is_goal();
    test_visited();
    return test_result();
}
