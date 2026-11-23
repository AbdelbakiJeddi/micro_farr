// SRCS: main/app/maze.c main/app/maze_store.c -DMAZE_STORE_HOST
#include "maze.h"
#include "maze_store.h"
#include "test.h"

static const maze_cell_t GOAL_A[] = { {1, 1} };
static const maze_cell_t GOAL_B[] = { {2, 2} };

static void test_store_rejects_mismatch(void)
{
    maze_t saved, expected;

    maze_init(&expected, 4, 4, GOAL_A, 1);

    maze_init(&saved, 3, 4, GOAL_A, 1);
    CHECK(!maze_blob_valid(&saved, &expected));

    maze_init(&saved, 4, 4, GOAL_B, 1);
    CHECK(!maze_blob_valid(&saved, &expected));

    maze_init(&saved, 4, 4, GOAL_A, 1);
    maze_mark_visited(&saved, 0, 0);
    maze_set_wall(&saved, 0, 0, DIR_EAST, true);
    CHECK(maze_blob_valid(&saved, &expected));
}

int main(void)
{
    test_store_rejects_mismatch();
    return test_result();
}
