#pragma once

/* Starts the maze-solving task: waits for a button press, then explores to the goal
 * with flood fill. Repeats after each run (reposition the robot at the start first). */
void navigation_task_start(void);
