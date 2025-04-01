import pybullet as p
import pybullet_data
import numpy as np
import time
from collections import deque

initialx,initialy=7,0

size = 8  
wall_thickness = 0.05 
cell_size = 1.0
infinite = 255     
goal = [(3, 4), (4, 4), (4, 3), (3, 3)]                      # Goal positions
cost_map = np.full((size, size), infinite, dtype=np.uint8)   # Cost map (all infinte)
walls = np.zeros((size, size, 4), dtype=np.uint8)            # Wall storage (all zeros)
directions = [(1,0),(0,-1),(-1,0),(0,1)]             # (Right, Down, Left, Up)

walls[7,7,0]=1 # Block right movement at (7,7)
walls[7,7,3]=1 # Block up movement at    (7,7)
walls[7,6,0]=1 # Block right movement at (7,6)
walls[7,5,0]=1 # Block right movement at (7,5)
walls[7,4,0]=1 # Block right movement at (7,4)
walls[7,3,0]=1 # Block right movement at (7,3)
walls[7,2,0]=1 # Block right movement at (7,2)
walls[7,1,0]=1 # Block right movement at (7,1)
walls[7,0,0]=1 # Block right movement at (7,0)
walls[7,0,1]=1 # Block down movement at (7,0)
walls[6,0,1]=1 # Block down movement at (6,0)
walls[5,0,1]=1 # Block down movement at (5,0)
walls[4,0,1]=1 # Block down movement at (4,0)
walls[3,0,1]=1 # Block down movement at (3,0)
walls[2,0,1]=1 # Block down movement at (2,0)
walls[1,0,1]=1 # Block down movement at (1,0)
walls[0,0,1]=1 # Block down movement at (0,0)
walls[0,0,2]=1 # Block left movement at (0,0)
walls[0,1,2]=1 # Block left movement at (0,1)
walls[0,2,2]=1 # Block left movement at (0,2)
walls[0,3,2]=1 # Block left movement at (0,3)
walls[0,4,2]=1 # Block left movement at (0,4)   
walls[0,5,2]=1 # Block left movement at (0,5)
walls[0,6,2]=1 # Block left movement at (0,6)
walls[0,7,2]=1 # Block left movement at (0,7)
walls[0,7,3]=1 # Block up movement at (0,7)
walls[1,7,3]=1 # Block up movement at (1,7)
walls[2,7,3]=1 # Block up movement at (2,7)
walls[3,7,3]=1 # Block up movement at (3,7)
walls[4,7,3]=1 # Block up movement at (4,7)
walls[5,7,3]=1 # Block up movement at (5,7)
walls[6,7,3]=1 # Block up movement at (6,7)
walls[7,7,3]=1 # Block up movement at (7,7)

walls[3,3,2]=1 # Block left movement at (3,3)
walls[3,3,1]=1 # Block down movement at (3,3)
walls[4,3,0]=1 # Block right movement at (4,3)
walls[4,3,1]=1 # Block down movement at (4,3)
walls[4,4,3]=1 # Block up movement at (4,4)
walls[3,4,2]=1 # Block left movement at (3,4)
walls[3,4,3]=1 # Block up movement at (3,4)


walls[5,6,0]=1 # Block right movement at (5,6)
walls[5,6,3]=1 # Block up movement at (5,6)
walls[6,6,0]=1 # Block right movement at (6,6)
walls[2,5,1]=1 # Block down movement at (2,5)
walls[2,5,2]=1 # Block left movement at (2,5)
walls[5,2,2]=1 # Block left movement at (5,2)
walls[5,2,3]=1 # Block up movement at (5,2)
walls[6,2,0]=1 # Block right movement at (6,2)
walls[6,2,3]=1 # Block up movement at (6,2)
walls[1,4,2]=1 # Block left movement at (1,4)
walls[1,4,1]=1 # Block down movement at (1,4)
walls[1,2,0]=1 # Block right movement at (1,2)
walls[1,2,3]=1 # Block up movement at (1,2)
walls[2,1,0]=1 # Block right movement at (2,1)
walls[2,1,1]=1 # Block down movement at (2,1)
walls[4,5,0]=1 # Block right movement at (4,5)
walls[4,5,3]=1 # Block up movement at (4,5)
walls[5,5,0]=1 # Block right movement at (5,5)
walls[5,5,1]=1 # Block down movement at (5,5)
walls[0,6,0]=1 # Block right movement at (0,6)
walls[0,5,0]=1 # Block right movement at (0,5)
walls[0,4,0]=1 # Block right movement at (0,4)
walls[0,3,0]=1 # Block right movement at (0,3)
walls[0,2,0]=1 # Block right movement at (0,2)
walls[0,1,0]=1 # Block right movement at (0,1)
walls[0,0,0]=1 # Block right movement at (0,0)
walls[7,0,3]=1 # Block up movement at (7,0)
walls[6,0,3]=1 # Block up movement at (6,0)
walls[5,0,3]=1 # Block up movement at (5,0)
walls[4,0,3]=1 # Block up movement at (4,0)
walls[3,0,3]=1 # Block up movement at (3,0)



visited_nodes = set()


def best_move(x, y, stack):
    """Decide the next move for exploration, using DFS with backtracking."""
    possible_moves = []

    print(f"\n🔍 Checking moves from ({x}, {y})")

    for i, (dx, dy) in enumerate(directions):
        nx, ny = x + dx, y + dy
        print(f"👉 Checking ({nx}, {ny})")

        # ✅ Check if the move is within bounds
        if 0 <= nx < size and 0 <= ny < size:
            # ❌ Stop if there's a wall in that direction
            if walls[x, y, i] == 1:
                print(f"🚧 Wall detected at ({nx}, {ny}), skipping")
                continue

            # ✅ Prioritize unvisited nodes
            if (nx, ny) not in visited_nodes:
                possible_moves.append((nx, ny, (dx, dy)))
                print(f"✅ Added ({nx}, {ny}) to possible moves")

    if possible_moves:
        # ✅ Pick the last move (DFS-style)
        next_move = possible_moves[-1]
        best_direction = next_move[2]
        visited_nodes.add((next_move[0], next_move[1]))  # Mark as visited
        stack.append((x, y))  # Push current position to stack (for backtracking)
        print(f"🟢 Moving to {next_move[0]}, {next_move[1]} via {best_direction}")
        return best_direction, (next_move[0], next_move[1])

    # ❌ No valid moves → **Backtrack**
    if stack:
        prev_x, prev_y = stack.pop()  # Pop last visited cell
        print(f"🔙 Backtracking to ({prev_x}, {prev_y})")
        return None, (prev_x, prev_y)  # Return to the previous cell

    print(f"❌ Exploration complete. No more moves left.")
    return None, None  # No more moves, exploration finished



def detect_walls(robot, x, y, walls, size, cell_size=1.0):
    """Uses raycasting to detect walls and update the walls array dynamically, with debug lines."""
    sensor_range = 0.8
    directions1 = [(1,0),(0,-1),(-1,0),(0,1)]  # Right, Down, Left, Up
    direction1_names = ["Right", "Left", "Up", "Down"]  # For debugging output

    robot_pos, _ = p.getBasePositionAndOrientation(robot)

    for i, (dx, dy) in enumerate(directions1):
        ray_start = [robot_pos[0], robot_pos[1], 0.5]  
        ray_end = [robot_pos[0] + dx * sensor_range, robot_pos[1] + dy * sensor_range, 0.5]
        hit_info = p.rayTest(ray_start, ray_end)[0]  

        walls[x, y, i] = 1 if hit_info[0] != -1 else 0  # ✅ Directly update walls[x, y, i]

        color = [1, 0, 0] if hit_info[0] != -1 else [0, 1, 0]  # Red for walls, Green otherwise
        p.addUserDebugLine(ray_start, ray_end, color, 2)

    






MAX_ITERATIONS = 100  # Prevents infinite looping

def exploration():
    """Micromouse explores the maze using DFS with backtracking, mapping walls dynamically."""
    visited = set()
    stack = []  # Stack for DFS and backtracking
    x, y = initialx,initialy  # Start position
    spawn_pos = [(x + 0.5) * cell_size, (y + 0.5) * cell_size, 0.1]
    p.resetBasePositionAndOrientation(robot, spawn_pos, [0, 0, 0, 1])
    print(f"✅ Robot spawned at ({x}, {y})")

    iterations = 0

    while iterations < MAX_ITERATIONS:
        iterations += 1
        visited.add((x, y))

        detect_walls(robot, x, y, walls, size)  
        

        move, new_pos = best_move(x, y, stack)

        if new_pos is None:
            print("✅ Exploration Complete! All reachable cells visited.")
            break  # Stop loop when all cells are explored

        if move is None:
            # 🔙 Backtracking case: move to previous position
            x, y = new_pos  # Update position
            new_pos = [(x + 0.5) * cell_size, (y + 0.5) * cell_size, 0.1]
            move_robot_smoothly(robot, new_pos, steps=10, delay=0.0001)
            print(f"🔙 Backtracking to ({x}, {y})")
            continue  # Continue exploration

        # ✅ Normal move case
        dx, dy = move  
        x, y = new_pos  # Update robot's position

        new_pos = [(x + 0.5) * cell_size, (y + 0.5) * cell_size, 0.1]
        move_robot_smoothly(robot, new_pos, steps=10, delay=0.0001)
        time.sleep(0.1)

    print("✅ Exploration Finished! Returning to Start Position...")
    return_to_start(x, y)


def return_to_start(x, y):
    """Moves the robot back to (5,7) using BFS."""
    start_position = (initialx,initialy)  # Change this if your initial position is different
    path = bfs_shortest_path((x, y), {start_position})  # Use BFS to get shortest path

    if not path:
        print("❌ No path found back to start!")
        return

    for (nx, ny) in path:
        new_pos = [(nx + 0.5) * cell_size, (ny + 0.5) * cell_size, 0.1]
        move_robot_smoothly(robot, new_pos, steps=10, delay=0.0001)
        time.sleep(0.1)

    print("🏁✅ Robot returned to start position!")





def final_run():
    """Micromouse follows shortest path from start to goal."""
    x, y = initialx,initialy
    path = bfs_shortest_path((x, y), goal)

    if not path:
        print("❌ No path found to goal!")
        return

    for (nx, ny) in path:
        new_pos = [(nx + 0.5) * cell_size, (ny + 0.5) * cell_size, 0.1]
        move_robot_smoothly(robot, new_pos, steps=10, delay=0.0001)
        time.sleep(0.1)

    print("🏆 Final run complete!")

def opp(i):
    if i==0:
        return 2
    if i==1:
        return 3
    if i==2:
        return 0
    if i==3:
        return 1
    
def bfs_shortest_path(start, goals):
    """Finds shortest path using BFS."""
    queue = deque([(start, [])])
    visited = set()

    while queue:
        (x, y), path = queue.popleft()

        if (x, y) in goals:
            return path

        visited.add((x, y))

        for i, (dx, dy) in enumerate(directions):
            nx, ny = x + dx, y + dy

            if 0 <= nx < size and 0 <= ny < size:
                if walls[x, y, i] == 0 and walls[nx,ny,opp(i)]==0 and (nx, ny) not in visited:
                    queue.append(((nx, ny), path + [(nx, ny)]))

    return None




print("Starting Exploration Phase...")
exploration()
print(f"wall at 4,5,1 is  {walls[4,5,1]}")
final_run()
