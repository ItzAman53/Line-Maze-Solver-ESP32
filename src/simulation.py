import pybullet as p
import pybullet_data
import numpy as np
import time
from collections import deque

initialx,initialy=0,0


# PyBullet Initialization
p.connect(p.GUI, options="--opengl3")
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.resetDebugVisualizerCamera(                                   #camera settings
    cameraDistance=3, 
    cameraYaw=90, 
    cameraPitch=-45,  
    cameraTargetPosition=[2, 2, 0]  
)
time.sleep(1)                                                 # Give 1sec for objects to be created




# Define Maze Parameters
size = 8  
wall_thickness = 0.05 
cell_size = 1.0    
goal = [(3, 4), (4, 4), (4, 3), (3, 3)]                      # Goal positions
  # Cost map (all infinte)
walls = np.zeros((size, size, 4), dtype=np.uint8)            # Wall storage (all zeros)
directions = [(1,0),(0,-1),(-1,0),(0,1)]             # (Right, Down, Left, Up)



# Add Ground Plane
plane = p.loadURDF("plane.urdf")



# Manually create an open maze with a single path to the goal
walls[7,7,0]=1 # Block right movement at (7,7)
walls[7,7,3]=1 # Block up movement at (7,7)
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



wall_ids=[]


# Function to create walls in PyBullet
def create_wall(x, y, Direction):
    """Creates a wall at grid cell (x, y) in the specified direction."""
    
    wall_thickness = 0.1  # Adjust thickness
    wall_height = 1.0  # Increase if needed

    # Define correct half extents based on direction
    if Direction in {0,2}:  # Right & Left walls
        half_extents = [wall_thickness, cell_size / 2, wall_height]
    else:  # Up & Down walls
        half_extents = [cell_size / 2, wall_thickness, wall_height]

    # Create collision and visual shapes
    wall_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    wall_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=[1, 0, 0, 1])

    # Adjust position based on direction
    if Direction == 0:  # Right wall
        pos = [(x + 1) * cell_size, (y + 0.5) * cell_size, wall_height / 2]
    elif Direction == 1:  # Down wall
        pos = [(x + 0.5) * cell_size, y * cell_size, wall_height / 2]
    elif Direction == 2:  # Left wall
        pos = [x * cell_size, (y + 0.5) * cell_size, wall_height / 2]
    elif Direction == 3:  # Up wall
        pos = [(x + 0.5) * cell_size, (y + 1) * cell_size, wall_height / 2]

    # Create the wall in PyBullet
    wall_id = p.createMultiBody(
        baseMass=0,  # Static wall (no physics movement)
        baseCollisionShapeIndex=wall_collision_shape,
        baseVisualShapeIndex=wall_visual_shape,
        basePosition=pos
    )

    # Ensure collision works properly
    p.changeDynamics(wall_id, -1, collisionMargin=0.01)
    
    return wall_id  # Return wall ID





# Add walls to simulation
for x in range(size):
    for y in range(size):
        for Direction in range(4):
            if walls[x, y, Direction] == 1:
                wall_id = create_wall(x, y, Direction)
                wall_ids.append(wall_id)  # Store wall ID



# Spawn Micromouse Robot
robot_start_pos = [0.5, 0.5, 0.1]  # Starting position
robot_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.1])
robot_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.1], rgbaColor=[0, 0, 1, 1])
robot = p.createMultiBody(
    baseMass=1, 
    baseCollisionShapeIndex=robot_shape, 
    baseVisualShapeIndex=robot_visual_shape,
    basePosition=robot_start_pos
)
p.setRealTimeSimulation(1) 
p.setGravity(0, 0, -9.8)  # Ensure gravity is set
p.setPhysicsEngineParameter(enableFileCaching=0)  # Disable file caching to avoid old settings
for _ in range(100):  # Run some steps
    p.stepSimulation()
    time.sleep(0.01)

print("Objects after simulation start:", p.getNumBodies())
for wall in wall_ids:
    p.setCollisionFilterPair(robot, wall, -1, -1, enableCollision=True)


def set_top_down_view():
    """Sets PyBullet camera to a fixed top-down view covering the whole maze."""
    cam_distance = size * 0.7  # Adjust distance for full maze view
    cam_yaw = 90  # Rotation angle
    cam_pitch = -90  # Top-down perspective
    cam_target = [size / 2, size / 2, 0]  # Center of maze

    p.resetDebugVisualizerCamera(cameraDistance=cam_distance,
                                 cameraYaw=cam_yaw,
                                 cameraPitch=cam_pitch,
                                 cameraTargetPosition=cam_target)

set_top_down_view()

def move_robot_smoothly(robot, target_pos, steps=20, delay=0.05):
    """Moves the robot smoothly to target position in small increments."""
    current_pos, _ = p.getBasePositionAndOrientation(robot)
    
    for i in range(1, steps + 1):
        # Interpolate position
        new_x = current_pos[0] + (target_pos[0] - current_pos[0]) * (i / steps)
        new_y = current_pos[1] + (target_pos[1] - current_pos[1]) * (i / steps)
        p.resetBasePositionAndOrientation(robot, [new_x, new_y, 0.1], [0, 0, 0, 1])
        time.sleep(delay)


visited_nodes = set()

def best_move(x, y, stack):
    possible_moves = []
    for i, (dx, dy) in enumerate(directions):
        nx, ny = x + dx, y + dy

        # Check bounds and walls
        if 0 <= nx < size and 0 <= ny < size and walls[x, y, i] == 0:
            if (nx, ny) not in visited_nodes:
                possible_moves.append((nx, ny, (dx, dy)))

    if possible_moves:
        # ✅ Select last-added move (DFS)
        nx, ny, direction = possible_moves[-1]
        visited_nodes.add((nx, ny))     # Mark as visited
        stack.append((x, y))            # Push current position for backtracking
        return direction, (nx, ny)

    # ❌ No valid moves → **Backtrack**
    if stack:
        prev_x, prev_y = stack.pop()
        return None, (prev_x, prev_y)

    return None, None  # No more moves, exploration finished



def detect_walls(robot, x, y, walls, size, cell_size=1.0):
    """Uses raycasting to detect walls and update the walls array dynamically, with debug lines."""
    sensor_range = 0.8
    directions1 = [(1,0),(0,-1),(-1,0),(0,1)]  # Right, Down, Left, Up

    robot_pos, _ = p.getBasePositionAndOrientation(robot)

    for i, (dx, dy) in enumerate(directions1):
        ray_start = [robot_pos[0], robot_pos[1], 0.5]  
        ray_end = [robot_pos[0] + dx * sensor_range, robot_pos[1] + dy * sensor_range, 0.5]
        hit_info = p.rayTest(ray_start, ray_end)[0]  

        walls[x, y, i] = 1 if hit_info[0] != -1 else 0  # ✅ Directly update walls[x, y, i]

        color = [1, 0, 0] if hit_info[0] != -1 else [0, 1, 0]  # Red for walls, Green otherwise
        

    

MAX_ITERATIONS = 100  # Prevents infinite looping

def exploration():
    """Micromouse explores the maze using DFS with backtracking, mapping walls dynamically."""
    visited = set()
    stack = []  # Stack for DFS and backtracking
    x, y = initialx,initialy  # Start position
    spawn_pos = [(x + 0.5) * cell_size, (y + 0.5) * cell_size, 0.1]
    p.resetBasePositionAndOrientation(robot, spawn_pos, [0, 0, 0, 1])
    print(f"Robot spawned at ({x}, {y})")

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

def set_top_down_view():
    """Sets PyBullet camera to top-down view covering the whole maze."""
    p.resetDebugVisualizerCamera(
    cameraDistance=4,  # Reduce distance if too far
    cameraYaw=0,       # Doesn't matter for top-down view
    cameraPitch=-88,  # Use -89.99 instead of -90 to avoid rendering issues
    cameraTargetPosition=[4, 4, 0.5]  # Raise Z slightly to focus properly
)


set_top_down_view()
print("Starting Exploration Phase...")
exploration()
print(f"wall at 4,5,1 is  {walls[4,5,1]}")
if int(input())==1:
    final_run()

