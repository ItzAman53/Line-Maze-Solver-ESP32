size = 8
walls = bytearray(size * size * 4)  # 4 for 4 directions
directions = [(0, -1), (-1, 0), (0, 1), (1, 0)]  # left, up, right, down

# Changing and accessing walls array
def set_wall(walls, row, col, depth, value):
    index = (row * size * 4) + (col * 4) + depth
    walls[index] = value

def get_wall(walls, row, col, depth):
    index = (row * size * 4) + (col * 4) + depth
    return walls[index]




visited_nodes = bytearray(size * size)

def best_move(x, y, stack):
    possible_moves = []
    
    for i in range(4):  # Iterate over 4 directions
        dx, dy = directions[i]
        nx, ny = x + dx, y + dy

        # Bounds & walls check
        if 0 <= nx < size and 0 <= ny < size and get_wall(walls, x, y, i) == 0:
            if visited_nodes[(nx * size) + ny] == 0:  # Check if not visited  
                possible_moves.append((nx, ny, dx, dy))
                
    if possible_moves:
        # ✅ Select last-added move (DFS)
        nx, ny, dx, dy = possible_moves[-1]
        visited_nodes[(nx * size) + ny] = 1  # Mark as visited
        stack.append((x, y))  # Push current position for backtracking
        return dx, dy, nx, ny  # Return movement direction & new position

    # No valid moves → **Backtrack**
    if stack:
        prev_x, prev_y = stack.pop()
        return None, None, prev_x, prev_y  # Backtrack to previous cell

    return None, None, None, None  # No moves left (exploration finished)




