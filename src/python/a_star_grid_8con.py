import matplotlib.pyplot as plt
import heapq
import random
import math

def heuristic(a, b):
    """
    Octile distance for 8-connected grids:
      h((r1, c1), (r2, c2)) = max(Δr, Δc) + (sqrt(2) - 1) * min(Δr, Δc)
    """
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    return max(dr, dc) + (math.sqrt(2) - 1) * min(dr, dc)

def get_neighbors_8(current, grid_size, obstacles):
    """
    Return up to 8 neighbors of 'current' in an 8-connected grid,
    skipping out-of-bounds and obstacles, and preventing corner cutting.
    """
    rows, cols = grid_size
    (r, c) = current
    neighbors = []
    
    # Possible relative moves (including diagonals).
    # dx, dy pairs for 8 directions:
    deltas = [
        (-1, 0), (1, 0),  (0, -1), (0, 1),   # Up, Down, Left, Right
        (-1, -1), (-1, 1), (1, -1), (1, 1)   # Diagonals: NW, NE, SW, SE
    ]
    
    for dx, dy in deltas:
        nr, nc = r + dx, c + dy
        
        # Check bounds first
        if not (0 <= nr < rows and 0 <= nc < cols):
            continue
        
        # Check if obstacle
        if (nr, nc) in obstacles:
            continue
        
        neighbors.append((nr, nc))
    # print("neighbors = ", neighbors)

    for dx, dy in deltas[4:]:  # Diagonal moves
        nr, nc = r + dx, c + dy
        # Check if diagonal move is valid (not cutting corners)
        if (nr, c) in obstacles or (r, nc) in obstacles:
            # print(f"Removing corner-cutting neighbor: {(nr, nc)}")
            if (nr, nc) in neighbors:
                neighbors.remove((nr, nc))

    # print("final neighbors = ", neighbors)
    return neighbors

def move_cost(current, neighbor):
    """
    Return cost of moving from 'current' to 'neighbor' in an 8-connected grid:
      = 1    if they're horizontally/vertically adjacent
      = sqrt(2)  if they're diagonally adjacent
    """
    (r1, c1) = current
    (r2, c2) = neighbor
    # Diagonal check: if row & col both changed => sqrt(2), else => 1
    if abs(r1 - r2) == 1 and abs(c1 - c2) == 1:
        return math.sqrt(2)
    else:
        return 1

def generate_random_obstacles(grid_size, num_obstacles, start, goal):
    rows, cols = grid_size
    obstacles = set()
    while len(obstacles) < num_obstacles:
        obstacle = (random.randint(0, rows - 1), random.randint(0, cols - 1))
        if obstacle != start and obstacle != goal:
            obstacles.add(obstacle)
    return obstacles

# Example usage
grid_size = (20, 20)
num_obstacles = grid_size[0]*3  # Adjust the number of obstacles as needed
start = (0, 0)
goal = (grid_size[0]-1, grid_size[0]-1)
obstacles = generate_random_obstacles(grid_size, num_obstacles, start, goal)

def a_star(start, goal):
    open_list = []
    heapq.heappush(open_list, (0, start))
    closed_list = set()
    
    g = {start: 0}
    f = {start: heuristic(start, goal)}
    came_from = {}
    
    while open_list:
        # Pop the cell with the smallest f-score
        current = heapq.heappop(open_list)[1]
        
        # If we reached the goal, reconstruct the path
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        
        closed_list.add(current)
        
        # Explore 8-connected neighbors
        for nb in get_neighbors_8(current, grid_size, obstacles):
            if nb in closed_list:
                continue
            
            # Actual cost from 'start' to 'nb' via 'current'
            tentative_g = g[current] + move_cost(current, nb)
            
            if tentative_g < g.get(nb, float('inf')):
                came_from[nb] = current
                g[nb] = tentative_g
                f[nb] = tentative_g + heuristic(nb, goal)
                # If not already in open_list, push
                # Or if better path found, push again; the old entry is superseded
                heapq.heappush(open_list, (f[nb], nb))
    
    return None  # No path found


def visualize(grid_size, obstacles, path, start, goal):
    grid = [[' ' for _ in range(grid_size[1])] for _ in range(grid_size[0])]

    # Mark obstacles
    for obs in obstacles:
        grid[obs[0]][obs[1]] = 'X'

    # Mark path
    if path:
        for step in path:
            if step != start and step != goal:
                grid[step[0]][step[1]] = '.'

    # Mark start and goal
    grid[start[0]][start[1]] = 'S'
    grid[goal[0]][goal[1]] = 'G'

    # Plot the grid
    fig, ax = plt.subplots()
    ax.set_xticks(range(grid_size[1] + 1))
    ax.set_yticks(range(grid_size[0] + 1))
    ax.grid(color='black', linestyle='-', linewidth=1)

    for i in range(grid_size[0]):
        for j in range(grid_size[1]):
            if grid[i][j] == 'X':
                ax.add_patch(plt.Rectangle((j, i), 1, 1, color='black'))
            elif grid[i][j] == '.':
                ax.add_patch(plt.Rectangle((j, i), 1, 1, color='blue'))
            elif grid[i][j] == 'S':
                ax.add_patch(plt.Rectangle((j, i), 1, 1, color='green'))
            elif grid[i][j] == 'G':
                ax.add_patch(plt.Rectangle((j, i), 1, 1, color='red'))

    ax.set_xlim(0, grid_size[1])
    ax.set_ylim(0, grid_size[0])
    ax.set_aspect('equal')

    # Add legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='black', edgecolor='black', label='Obstacle'),
        Patch(facecolor='blue', edgecolor='black', label='Path'),
        Patch(facecolor='green', edgecolor='black', label='Start'),
        Patch(facecolor='red', edgecolor='black', label='Goal')
    ]
    ax.legend(handles=legend_elements, loc='upper right')

    plt.show()

path = a_star(start, goal)
print(path)
visualize(grid_size, obstacles, path, start, goal)
