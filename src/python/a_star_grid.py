import matplotlib.pyplot as plt
import heapq
import random

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def nbs(node, grid_size):
    x, y = node
    return [(x + dx, y + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)] if 0 <= x + dx < grid_size[0] and 0 <= y + dy < grid_size[1]]

def generate_random_obstacles(grid_size, num_obstacles, start, goal):
    """
    Generate a set of random obstacles for an n x n grid.

    Args:
        grid_size (tuple): The size of the grid as (rows, cols).
        num_obstacles (int): The number of obstacles to generate.

    Returns:
        set: A set of tuples representing obstacle coordinates.
    """
    rows, cols = grid_size
    obstacles = set()

    while len(obstacles) < num_obstacles:
        obstacle = (random.randint(0, rows - 1), random.randint(0, cols - 1))
        if obstacle != start and obstacle != goal:
            obstacles.add(obstacle)

    return obstacles

# Example usage
grid_size = (20, 20)
num_obstacles = 50  # Adjust the number of obstacles as needed
start = (0, 0)
goal = (19, 19)
obstacles = generate_random_obstacles(grid_size, num_obstacles, start, goal)

def a_star(start, goal):
    open_list = []
    heapq.heappush(open_list, (0, start))
    closed_list = set()
    g = {start: 0}
    f = {start: heuristic(start, goal)}
    came_from = {}

    while open_list:
        current = heapq.heappop(open_list)[1]
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        closed_list.add(current)

        for nb in nbs(current, grid_size):
            if nb in closed_list or nb in obstacles:
                continue
            tentative_g = g[current] + 1
            if tentative_g >= g.get(nb, float('inf')):
                continue
            came_from[nb] = current
            g[nb] = tentative_g
            f[nb] = g[nb] + heuristic(nb, goal)
            heapq.heappush(open_list, (f[nb], nb))

    return None

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


