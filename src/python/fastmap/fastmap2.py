# Full fixed Python code to:
# 1. Generate a large octile map
# 2. Parse it
# 3. Run A* using Manhattan, Octile, and FastMap heuristics
# 4. Compare results

import random
import heapq
import time
import pandas as pd

# Directions for 8-connected grid (Octile)
DIRS_8 = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

# Step 1: Generate Octile-style map
def generate_octile_map(width, height, filename):
    symbols = ['@', 'T', '.', '#']
    weights = [0.3, 0.3, 0.3, 0.1]  # Probability of each terrain type
    map_data = []
    for _ in range(height):
        row = ''.join(random.choices(symbols, weights, k=width))
        map_data.append(row)
    with open(filename, 'w') as f:
        f.write(f"type octile\n")
        f.write(f"height {height}\n")
        f.write(f"width {width}\n")
        f.write(f"map\n")
        for row in map_data:
            f.write(row + "\n")
    return map_data

map_path = "generated_map.map"  # Save in the current directory
generate_octile_map(width=182, height=50, filename=map_path)


# Step 2: Load map
def load_octile_map(path):
    with open(path, 'r') as f:
        lines = f.readlines()
    header_end = lines.index('map\n')
    grid = [list(line.strip()) for line in lines[header_end + 1:]]
    return grid

grid = load_octile_map(map_path)
height, width = len(grid), len(grid[0])

# Step 3: Heuristic functions and graph tools
def in_bounds(x, y):
    return 0 <= x < height and 0 <= y < width and grid[x][y] != '@' and grid[x][y] != '#'

def neighbors(pos):
    x, y = pos
    for dx, dy in DIRS_8:
        nx, ny = x + dx, y + dy
        if in_bounds(nx, ny):
            yield (nx, ny)

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def octile(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dx, dy) + (2**0.5 - 1) * min(dx, dy)

def dijkstra(source):
    dist = {source: 0}
    pq = [(0, source)]
    while pq:
        cost, u = heapq.heappop(pq)
        for v in neighbors(u):
            move_cost = (2**0.5 if abs(v[0] - u[0]) == 1 and abs(v[1] - u[1]) == 1 else 1)
            new_cost = cost + move_cost
            if v not in dist or new_cost < dist[v]:
                dist[v] = new_cost
                heapq.heappush(pq, (new_cost, v))
    return dist

def fastmap_embedding(pivot1, pivot2):
    d_p1 = dijkstra(pivot1)
    d_p2 = dijkstra(pivot2)
    d_p1p2 = d_p1.get(pivot2, float('inf'))
    embedding = {}
    for x in range(height):
        for y in range(width):
            if in_bounds(x, y):
                v = (x, y)
                dx = d_p1.get(v, float('inf'))
                dy = d_p2.get(v, float('inf'))
                embedding[v] = (dx**2 + d_p1p2**2 - dy**2) / (2 * d_p1p2) if d_p1p2 != 0 else 0
    return embedding

def fastmap_heuristic(embedding, u, v):
    return abs(embedding.get(u, 0) - embedding.get(v, 0))

# Step 4: A* implementation
def astar(start, goal, heuristic_fn):
    open_set = [(0 + heuristic_fn(start, goal), 0, start)]
    g_score = {start: 0}
    visited = set()
    expansions = 0

    while open_set:
        _, cost, current = heapq.heappop(open_set)
        if current == goal:
            break
        if current in visited:
            continue
        visited.add(current)
        expansions += 1
        for neighbor in neighbors(current):
            move_cost = (2**0.5 if abs(neighbor[0]-current[0])==1 and abs(neighbor[1]-current[1])==1 else 1)
            tentative_g = cost + move_cost
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic_fn(neighbor, goal)
                heapq.heappush(open_set, (f, tentative_g, neighbor))
    return expansions

# Step 5: Run all heuristics
def find_valid_points(grid):
    free_cells = [(x, y) for x in range(height) for y in range(width) if in_bounds(x, y)]
    return free_cells[0], free_cells[-1]  # first and last valid cell

start, goal = find_valid_points(grid)
embedding = fastmap_embedding(start, goal)

results = {}

# Manhattan
start_time = time.time()
exp_manhattan = astar(start, goal, manhattan)
results['Manhattan'] = (exp_manhattan, time.time() - start_time)

# Octile
start_time = time.time()
exp_octile = astar(start, goal, octile)
results['Octile'] = (exp_octile, time.time() - start_time)

# FastMap
start_time = time.time()
exp_fastmap = astar(start, goal, lambda u, v: fastmap_heuristic(embedding, u, v))
results['FastMap'] = (exp_fastmap, time.time() - start_time)


df = pd.DataFrame(results, index=['Nodes Expanded', 'Time (s)']).T
from tabulate import tabulate
print("A* Heuristic Comparison")
print(tabulate(df, headers='keys', tablefmt='grid'))

print("A* Heuristic Comparison")
print(df.to_string())
