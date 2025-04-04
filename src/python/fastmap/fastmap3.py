import random
import heapq
import time
import pandas as pd

# 8-connected movement (Octile grid)
DIRS_8 = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

# Step 1: Generate a large octile map and save to file
def generate_octile_map(width, height, filename):
    symbols = ['@', 'T', '.', '#']
    weights = [0.3, 0.3, 0.3, 0.1]
    map_data = []
    for _ in range(height):
        row = ''.join(random.choices(symbols, weights, k=width))
        map_data.append(row)
    with open(filename, 'w') as f:
        f.write(f"type octile\nheight {height}\nwidth {width}\nmap\n")
        for row in map_data:
            f.write(row + "\n")
    return map_data

map_path = "generated_map.map"
generate_octile_map(width=182, height=50, filename=map_path)

# Step 2: Load the map
def load_octile_map(path):
    with open(path, 'r') as f:
        lines = f.readlines()
    header_end = lines.index('map\n')
    grid = [list(line.strip()) for line in lines[header_end + 1:]]
    return grid

grid = load_octile_map(map_path)
height, width = len(grid), len(grid[0])

# Step 3: Utility functions
def in_bounds(x, y):
    return 0 <= x < height and 0 <= y < width and grid[x][y] not in {'@', '#'}

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

# Step 4: Full FastMap (paper version)
def dijkstra_with_weights(source, edge_weights):
    dist = {source: 0}
    pq = [(0, source)]
    while pq:
        cost, u = heapq.heappop(pq)
        for v in neighbors(u):
            move_cost = edge_weights.get((u, v), 1.0)
            new_cost = cost + move_cost
            if v not in dist or new_cost < dist[v]:
                dist[v] = new_cost
                heapq.heappush(pq, (new_cost, v))
    return dist

def get_all_edges():
    edges = {}
    for x in range(height):
        for y in range(width):
            u = (x, y)
            if not in_bounds(x, y): continue
            for v in neighbors(u):
                move_cost = 2**0.5 if abs(v[0]-u[0]) == 1 and abs(v[1]-u[1]) == 1 else 1
                edges[(u, v)] = move_cost
    return edges

def fastmap_embedding_paper(G, Kmax=5, eps=1e-3):
    coords = {}
    edge_weights = G.copy()
    for k in range(Kmax):
        na = random.choice(list(set(u for (u, _) in edge_weights)))
        dist_na = dijkstra_with_weights(na, edge_weights)
        nb = max(dist_na, key=lambda v: dist_na.get(v, float('inf')))
        dist_nb = dijkstra_with_weights(nb, edge_weights)
        na = max(dist_nb, key=lambda v: dist_nb.get(v, float('inf')))

        dist_na = dijkstra_with_weights(na, edge_weights)
        dist_nb = dijkstra_with_weights(nb, edge_weights)
        dab = dist_na.get(nb, float('inf'))

        if dab < eps:
            break

        for v in dist_na:
            dav = dist_na.get(v, float('inf'))
            dvb = dist_nb.get(v, float('inf'))
            coords.setdefault(v, [])
            coords[v].append((dav + dab - dvb) / 2)

        for (u, v) in list(edge_weights.keys()):
            if u in coords and v in coords:
                pu = coords[u][-1]
                pv = coords[v][-1]
                edge_weights[(u, v)] = max(0, edge_weights[(u, v)] - abs(pu - pv))
    return coords

def fastmap_heuristic(coords, u, v):
    return sum(abs(coords.get(u, [0]*len(coords.get(v, [])))[i] - coords.get(v, [0]*len(coords.get(u, [])))[i])
               for i in range(len(coords.get(u, []))))

# Step 5: A* Search
def astar(start, goal, heuristic_fn):
    open_set = [(heuristic_fn(start, goal), 0, start)]
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
            move_cost = 2**0.5 if abs(neighbor[0]-current[0]) == 1 and abs(neighbor[1]-current[1]) == 1 else 1
            tentative_g = cost + move_cost
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic_fn(neighbor, goal)
                heapq.heappush(open_set, (f, tentative_g, neighbor))
    return expansions

# Step 6: Run comparisons
def find_two_valid_points(grid):
    free_cells = [(x, y) for x in range(height) for y in range(width) if in_bounds(x, y)]
    return free_cells[0], free_cells[-1] if len(free_cells) >= 2 else (None, None)

start, goal = find_two_valid_points(grid)
G = get_all_edges()
embedding_fm = fastmap_embedding_paper(G, Kmax=5)

results = {}

# Manhattan
start_time = time.time()
exp_manhattan = astar(start, goal, manhattan)
results['Manhattan'] = (exp_manhattan, time.time() - start_time)

# Octile
start_time = time.time()
exp_octile = astar(start, goal, octile)
results['Octile'] = (exp_octile, time.time() - start_time)

# FastMap (Paper Version)
start_time = time.time()
exp_fastmap = astar(start, goal, lambda u, v: fastmap_heuristic(embedding_fm, u, v))
results['FastMap_Paper'] = (exp_fastmap, time.time() - start_time)

# Output results
df = pd.DataFrame(results, index=['Nodes Expanded', 'Time (s)']).T
from tabulate import tabulate
print("A* Heuristic Comparison")
print(tabulate(df, headers='keys', tablefmt='grid'))

print("\nFull Results:")
print(df.to_string())
