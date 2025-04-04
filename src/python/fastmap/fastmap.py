import heapq
import numpy as np
import time
import matplotlib.pyplot as plt
from collections import defaultdict

# Define the grid (5x5)
grid = [
    ['S', '.', '.', '.', 'G'],
    ['.', '#', '#', '#', '.'],
    ['.', '.', '.', '#', '.'],
    ['.', '#', '.', '.', '.'],
    ['.', '.', '.', '#', '.']
]

rows, cols = len(grid), len(grid[0])
start, goal = (0, 0), (0, 4)

# Directions for 4-connected and 8-connected grid
DIRS_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
DIRS_8 = DIRS_4 + [(-1, -1), (-1, 1), (1, -1), (1, 1)]

def in_bounds(x, y):
    return 0 <= x < rows and 0 <= y < cols and grid[x][y] != '#'

def neighbors(pos, use_diagonals=False):
    dirs = DIRS_8 if use_diagonals else DIRS_4
    for dx, dy in dirs:
        nx, ny = pos[0] + dx, pos[1] + dy
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
        for v in neighbors(u, use_diagonals=True):
            new_cost = cost + (2**0.5 if abs(v[0]-u[0]) == 1 and abs(v[1]-u[1]) == 1 else 1)
            if v not in dist or new_cost < dist[v]:
                dist[v] = new_cost
                heapq.heappush(pq, (new_cost, v))
    return dist

# FastMap: 1D embedding using two pivots
def fastmap_embedding(pivot1, pivot2):
    d_p1 = dijkstra(pivot1)
    d_p2 = dijkstra(pivot2)
    d_p1p2 = d_p1.get(pivot2, float('inf'))
    embedding = {}
    for x in range(rows):
        for y in range(cols):
            if grid[x][y] != '#':
                v = (x, y)
                dx = d_p1.get(v, float('inf'))
                dy = d_p2.get(v, float('inf'))
                embedding[v] = (dx**2 + d_p1p2**2 - dy**2) / (2 * d_p1p2) if d_p1p2 != 0 else 0
    return embedding

def fastmap_heuristic(embedding, u, v):
    return abs(embedding.get(u, 0) - embedding.get(v, 0))

# A* Implementation
def astar(start, goal, heuristic_fn):
    open_set = [(0 + heuristic_fn(start, goal), 0, start)]
    came_from = {}
    g_score = defaultdict(lambda: float('inf'))
    g_score[start] = 0
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
        for neighbor in neighbors(current, use_diagonals=True):
            tentative_g = cost + (2**0.5 if abs(neighbor[0]-current[0])==1 and abs(neighbor[1]-current[1])==1 else 1)
            if tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic_fn(neighbor, goal)
                heapq.heappush(open_set, (f, tentative_g, neighbor))
                came_from[neighbor] = current
    return expansions

# Run experiments
results = {}

# Manhattan
start_time = time.time()
man_exp = astar(start, goal, manhattan)
results['Manhattan'] = (man_exp, time.time() - start_time)

# Octile
start_time = time.time()
oct_exp = astar(start, goal, octile)
results['Octile'] = (oct_exp, time.time() - start_time)

# FastMap
p1, p2 = (0, 0), (4, 4)
embedding = fastmap_embedding(p1, p2)
start_time = time.time()
fm_exp = astar(start, goal, lambda u, v: fastmap_heuristic(embedding, u, v))
results['FastMap'] = (fm_exp, time.time() - start_time)

import pandas as pd
df = pd.DataFrame(results, index=['Nodes Expanded', 'Time (s)']).T
from tabulate import tabulate
print("A* Heuristic Comparison")
print(tabulate(df, headers='keys', tablefmt='grid'))

print("A* Heuristic Comparison")
print(df.to_string())
