import numpy as np
import matplotlib.pyplot as plt

def heuristic(a, b):
    # Manhattan distance heuristic for A* algorithm
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def nbs(node):
    # Get the neighbors of a node in a grid
    x, y = node
    return []

obstacles = {(1, 1), (1, 2), (2, 1), (2, 2)} # example obstacles
grid_size = (5, 5) # example grid size

start = (0, 0) # starting point
goal = (4, 4) # goal point


def a_star(start, goal):
    open_list = {start} # what is this file format?
    closed_list = {}
    g = {start: 0} # cost from start to current node
    f = {start: heuristic(start, goal)} # cost from start to goal through current node
    came_from = {} # to reconstruct the path

    while open_list:
        current = min(open_list, key=lambda o: f[o])
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]
        open_list.remove(current)
        closed_list.add(current)

        for nb in nbs(current):
            if nb in closed_list or nb not in obstacles:
                continue
            if nb not in open_list:
                open_list.add(nb)
            tentative_g = g[current] + 1
            if tentative_g >= g.get(nb, float('inf')):
                continue
            came_from[nb] = current
            g[nb] = tentative_g
            f[nb] = g[nb] + heuristic(nb, goal)

    return print("No path found")

path = a_star(start, goal)
print(path)
