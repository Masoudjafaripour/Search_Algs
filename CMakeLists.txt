cmake_minimum_required(VERSION 3.10)

# Set the project name
project(SlidingPuzzle)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Create separate executables for BFS, DFS, and Gridworld
# add_executable(bfs src/bfs.cpp src/puzzle.cpp)
# add_executable(dfs src/dfs.cpp src/puzzle.cpp)
# add_executable(grid_search src/bfs_dfs_grid.cpp)

# add_executable(Rank src/Rank.cpp)
# add_executable(FM src/cpp/fastmap.cpp)
add_executable(A_star src/cpp/a_star_grid_8_con.cpp)

# Link libraries (if necessary)
