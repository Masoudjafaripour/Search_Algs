cmake_minimum_required(VERSION 3.10)

# Set the project name
project(SlidingPuzzle)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable (ensure all necessary files are included)
add_executable(SlidingPuzzle src/bfs.cpp src/puzzle.cpp)
