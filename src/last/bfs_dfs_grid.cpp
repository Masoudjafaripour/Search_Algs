#include <iostream>
#include <vector>
#include <queue>
#include <stack>

using namespace std;

struct Node {
    int x, y;
    vector<pair<int, int>> path;  // Stores the path taken
};

// Possible moves (Up, Down, Left, Right)
vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

// Function to check if a position is valid
bool is_valid(int x, int y, const vector<vector<int>>& grid, vector<vector<bool>>& visited) {
    int rows = grid.size(), cols = grid[0].size();
    return (x >= 0 && x < rows && y >= 0 && y < cols && grid[x][y] == 0 && !visited[x][y]);
}

// Breadth-First Search (BFS) for shortest path
vector<pair<int, int>> bfs_solve(const vector<vector<int>>& grid, pair<int, int> start, pair<int, int> goal) {
    int rows = grid.size(), cols = grid[0].size();
    queue<Node> q;
    vector<vector<bool>> visited(rows, vector<bool>(cols, false));

    q.push({start.first, start.second, {start}});
    visited[start.first][start.second] = true;

    while (!q.empty()) {
        Node current = q.front();
        q.pop();

        if (current.x == goal.first && current.y == goal.second) {
            return current.path;  // Return the shortest path
        }

        for (auto [dx, dy] : directions) {
            int nx = current.x + dx, ny = current.y + dy;
            if (is_valid(nx, ny, grid, visited)) {
                visited[nx][ny] = true;
                vector<pair<int, int>> new_path = current.path;
                new_path.emplace_back(nx, ny);
                q.push({nx, ny, new_path});
            }
        }
    }

    return {};  // No path found
}

// Depth-First Search (DFS) for pathfinding (may not find the shortest path)
vector<pair<int, int>> dfs_solve(const vector<vector<int>>& grid, pair<int, int> start, pair<int, int> goal) {
    int rows = grid.size(), cols = grid[0].size();
    stack<Node> s;
    vector<vector<bool>> visited(rows, vector<bool>(cols, false));

    s.push({start.first, start.second, {start}});
    visited[start.first][start.second] = true;

    while (!s.empty()) {
        Node current = s.top();
        s.pop();

        if (current.x == goal.first && current.y == goal.second) {
            return current.path;  // Return the found path
        }

        for (auto [dx, dy] : directions) {
            int nx = current.x + dx, ny = current.y + dy;
            if (is_valid(nx, ny, grid, visited)) {
                visited[nx][ny] = true;
                vector<pair<int, int>> new_path = current.path;
                new_path.emplace_back(nx, ny);
                s.push({nx, ny, new_path});
            }
        }
    }

    return {};  // No path found
}

// Function to print the grid with path
void print_grid_with_path(vector<vector<int>> grid, const vector<pair<int, int>>& path) {
    for (auto [x, y] : path) {
        grid[x][y] = 2;  // Mark path in the grid
    }

    cout << "Grid (2 = path, 1 = obstacle, 0 = open):\n";
    for (const auto& row : grid) {
        for (int cell : row) {
            cout << cell << " ";
        }
        cout << "\n";
    }
    cout << "\n";
}

int main() {
    vector<vector<int>> grid = {
        {0, 0, 0, 0, 1},
        {1, 1, 0, 1, 0},
        {0, 0, 0, 1, 0},
        {0, 1, 1, 0, 0},
        {0, 0, 0, 0, 0}
    };

    pair<int, int> start = {0, 0};  // Start position
    pair<int, int> goal = {4, 4};   // Goal position

    cout << "Solving using BFS...\n";
    vector<pair<int, int>> bfs_path = bfs_solve(grid, start, goal);
    if (!bfs_path.empty()) {
        cout << "BFS Path found (" << bfs_path.size() - 1 << " moves):\n";
        for (auto [x, y] : bfs_path) cout << "(" << x << "," << y << ") ";
        cout << "\n";
        print_grid_with_path(grid, bfs_path);
    } else {
        cout << "No BFS path found.\n";
    }

    cout << "Solving using DFS...\n";
    vector<pair<int, int>> dfs_path = dfs_solve(grid, start, goal);
    if (!dfs_path.empty()) {
        cout << "DFS Path found (" << dfs_path.size() - 1 << " moves):\n";
        for (auto [x, y] : dfs_path) cout << "(" << x << "," << y << ") ";
        cout << "\n";
        print_grid_with_path(grid, dfs_path);
    } else {
        cout << "No DFS path found.\n";
    }

    return 0;
}
