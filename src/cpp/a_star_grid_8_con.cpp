#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <set>
#include <fstream>

using namespace std;

using pii = pair<int, int>;

struct pair_hash {
    size_t operator()(const pii& p) const {
        return hash<int>()(p.first) ^ (hash<int>()(p.second) << 1);
    }
};

float heuristic(const pii& a, const pii& b) {
    int dr = abs(a.first - b.first);
    int dc = abs(a.second - b.second);
    return max(dr, dc) + (sqrt(2.0) - 1) * min(dr, dc);
}

bool is_in_bounds(int r, int c, int rows, int cols) {
    return (r >= 0 && r < rows && c >= 0 && c < cols);
}

vector<pii> get_neighbors_8(const pii& current, const pair<int, int>& grid_size,
                            const unordered_set<pii, pair_hash>& obstacles) {
    vector<pii> neighbors;
    int r = current.first;
    int c = current.second;
    int rows = grid_size.first;
    int cols = grid_size.second;

    const int DX[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
    const int DY[8] = {-1, 0, 1,-1, 1,-1, 0, 1};

    for (int i = 0; i < 8; ++i) {
        int nr = r + DX[i];
        int nc = c + DY[i];
        if (!is_in_bounds(nr, nc, rows, cols)) continue;
        if (obstacles.count({nr, nc})) continue;



        neighbors.emplace_back(nr, nc);
    }

    // Remove diagonal neighbors that cut corners
    const int DX_d[4] = {-1, -1, 1, 1};
    const int DY_d[4] = {-1, 1, -1, 1};

    for (int i = 0; i < 4; ++i) {
        int nr = r + DX_d[i];
        int nc = c + DY_d[i];

        // Check bounds again if needed
        if (!is_in_bounds(nr, nc, rows, cols)) continue;

        // Check if both adjacent sides are blocked => corner cutting
        if (obstacles.count({r, nc}) || obstacles.count({nr, c})) {
            // Try to remove (nr, nc) from neighbors
            auto it = std::find(neighbors.begin(), neighbors.end(), std::make_pair(nr, nc));
            if (it != neighbors.end()) {
                neighbors.erase(it);
            }
        }
    }

    
    return neighbors;
}

float move_cost(const pii& a, const pii& b) {
    return (a.first != b.first && a.second != b.second) ? sqrt(2.0) : 1.0;
}

vector<pii> reconstruct_path(unordered_map<pii, pii, pair_hash>& came_from, pii current) {
    vector<pii> path = {current};
    while (came_from.count(current)) {
        current = came_from[current];
        path.push_back(current);
    }
    reverse(path.begin(), path.end());
    return path;
}

vector<pii> a_star(const pii& start, const pii& goal, const pair<int,int>& grid_size,
                   const unordered_set<pii, pair_hash>& obstacles) {
    using QueueElement = pair<float, pii>;
    priority_queue<QueueElement, vector<QueueElement>, greater<>> open_list;
    open_list.emplace(0.0f, start);

    unordered_set<pii, pair_hash> closed_list;
    unordered_map<pii, float, pair_hash> g;
    unordered_map<pii, float, pair_hash> f;
    unordered_map<pii, pii, pair_hash> came_from;

    g[start] = 0.0;
    f[start] = heuristic(start, goal);

    while (!open_list.empty()) {
        pii current = open_list.top().second;
        open_list.pop();

        if (current == goal) {
            return reconstruct_path(came_from, current);
        }

        closed_list.insert(current);

        for (const pii& nb : get_neighbors_8(current, grid_size, obstacles)) {
            if (closed_list.count(nb)) continue;

            float tentative_g = g[current] + move_cost(current, nb);

            if (!g.count(nb) || tentative_g < g[nb]) {
                came_from[nb] = current;
                g[nb] = tentative_g;
                f[nb] = tentative_g + heuristic(nb, goal);
                open_list.emplace(f[nb], nb);
            }
        }
    }

    return {}; // No path found
}

unordered_set<pii, pair_hash> generate_random_obstacles(int rows, int cols, int num_obstacles,
                                                        const pii& start, const pii& goal) {
    unordered_set<pii, pair_hash> obstacles;
    srand(time(nullptr));
    while (obstacles.size() < num_obstacles) {
        int r = rand() % rows;
        int c = rand() % cols;
        pii obs = {r, c};
        if (obs != start && obs != goal)
            obstacles.insert(obs);
    }
    return obstacles;
}

int main() {
    int rows = 20, cols = 20;
    int num_obstacles = rows * 5;
    pii start = {0, 0};
    pii goal = {rows - 1, cols - 1};

    unordered_set<pii, pair_hash> obstacles = generate_random_obstacles(rows, cols, num_obstacles, start, goal);
    pair<int, int> grid_size = {rows, cols};

    vector<pii> path = a_star(start, goal, grid_size, obstacles);

    if (!path.empty()) {
        cout << "Path found:\n";
        for (const auto& p : path) {
            cout << "(" << p.first << "," << p.second << ") ";
        }
        cout << endl;
    } else {
        cout << "No path found.\n";
    }

    // Write to file for Python visualization
    ofstream fout("path_output.txt");
    fout << rows << " " << cols << "\n";
    fout << start.first << " " << start.second << "\n";
    fout << goal.first << " " << goal.second << "\n";
    fout << obstacles.size() << "\n";
    for (const auto& o : obstacles)
        fout << o.first << " " << o.second << "\n";
    for (const auto& p : path)
        fout << p.first << " " << p.second << "\n";
    fout.close();

    // Call the Python visualization
    system("\"C:\\Program Files\\Python312\\python.exe\" visualize.py");

    return 0;
}
