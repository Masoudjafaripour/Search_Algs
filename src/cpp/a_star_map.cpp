
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
#include <string>   // For std::string
#include <sstream>  // For std::istringstream


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

vector<string> read_map(const string& filename, unordered_set<pii, pair_hash>& obstacles, int& rows, int& cols) {
    ifstream fin(filename);
    if (!fin.is_open()) {
        cerr << "Failed to open map file: " << filename << endl;
        exit(1);}
    string line;
    vector<string> grid;
    while (getline(fin, line)) {
        if (line.rfind("height", 0) == 0) {
            rows = stoi(line.substr(7));
        } else if (line.rfind("width", 0) == 0) {
            cols = stoi(line.substr(6));
        } else if (line == "map") {
            break;
        }
    }
    for (int r = 0; r < rows && getline(fin, line); ++r) {
        grid.push_back(line);
        for (int c = 0; c < cols; ++c) {
            char ch = line[c];
            if (ch == '@' || ch == 'T' || ch == 'W') {
                obstacles.insert({r, c});
            }
        }
    }
    return grid;
}


struct Scenario {
    pii start;
    pii goal;
    float cost;
};

vector<Scenario> read_scenarios(const string& filename) {
    ifstream fin(filename);
    if (!fin.is_open()) {
        cerr << "Failed to open scenario file: " << filename << endl;
        exit(1);
    }
    string line;
    getline(fin, line); // skip version line

    vector<Scenario> scenarios;
    while (getline(fin, line)) {
        istringstream iss(line);
        int bucket, width, height, sx, sy, gx, gy;
        string map_name;
        float cost;
        iss >> bucket >> map_name >> width >> height >> sx >> sy >> gx >> gy >> cost;
        scenarios.push_back({{sy, sx}, {gy, gx}, cost});  // note: row, col order!
    }
    return scenarios;
}

void print_map_region(const vector<string>& grid, pii center, int radius = 5) {
    int r0 = max(0, center.first - radius);
    int r1 = min((int)grid.size(), center.first + radius + 1);
    int c0 = max(0, center.second - radius);
    int c1 = min((int)grid[0].size(), center.second + radius + 1);

    for (int r = r0; r < r1; ++r) {
        for (int c = c0; c < c1; ++c) {
            if (pii{r, c} == center)
                cout << 'S';  // Mark center
            else
                cout << grid[r][c];
        }
        cout << '\n';
    }
}

bool is_valid(const pii& p, int rows, int cols, const unordered_set<pii, pair_hash>& obstacles) {
    return is_in_bounds(p.first, p.second, rows, cols) && !obstacles.count(p);
}


int main() {
    // string map_file = "rmtst01.map";
    // string scen_file = "rmtst01.map.scen";

    string map_file = "AcrosstheCape.map";
    string scen_file = "AcrosstheCape.map.scen";

    unordered_set<pii, pair_hash> obstacles;
    int rows, cols;
    vector<string> grid = read_map(map_file, obstacles, rows, cols);
    pair<int, int> grid_size = {rows, cols};

    vector<Scenario> scenarios = read_scenarios(scen_file);

    int max_scenarios = min(500, (int)scenarios.size());
    int solved_count = 0;
    int total_path_length = 0;
    vector<int> failed_indices;

    for (int i = 0; i < max_scenarios; ++i) {
        const auto& s = scenarios[i];

        // Optional: Skip scenarios where start or goal is inside an obstacle
        if (obstacles.count(s.start) || obstacles.count(s.goal)) {
            failed_indices.push_back(i);
            continue;
        }

        if (obstacles.count(s.start)) cout << "  Start is inside an obstacle!\n";
        if (obstacles.count(s.goal)) cout << "  Goal is inside an obstacle!\n";

        if (!is_valid(s.start, rows, cols, obstacles) || !is_valid(s.goal, rows, cols, obstacles)) {
            cout << "  ⚠️ Scenario " << i << " is invalid (start/goal out of bounds or in obstacle).\n";
            failed_indices.push_back(i);
            continue;
        }

        vector<pii> path = a_star(s.start, s.goal, grid_size, obstacles);

        if (!path.empty()) {
            solved_count++;
            total_path_length += path.size();
        } else {
            failed_indices.push_back(i);
        }
    }

    // Print summary
    cout << "\n=== Summary ===\n";
    cout << "Total scenarios attempted: " << max_scenarios << "\n";
    cout << "Solved: " << solved_count << "\n";
    cout << "Failed: " << failed_indices.size() << "\n";

    if (solved_count > 0)
        cout << "Average path length (of solved): " << (float)total_path_length / solved_count << "\n";
    else
        cout << "Average path length: N/A\n";

    if (!failed_indices.empty()) {
        cout << "\nFailed scenarios:\n";
        for (int idx : failed_indices) {
            const auto& s = scenarios[idx];
            cout << "  Scenario " << idx << ": Start (" << s.start.first << "," << s.start.second
                 << ") → Goal (" << s.goal.first << "," << s.goal.second << ")\n";
                //  print_map_region(grid, s.start);
                //  print_map_region(grid, s.goal);
        }
        
    }

    return 0;
}
