
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



int main() {
    string map_file = "rmtst01.map";
    string scen_file = "rmtst01.map.scen";

    unordered_set<pii, pair_hash> obstacles;
    int rows, cols;
    vector<string> grid = read_map(map_file, obstacles, rows, cols);
    pair<int, int> grid_size = {rows, cols};

    vector<Scenario> scenarios = read_scenarios(scen_file);

    for (int i = 0; i < min(500, (int)scenarios.size()); ++i) {
        const auto& s = scenarios[i];
        cout << "Solving scenario " << i << ": start (" << s.start.first << "," << s.start.second
             << ") → goal (" << s.goal.first << "," << s.goal.second << ")\n";

        vector<pii> path = a_star(s.start, s.goal, grid_size, obstacles);

        if (!path.empty()) {
            cout << "  Path found with " << path.size() << " steps.\n";
        } else {
            cout << "  No path found.\n";
        }
    }

    return 0;
}
