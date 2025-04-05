// A* Search on 8-connected grid with FastMap heuristic (preprocessed)
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <set>
#include <tuple>

using namespace std;

struct Node {
    int x, y;
    double cost;
    bool operator>(const Node& other) const {
        return cost > other.cost;
    }
};

const int DX[8] = {-1,-1,-1, 0, 0, 1, 1, 1};
const int DY[8] = {-1, 0, 1,-1, 1,-1, 0, 1};
const double MOVE_COST[8] = {
    sqrt(2), 1, sqrt(2), 1, 1, sqrt(2), 1, sqrt(2)
};

int H, W;
vector<string> grid;
unordered_map<int, vector<double>> fastmap_embedding;

inline bool in_bounds(int x, int y) {
    return x >= 0 && y >= 0 && x < H && y < W && grid[x][y] != '@' && grid[x][y] != '#';
}

inline bool can_move_diag(int x, int y, int dx, int dy) {
    return in_bounds(x + dx, y) && in_bounds(x, y + dy);
}

inline double heuristic(const pair<int,int>& a, const pair<int,int>& b) {
    const auto& va = fastmap_embedding[a.first * W + a.second];
    const auto& vb = fastmap_embedding[b.first * W + b.second];
    double h = 0;
    for (size_t i = 0; i < va.size(); ++i) h += fabs(va[i] - vb[i]);
    return h;
}

int astar(pair<int,int> start, pair<int,int> goal) {
    priority_queue<Node, vector<Node>, greater<Node>> open;
    unordered_map<int, double> g;
    set<int> visited;

    int sid = start.first * W + start.second;
    int gid = goal.first * W + goal.second;
    g[sid] = 0;
    open.push({start.first, start.second, heuristic(start, goal)});

    while (!open.empty()) {
        Node cur = open.top(); open.pop();
        int cid = cur.x * W + cur.y;
        if (visited.count(cid)) continue;
        visited.insert(cid);
        if (cur.x == goal.first && cur.y == goal.second) return visited.size();

        for (int i = 0; i < 8; ++i) {
            int nx = cur.x + DX[i];
            int ny = cur.y + DY[i];
            if (!in_bounds(nx, ny)) continue;
            if (i % 2 == 0 && !can_move_diag(cur.x, cur.y, DX[i], DY[i])) continue; // corner-cutting check

            int nid = nx * W + ny;
            double ng = g[cid] + MOVE_COST[i];
            if (!g.count(nid) || ng < g[nid]) {
                g[nid] = ng;
                double f = ng + heuristic({nx, ny}, goal);
                open.push({nx, ny, f});
            }
        }
    }
    return -1; // path not found
}

int main() {
    // Load map
    H = 5; W = 10;
    grid = {
        "@@@@@@@@@@",
        "TTWW@....@",
        "TTWW@....@",
        "TTSS@....@",
        "TTSS.....@"
    };

    // Example: mock FastMap embedding
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            if (grid[i][j] != '@' && grid[i][j] != '#')
                fastmap_embedding[i * W + j] = {double(i), double(j)}; // mock coords
        }
    }

    pair<int,int> start = {4, 0}, goal = {1, 8};
    int expanded = astar(start, goal);
    if (expanded != -1)
        cout << "Path found. Nodes expanded: " << expanded << endl;
    else
        cout << "No path found." << endl;
    return 0;
}
