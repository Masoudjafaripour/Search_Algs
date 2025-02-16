#include "puzzle.h"
#include <iostream>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>  


using namespace std;

vector<array<int, 4>> bfs_solve(array<int, 4> initial_state) {
    array<int, 4> goal_state = {1, 2, 3, 0};

    queue<array<int, 4>> q;
    unordered_set<string> visited;
    unordered_map<string, array<int, 4>> parent;

    auto to_string_state = [](array<int, 4> state) {
        string s = "";
        for (int num : state) s += to_string(num);
        return s;
    };

    q.push(initial_state);
    visited.insert(to_string_state(initial_state));
    parent[to_string_state(initial_state)] = {-1, -1, -1, -1};

    while (!q.empty()) {
        array<int, 4> current_state = q.front();
        q.pop();

        if (current_state == goal_state) {
            vector<array<int, 4>> path;
            while (current_state != array<int, 4>{-1, -1, -1, -1}) {
                path.push_back(current_state);
                current_state = parent[to_string_state(current_state)];
            }
            reverse(path.begin(), path.end());
            return path;
        }

        SlidingPuzzle puzzle(current_state);
        for (array<int, 4> next_state : puzzle.get_next_states()) {
            string state_str = to_string_state(next_state);
            if (visited.find(state_str) == visited.end()) {
                q.push(next_state);
                visited.insert(state_str);
                parent[state_str] = current_state;
            }
        }
    }

    return {};
}

int main() {
    array<int, 4> initial_state = {1, 2, 0, 3};
    vector<array<int, 4>> solution_path = bfs_solve(initial_state);

    if (!solution_path.empty()) {
        cout << "Solution found in " << solution_path.size() - 1 << " moves:\n";
        for (auto state : solution_path) {
            for (int num : state) cout << num << " ";
            cout << endl;
        }
    } else {
        cout << "No solution found.\n";
    }

    return 0;
}
