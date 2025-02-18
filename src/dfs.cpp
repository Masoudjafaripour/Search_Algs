#include <iostream>
#include <stack>
#include <unordered_set>
#include <unordered_map>
#include "puzzle.cpp"  // Include your SlidingPuzzle class
#include <string>


using namespace std;

// DFS function to solve the sliding puzzle
vector<array<int, 4>> dfs_solve(array<int, 4> initial_state) {
    array<int, 4> goal_state = {1, 2, 3, 0};

    stack<array<int, 4>> s;  // DFS stack
    unordered_set<string> visited;  // Track visited states
    unordered_map<string, array<int, 4>> parent;  // Store parent states

    auto to_string_state = [](array<int, 4> state) {
        string s = "";
        for (int num : state) s += to_string(num);
        return s;
    };

    s.push(initial_state);
    visited.insert(to_string_state(initial_state));
    parent[to_string_state(initial_state)] = {-1, -1, -1, -1};  // Sentinel for root

    while (!s.empty()) {
        array<int, 4> current_state = s.top();
        s.pop();

        // If we reach the goal state, reconstruct the path
        if (current_state == goal_state) {
            vector<array<int, 4>> path;
            while (current_state != array<int, 4>{-1, -1, -1, -1}) {
                path.push_back(current_state);
                current_state = parent[to_string_state(current_state)];
            }
            reverse(path.begin(), path.end());
            return path;
        }

        // Expand neighbors (LIFO order)
        SlidingPuzzle puzzle(current_state);
        vector<array<int, 4>> next_states = puzzle.get_next_states();
        reverse(next_states.begin(), next_states.end());  // Ensure LIFO behavior

        for (array<int, 4> next_state : next_states) {
            string state_str = to_string_state(next_state);
            if (visited.find(state_str) == visited.end()) {
                s.push(next_state);
                visited.insert(state_str);
                parent[state_str] = current_state;
            }
        }
    }

    return {};  // No solution found
}

// Example usage
int main() {
    array<int, 4> initial_state = {1, 2, 0, 3};
    vector<array<int, 4>> solution_path = dfs_solve(initial_state);

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
