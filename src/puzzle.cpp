#include "puzzle.h"
#include <iostream>
#include <algorithm>

using namespace std;

SlidingPuzzle::SlidingPuzzle(array<int, 4> initial_state) {
    state = initial_state;
}

int SlidingPuzzle::find_zero() {
    return distance(state.begin(), find(state.begin(), state.end(), 0));
}

vector<pair<int, int>> SlidingPuzzle::get_valid_moves() {
    int zero_idx = find_zero();
    vector<pair<int, int>> moves;
    
    // Define valid swaps based on index of zero
    switch (zero_idx) {
        case 0: moves = {{0, 1}, {0, 2}}; break;
        case 1: moves = {{1, 0}, {1, 3}}; break;
        case 2: moves = {{2, 0}, {2, 3}}; break;
        case 3: moves = {{3, 1}, {3, 2}}; break;
    }
    return moves;
}

array<int, 4> SlidingPuzzle::apply_move(pair<int, int> move) {
    array<int, 4> new_state = state;
    swap(new_state[move.first], new_state[move.second]);
    return new_state;
}

vector<array<int, 4>> SlidingPuzzle::get_next_states() {
    vector<array<int, 4>> next_states;
    for (auto move : get_valid_moves()) {
        next_states.push_back(apply_move(move));
    }
    return next_states;
}

bool SlidingPuzzle::is_goal() {
    return state == array<int, 4>{1, 2, 3, 0};
}

int main() {
    array<int, 4> initial_state = {1, 2, 0, 3};
    SlidingPuzzle puzzle(initial_state);

    cout << "Initial state: ";
    for (int num : puzzle.state) cout << num << " ";
    cout << endl;

    cout << "Valid moves: ";
    for (auto move : puzzle.get_valid_moves()) {
        cout << "(" << move.first << ", " << move.second << ") ";
    }
    cout << endl;

    cout << "Next states:\n";
    for (auto new_state : puzzle.get_next_states()) {
        for (int num : new_state) cout << num << " ";
        cout << endl;
    }

    cout << "Is goal state? " << (puzzle.is_goal() ? "Yes" : "No") << endl;
    
    return 0;
}