#include "puzzle.h"
#include <algorithm>

SlidingPuzzle::SlidingPuzzle(array<int, 4> initial_state) {
    state = initial_state;
}

int SlidingPuzzle::find_zero() {
    return distance(state.begin(), find(state.begin(), state.end(), 0));
}

vector<pair<int, int>> SlidingPuzzle::get_valid_moves() {
    int zero_idx = find_zero();
    switch (zero_idx) {
        case 0: return {{0, 1}, {0, 2}};
        case 1: return {{1, 0}, {1, 3}};
        case 2: return {{2, 0}, {2, 3}};
        case 3: return {{3, 1}, {3, 2}};
    }
    return {};
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
