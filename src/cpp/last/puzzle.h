#ifndef PUZZLE_H
#define PUZZLE_H

#include <array>
#include <vector>

using namespace std;

class SlidingPuzzle {
public:
    array<int, 4> state;

    SlidingPuzzle(array<int, 4> initial_state);
    int find_zero();
    vector<pair<int, int>> get_valid_moves();
    array<int, 4> apply_move(pair<int, int> move);
    vector<array<int, 4>> get_next_states();
    bool is_goal();
};

#endif // PUZZLE_H
