#ifndef PUZZLE_H
#define PUZZLE_H

#include <array>
#include <vector>
#include <utility>

class SlidingPuzzle {
public:
    std::array<int, 4> state;  // 2x2 puzzle stored as a 1D array

    SlidingPuzzle(std::array<int, 4> initial_state);
    int find_zero();
    std::vector<std::pair<int, int>> get_valid_moves();
    std::array<int, 4> apply_move(std::pair<int, int> move);
    std::vector<std::array<int, 4>> get_next_states();
    bool is_goal();
};

#endif // PUZZLE_H