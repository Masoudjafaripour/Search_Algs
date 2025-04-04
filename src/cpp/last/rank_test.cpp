#include <queue>
#include <iostream>
#include <fstream>
#include <cassert>
#include <set>
#include <filesystem>
#include <vector>
#include <algorithm>


// Factorial function for computing pattern sizes
size_t factorial(int n) {
    size_t result = 1;
    for (int i = 2; i <= n; i++) {
        result *= i;
    }
    return result;
}

// Compute a pattern key using bitwise packing
uint64_t computePatternKeyInternal(const Puzzle &puzzle, const int *tiles, int count) {
    uint64_t key = 0;
    std::vector<int> positions;

    // Extract positions of relevant tiles in the puzzle
    for (int i = 0; i < count; i++) {
        for (int idx = 0; idx < Puzzle::SIZE; idx++) {
            if (puzzle.tiles[idx] == tiles[i]) {
                positions.push_back(idx);
                break;
            }
        }
    }
    // cout << "Positions: ";
    // for (int i = 0; i < count; i++) {
    //     cout << positions[i] << " ";
    // }
    // cout << endl;
    
    // Compute lexicographic rank
    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            if (positions[j] > positions[i]) {
                positions[j]--;
            }
        }
        key += positions[i] * factorial(Puzzle::SIZE - i - 1) / factorial(Puzzle::SIZE - count);
        // cout << "Key_"<<i<<": " << key << endl;
    }

    // Validate key before returning
    uint64_t max_key = factorial(16) / factorial(16 - count);
    if (key >= max_key) {
        std::cerr << "Error: computePatternKeyInternal() produced an out-of-bounds key! key=" << key << ", max=" << max_key << "\n";
        return 0;
    }

    return key;