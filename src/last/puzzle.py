from typing import List, Tuple

class SlidingPuzzle:
    def __init__(self, initial_state: Tuple[int, int, int, int]):
        """Initialize the puzzle with a given initial state."""
        self.state = initial_state  # Store the state as a tuple
    
    def find_zero(self) -> int:
        """Find the index of the empty tile (0)."""
        return self.state.index(0)
    
    def get_valid_moves(self) -> List[Tuple[int, int]]:
        """Return a list of valid swaps (index pairs) based on zero position."""
        zero_idx = self.find_zero()
        swap_positions = {
            0: [(0, 1), (0, 2)],  # Move Right, Down
            1: [(1, 0), (1, 3)],  # Move Left, Down
            2: [(2, 0), (2, 3)],  # Move Right, Up
            3: [(3, 1), (3, 2)]   # Move Left, Up
        }
        return swap_positions[zero_idx]
    
    def apply_move(self, move: Tuple[int, int]) -> Tuple[int, int, int, int]:
        """Swap the zero tile with another tile and return the new state."""
        state_list = list(self.state)  # Convert tuple to a mutable list
        state_list[move[0]], state_list[move[1]] = state_list[move[1]], state_list[move[0]]  # Swap
        return tuple(state_list)  # Convert back to tuple
    
    def get_next_states(self) -> List[Tuple[int, int, int, int]]:
        """Generate all possible next states."""
        moves = self.get_valid_moves()
        return [self.apply_move(move) for move in moves]
    
    def is_goal(self) -> bool:
        """Check if the state is the goal state."""
        return self.state == (1, 2, 3, 0)

# Example usage:
initial_state = (1, 2, 0, 3)
puzzle = SlidingPuzzle(initial_state)

print("Initial state:", puzzle.state)
print("Valid moves:", puzzle.get_valid_moves())
print("Next states:", puzzle.get_next_states())
print("Is goal state?", puzzle.is_goal())

puzzle = SlidingPuzzle(puzzle.get_next_states()[0])

print("Valid moves:", puzzle.get_valid_moves())
print("Next states:", puzzle.get_next_states())
print("Is goal state?", puzzle.is_goal())
