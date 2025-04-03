# Re-import needed libraries after code execution environment was reset
import random

# Generate a larger random Octile-type map with some structure and save to file
def generate_octile_map(width, height, filename):
    symbols = ['@', 'T', '.', '#']
    weights = [0.3, 0.3, 0.3, 0.1]  # Probability of each terrain type
    map_data = []

    for _ in range(height):
        row = ''.join(random.choices(symbols, weights, k=width))
        map_data.append(row)

    with open(filename, 'w') as f:
        f.write(f"type octile\n")
        f.write(f"height {height}\n")
        f.write(f"width {width}\n")
        f.write(f"map\n")
        for row in map_data:
            f.write(row + "\n")
    
    return map_data

# Generate and store map in file
filename = "generated_map.map"  # Save in the current directory
octile_map = generate_octile_map(width=182, height=50, filename=filename)

# Show a small preview of the top of the map
print("\n".join(octile_map[:5]))  # Show first 5 rows as a preview
