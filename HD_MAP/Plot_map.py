import matplotlib.pyplot as plt

# Function to read waypoints from a file
def read_waypoints(filename):
    waypoints = []
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()  # Remove whitespace characters
            if line:  # Check if the line is not empty
                try:
                    latitude, longitude = map(float, line.split(','))
                    waypoints.append((latitude, longitude))
                except ValueError:
                    print(f"Skipping malformed line: {line}")
    print("Waypoints read from file:", waypoints)  # Debugging: print waypoints
    return waypoints

# Function to plot waypoints as a path
def plot_path(waypoints):
    if not waypoints:
        print("No waypoints to plot.")
        return
    
    latitudes = [point[0] for point in waypoints]
    longitudes = [point[1] for point in waypoints]

    # Create the plot
    plt.figure(figsize=(10, 6))
    
    # Plot the points
    plt.plot(longitudes, latitudes, marker='o', markersize=0, linestyle='-', color='b', label='Path')
    
    # Add labels and title
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Path from Waypoints')
    plt.legend()
    
    # Add grid and show the plot
    plt.grid(True)
    plt.show()

# Example usage
filename = 'output_xy.txt'  # Path to your text file
waypoints = read_waypoints(filename)

# Ensure waypoints are not empty before plotting
if waypoints:
    plot_path(waypoints)
else:
    print("No valid waypoints found in the file.")
