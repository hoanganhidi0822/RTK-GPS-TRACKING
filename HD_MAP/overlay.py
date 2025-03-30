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
    return waypoints

# Function to plot waypoints as a path with a specified color and label
def plot_path(waypoints, color, label):
    if not waypoints:
        print(f"No waypoints to plot for {label}.")
        return
    
    latitudes = [point[0] for point in waypoints]
    longitudes = [point[1] for point in waypoints]

    # Plot the points with specific color and label
    plt.plot(longitudes, latitudes, marker='o', markersize=0, linestyle='-', color=color, label=label)

# Example usage
filename1 = 'UTE_MAP\MAP_THUAN\HD_MAP1.txt'  # First file with waypoints
filename2 = 'UTE_MAP\MAP_THUAN\HD_MAP2.txt'  # Second file with waypoints
filename3 = 'UTE_MAP\MAP_NGHICH\HD_MAP1.txt'  # Third file with waypoints
filename4 = 'UTE_MAP\MAP_NGHICH\HD_MAP2.txt'  # Third file with waypoints

# Read waypoints from all three files
waypoints1 = read_waypoints(filename1)
waypoints2 = read_waypoints(filename2)
waypoints3 = read_waypoints(filename3)
waypoints4 = read_waypoints(filename4)

# Create the plot
plt.figure(figsize=(10, 6))

# Plot the three paths with different colors
plot_path(waypoints1, 'b', 'Path 1')  # Blue for path 1
plot_path(waypoints2, 'r', 'Path 2')  # Red for path 2
plot_path(waypoints3, 'g', 'Path 3')  # Green for path 3
plot_path(waypoints4, 'y', 'Path 4')  # Green for path 3

# Add labels, title, and legend
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('Paths from Three Sets of Waypoints')
plt.legend()

# Add grid and show the plot
plt.grid(True)
plt.show()
