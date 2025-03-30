import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point
from shapely.ops import nearest_points

# Function to calculate intersection points between path and lookahead circle
def find_intersection(circle_center, lookahead_distance, path_points):
    circle = Point(circle_center).buffer(lookahead_distance).boundary
    path = LineString(path_points)
    
    # Find the intersection points
    intersection = path.intersection(circle)
    
    # Check if we got points or not
    if intersection.is_empty:
        return None
    elif intersection.geom_type == 'Point':
        return [intersection]
    elif intersection.geom_type == 'MultiPoint':
        return list(intersection.geoms)  # Use .geoms to get individual points
    else:
        return None

# Create a path (set of waypoints)
# Simulated waypoints for the demonstration
path_points = np.array([[0, 0], [10, 5], [20, 10], [30, 10], [40, 15], [45, 20],
                        [50, 5], [55, 10], [60, 15], [65, 20], [70, 25], [80, 50], [100,50]])

# Car's position
car_position = np.array([60, 20])

# Lookahead distance
lookahead_distance = 7

# Find intersection points
intersection_points = find_intersection(car_position, lookahead_distance, path_points)

# Calculate the distance from the car to the path and find the nearest point
car_point = Point(car_position)
path_line = LineString(path_points)
nearest_point_on_path = nearest_points(car_point, path_line)[1]  # Get the nearest point on the path
distance_to_path = car_point.distance(path_line)
print(f"Distance from car to path: {distance_to_path:.2f}")

# Calculate distances from car to all waypoints
distances = np.linalg.norm(path_points - car_position, axis=1)

# Select the indices of the 10 nearest waypoints
nearest_indices = np.argsort(distances)[:10]

# Get the nearest waypoints
nearest_waypoints = path_points[nearest_indices]

# Create a new path from the nearest waypoints
new_path_line = LineString(nearest_waypoints)

# Plotting the path, car, and circle
fig, ax = plt.subplots()

# Plot original path (all waypoints)
path_x, path_y = path_points[:, 0], path_points[:, 1]
ax.plot(path_x, path_y, label="Original Path", color="lightblue", linestyle='--')

# Plot nearest waypoints (only the selected ones)
ax.scatter(nearest_waypoints[:, 0], nearest_waypoints[:, 1], label="Nearest Waypoints", color="blue", zorder=3)

# Plot car
ax.plot(car_position[0], car_position[1], 'ro', label="Car")

# Plot lookahead circle
circle = plt.Circle(car_position, lookahead_distance, color='green', fill=False, label="Lookahead Circle")
ax.add_patch(circle)

# Plot intersection points
if intersection_points:
    first = True
    for point in intersection_points:
        if first:
            ax.plot(point.x, point.y, 'go', label="Intersection Point")
            first = False
        else:
            ax.plot(point.x, point.y, 'go')  # No label for subsequent points

# Plot the nearest point on the path
ax.plot(nearest_point_on_path.x, nearest_point_on_path.y, 'bo', label="Nearest Point on Path")

# Draw a line from the car to the nearest point on the path
ax.plot([car_position[0], nearest_point_on_path.x], [car_position[1], nearest_point_on_path.y], 'r--', label="Line to Path")

# Labels and legends
ax.set_aspect('equal', 'box')
ax.set_xlabel('X')
ax.set_ylabel('Y')
plt.legend()
plt.grid(True)
plt.title(f'Path, Car, Lookahead Circle, and Intersection Points\nDistance to path: {distance_to_path:.2f}')
plt.show()
