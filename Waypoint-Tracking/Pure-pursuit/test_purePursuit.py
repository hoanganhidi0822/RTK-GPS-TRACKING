import numpy as np
import matplotlib.pyplot as plt
import GPS_module


class PurePursuit:
    def __init__(self, lookahead_distance):
        self.lookahead_distance = lookahead_distance

    def get_target_index(self, cx, cy, current_position):
        cx_array = np.array(cx)
        cy_array = np.array(cy)

        # Calculate the distances from the current position to all waypoints
        distances = np.sqrt((cx_array - current_position[0]) ** 2 + (cy_array - current_position[1]) ** 2)

        # Find the first waypoint that is beyond the lookahead distance
        target_index = np.where(distances >= self.lookahead_distance)[0]

        if target_index.size == 0:
            return None  # No target waypoint within lookahead distance
        return target_index[0]

    def calculate_steering_angle(self, current_position, target_position):
        # Calculate the angle to the target point
        angle_to_target = np.arctan2(target_position[1] - current_position[1], target_position[0] - current_position[0])
        return angle_to_target

    def find_lookahead_point(self, cx, cy, current_position):
        cx_array = np.array(cx)
        cy_array = np.array(cy)

        # Calculate the distances from the current position to all waypoints
        distances = np.sqrt((cx_array - current_position[0]) ** 2 + (cy_array - current_position[1]) ** 2)

        # Find the index of the waypoint just before the lookahead distance
        for i in range(len(distances) - 1):
            if distances[i] <= self.lookahead_distance < distances[i + 1]:
                # Interpolate between the current and next waypoint to find the exact lookahead point
                ratio = (self.lookahead_distance - distances[i]) / (distances[i + 1] - distances[i])
                lookahead_x = cx[i] + ratio * (cx[i + 1] - cx[i])
                lookahead_y = cy[i] + ratio * (cy[i + 1] - cy[i])
                return lookahead_x, lookahead_y

        return None  # No valid lookahead point found

def load_waypoints(filename):
    cx = []  # List for x-coordinates (meters)
    cy = []  # List for y-coordinates (meters)
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()  
            if line:
                try:
                    lat, lon = map(float, line.split(','))
                    # Convert lat/lon to meters
                    x = lon * 111320 * np.cos(lat * np.pi / 180)  # Longitude to x in meters
                    y = lat * 111320  # Latitude to y in meters
                    cx.append(x)  
                    cy.append(y)  
                except ValueError as e:
                    print(f"Error parsing line '{line}': {e}")  
    return cx, cy

def get_gps_data():
    # Simulate GPS data for the current position (this should be replaced by actual GPS readings)
    lat, lon, sat_count, alt = GPS_module.get_gps_data('/dev/ttyUSB0', baudrate=115200)
    
    x = float(lon) * 111320 * np.cos(float(lat) * np.pi / 180)  
    y = float(lat) * 111320  
    return (x, y)

def plot_navigation(ax, cx, cy, current_position, lookahead_position):
    ax.clear()  # Clear the axis for updating the plot

    # Plot full waypoint map
    ax.plot(cx, cy, 'b-', label='Waypoints')  # Plot waypoints as a connected blue line

    # Plot the current position in green
    ax.plot(current_position[0], current_position[1], 'go', label='Current Position', markersize=5)

    # Plot the lookahead point (calculated) in red
    if lookahead_position is not None:
        ax.plot(lookahead_position[0], lookahead_position[1], 'rv', label='Lookahead Point', markersize=5)

    # Set zoom limits to focus on the area around the current position
    zoom_size = 200  # meters for zoom window
    ax.set_xlim([current_position[0] - zoom_size, current_position[0] + zoom_size])
    ax.set_ylim([current_position[1] - zoom_size, current_position[1] + zoom_size])

    # Set labels and grid
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title('Pure Pursuit Navigation')
    ax.legend()
    ax.grid()
    ax.axis('equal')

    plt.pause(0.1)  # Pause to update the plot

def main():
    # Load waypoints from the file
    cx, cy = load_waypoints('/media/hoanganh/New Volume/Documents/Researches/2024_Project/RTK_GPS/HD_MAP/HD_MAP1.txt')

    # Initialize Pure Pursuit with a lookahead distance
    lookahead_distance = 10  # Lookahead distance of 5 meters
    pure_pursuit = PurePursuit(lookahead_distance)

    # Set up the plotting environment
    plt.ion()  # Enable interactive mode for live plotting
    fig, ax = plt.subplots(figsize=(10, 6))

    # Plot the initial full waypoint map
    ax.plot(cx, cy, 'b-', label='Waypoints')  
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title('Pure Pursuit Navigation')
    ax.legend()
    ax.grid()
    ax.axis('equal')

    while True:
        # Get the current GPS position
        current_position = get_gps_data()

        # Find the lookahead point using interpolation
        lookahead_position = pure_pursuit.find_lookahead_point(cx, cy, current_position)

        if lookahead_position is not None:
            steering_angle = pure_pursuit.calculate_steering_angle(current_position, lookahead_position)
            print(f"Current Position: {current_position}, Lookahead Position: {lookahead_position}, Steering Angle: {steering_angle}")
        else:
            print("No valid lookahead point found.")

        # Update the plot with current position and lookahead point
        plot_navigation(ax, cx, cy, current_position, lookahead_position)

        plt.pause(0.1)  # Pause to update the plot dynamically

if __name__ == '__main__':
    print("Starting Pure Pursuit Navigation...")
    main()
