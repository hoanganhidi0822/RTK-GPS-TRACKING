import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Import 3D toolkit
from yamspy import MSPy
import math

# Define the serial port and baud rate
SERIAL_PORT = "COM4"
BAUD_RATE = 115200

# Initialize position and velocity
x, y, z = 0, 0, 0  # Initial position (m)
vx, vy, vz = 0, 0, 0  # Initial velocity (m/s)
positions = []  # To store positions for plotting

# Time step (for integration)
dt = 0.01  # 10 ms (sampling at 100 Hz)

def yaw_to_rotation_matrix(yaw):
    """Convert yaw angle to a 2D rotation matrix."""
    return np.array([[np.cos(yaw), -np.sin(yaw)],
                     [np.sin(yaw),  np.cos(yaw)]])

def calculate_linear_position(accel_local, yaw, dt, prev_vx, prev_vy, prev_vz, prev_x, prev_y, prev_z):
    # Convert local accelerations to world frame
    rotation_matrix = yaw_to_rotation_matrix(yaw)
    accel_world = np.dot(rotation_matrix, accel_local[:2])  # Only use x and y

    # Integrate acceleration to update velocity
    vx = prev_vx + accel_world[0] * dt 
    vy = prev_vy + accel_world[1] * dt
    vz = prev_vz  # Keep Z velocity constant or update it as per your needs

    # Integrate velocity to update position
    x = prev_x + vx * dt * 100
    y = prev_y + vy * dt * 100
    z = prev_z + vz * dt * 100  # Update Z position if needed

    return x, y, z, vx, vy, vz

def draw_frame(ax, position, size=0.5):
    """Draw a frame around the current position."""
    x, y, z = position
    
    # Define the corners of the frame based on the current position and size
    frame_corners = np.array([[x - size, y - size, z - size],
                               [x + size, y - size, z - size],
                               [x + size, y + size, z - size],
                               [x - size, y + size, z - size],
                               [x - size, y - size, z + size],
                               [x + size, y - size, z + size],
                               [x + size, y + size, z + size],
                               [x - size, y + size, z + size]])

    # Define edges for the frame
    edges = [
        [frame_corners[0], frame_corners[1]],
        [frame_corners[1], frame_corners[2]],
        [frame_corners[2], frame_corners[3]],
        [frame_corners[3], frame_corners[0]],
        [frame_corners[4], frame_corners[5]],
        [frame_corners[5], frame_corners[6]],
        [frame_corners[6], frame_corners[7]],
        [frame_corners[7], frame_corners[4]],
        [frame_corners[0], frame_corners[4]],
        [frame_corners[1], frame_corners[5]],
        [frame_corners[2], frame_corners[6]],
        [frame_corners[3], frame_corners[7]],
    ]

    # Plot the edges
    for edge in edges:
        ax.plot(*zip(*edge), color='orange', linewidth=2)

def read_imu_and_plot_position():
    global x, y, z, vx, vy, vz
    
    try:
        # Initialize connection to the flight controller
        with MSPy(device=SERIAL_PORT, loglevel='ERROR', baudrate=BAUD_RATE) as board:
            if board == 1:
                print("Error: Unable to connect to the flight controller.")
                return None
            
            print("Connected to the flight controller. Reading IMU data...")

            # Initialize plot
            plt.ion()  # Interactive mode for real-time plotting
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.set_title("3D Position Plot (Right-Hand Rule)")
            ax.set_xlabel("X Position (cm)", color='green')
            ax.set_ylabel("Y Position (cm)", color='red')
            ax.set_zlabel("Z Position (cm)", color='blue')
            ax.set_xlim([-10, 10])
            ax.set_ylim([-10, 10])
            ax.set_zlim([-10, 10])
            
            # Draw the axes
            ax.quiver(0, 0, 0, 10, 0, 0, color='green', linewidth=2, label='X-axis')  # X-axis
            ax.quiver(0, 0, 0, 0, 10, 0, color='red', linewidth=2, label='Y-axis')    # Y-axis
            ax.quiver(0, 0, 0, 0, 0, 10, color='blue', linewidth=2, label='Z-axis')   # Z-axis
            
            # Continuous loop for reading and plotting IMU data
            while True:
                # Send request to read IMU data
                board.fast_read_imu()

                # Get sensor data
                accel = board.SENSOR_DATA.get('accelerometer', None)
                board.fast_read_attitude()  # Reading attitude data (yaw, pitch, roll)
                yaw_data = board.SENSOR_DATA.get('kinematics', None)  # Assuming 'attitude' contains yaw

                if accel and yaw_data:
                    yaw = math.radians(yaw_data[2])  # Convert yaw to radians
                    accel_local = np.array(accel) * 9.81  # Convert accel to m/s²

                    # Calculate new position and velocity
                    x, y, z, vx, vy, vz = calculate_linear_position(accel_local, yaw, dt, vx, vy, vz, x, y, z)

                    # Store the position for plotting
                    positions.append([x, y, z])

                    # Update plot data
                    pos_array = np.array(positions)
                    ax.scatter(pos_array[:, 0], pos_array[:, 1], pos_array[:, 2], color='orange')  # Current position
                    ax.plot(pos_array[:, 0], pos_array[:, 1], pos_array[:, 2], color='orange')  # Path

                    # Draw the frame around the current position
                    ax.cla()  # Clear the current axes
                    ax.set_title("3D Position Plot (Right-Hand Rule)")
                    ax.set_xlabel("X Position (cm)", color='green')
                    ax.set_ylabel("Y Position (cm)", color='red')
                    ax.set_zlabel("Z Position (cm)", color='blue')
                    ax.set_xlim([-10, 10])
                    ax.set_ylim([-10, 10])
                    ax.set_zlim([-10, 10])
                    
                    # Redraw the axes
                    ax.quiver(0, 0, 0, 10, 0, 0, color='green', linewidth=2, label='X-axis')  # X-axis
                    ax.quiver(0, 0, 0, 0, 10, 0, color='red', linewidth=2, label='Y-axis')    # Y-axis
                    ax.quiver(0, 0, 0, 0, 0, 10, color='blue', linewidth=2, label='Z-axis')   # Z-axis
                    
                    # Draw the frame
                    draw_frame(ax, (x, y, z), size=0.5)

                    # Set plot limits dynamically based on data
                    ax.set_xlim(np.min(pos_array[:, 0]) - 1, np.max(pos_array[:, 0]) + 1)
                    ax.set_ylim(np.min(pos_array[:, 1]) - 1, np.max(pos_array[:, 1]) + 1)
                    ax.set_zlim(np.min(pos_array[:, 2]) - 1, np.max(pos_array[:, 2]) + 1)

                    # Redraw plot
                    plt.draw()
                    plt.pause(0.01)  # Small pause to update the plot window

                else:
                    print("No valid data received from IMU.")

                time.sleep(dt)

    except Exception as e:
        print(f"An error occurred: {e}")
        return None

if __name__ == "__main__":
    # Start reading IMU data and plotting position
    read_imu_and_plot_position()
