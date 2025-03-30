import serial
import numpy as np
import math
import time

def dec2deg(value):
    if not value:
        return None
    dec = value / 100.00
    deg = int(dec)
    minutes = (dec - deg) * 100 / 60
    position = deg + minutes
    position = "{:.10f}".format(position)  # Higher precision
    return position

def wait_for_com_port(port):
    while True:
        try:
            ser = serial.Serial(port, baudrate=115200, timeout=0.1)
            ser.close()
            return
        except serial.SerialException:
            print(f"Port {port} not available yet, waiting...")
            time.sleep(1)

def get_gps_data(port, baudrate=115200):
    wait_for_com_port(port)
    ser = serial.Serial(port, baudrate, timeout=0.1)
    while True:
        gps_data = ser.readline().decode("utf8")
        if gps_data:
            try:
                gps_data = gps_data.split(',')
                if "$GPGGA" in gps_data[0]:
                    if len(gps_data) > 9:
                        lat = dec2deg(float(gps_data[2])) if gps_data[2] else None
                        lon = dec2deg(float(gps_data[4])) if gps_data[4] else None
                        alt = gps_data[9] if gps_data[9] else None
                        sat_count = gps_data[7] if gps_data[7] else None
                        ser.close()
                        return lat, lon, sat_count, alt
            except Exception as e:
                print("Error processing GPS data:", e)

def latlon_to_xy(lat, lon, ref_lat, ref_lon):
    """
    Approximate conversion from lat/lon to Cartesian x, y coordinates.
    This is a simple conversion assuming small area (local flat Earth assumption).
    """
    # Earth's radius in meters
    R = 6371000  

    # Convert degrees to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)

    # Approximate conversion using small angle formulas
    d_lat = lat_rad - ref_lat_rad
    d_lon = lon_rad - ref_lon_rad

    x = R * d_lon * math.cos((lat_rad + ref_lat_rad) / 2)
    y = R * d_lat

    return x, y

class PurePursuitKinematics:
    def __init__(self, wheelbase, lookahead_distance, gps_port, ref_lat, ref_lon):
        self.wheelbase = wheelbase
        self.lookahead_distance = lookahead_distance
        self.waypoints = None
        self.current_pos = None
        self.current_heading = None
        self.goal_idx = 0
        self.gps_port = gps_port
        self.ref_lat = ref_lat  # Reference latitude for x, y conversion
        self.ref_lon = ref_lon  # Reference longitude for x, y conversion

    def update_waypoints(self, waypoints):
        self.waypoints = waypoints

    def update_position_from_gps(self):
        """
        Get GPS data and convert it to x, y coordinates.
        """
        lat, lon, sat_count, alt = get_gps_data(self.gps_port)
        if lat and lon:
            lat = float(lat)
            lon = float(lon)
            self.current_pos = np.array(latlon_to_xy(lat, lon, self.ref_lat, self.ref_lon))

    def update_position(self, current_pos, current_heading):
        self.current_pos = np.array(current_pos)
        self.current_heading = current_heading

    def calculate_steering_angle(self):
        if self.waypoints is None or self.current_pos is None or self.current_heading is None:
            return None  # Missing data to calculate

        # Find the goal point (lookahead target)
        while self.goal_idx < len(self.waypoints) - 1:
            distance_to_waypoint = np.linalg.norm(self.waypoints[self.goal_idx] - self.current_pos)
            if distance_to_waypoint > self.lookahead_distance:
                break
            self.goal_idx += 1

        if self.goal_idx >= len(self.waypoints):
            return None  # End of path

        # Calculate the angle to the target point in the local car frame
        target = self.waypoints[self.goal_idx]
        dx = target[0] - self.current_pos[0]
        dy = target[1] - self.current_pos[1]

        local_x = dx * math.cos(-self.current_heading) - dy * math.sin(-self.current_heading)
        local_y = dx * math.sin(-self.current_heading) + dy * math.cos(-self.current_heading)

        if local_x == 0:
            return 0  # Avoid division by zero if directly on the waypoint
        curvature = (2 * local_y) / (local_x**2 + local_y**2)
        steering_angle = math.atan(curvature * self.wheelbase)

        steering_angle_limit = math.radians(30)
        steering_angle = max(-steering_angle_limit, min(steering_angle, steering_angle_limit))

        return steering_angle

    def update_kinematics(self, v, steering_angle, dt):
        if self.current_pos is None or self.current_heading is None:
            return None  # Need initial state to update

        x, y = self.current_pos
        theta = self.current_heading

        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += (v / self.wheelbase) * math.tan(steering_angle) * dt

        theta = (theta + np.pi) % (2 * np.pi) - np.pi

        self.current_pos = np.array([x, y])
        self.current_heading = theta

# Example usage:

waypoints_file = 'gps_data_map.txt'
waypoints = load_waypoints(waypoints_file)

# GPS port for reading data
gps_port = 'COM3'  # Change to your GPS port
ref_lat, ref_lon = 10.0, 10.0  # Reference latitude and longitude

wheelbase = 2.5
lookahead_distance = 5.0
controller = PurePursuitKinematics(wheelbase, lookahead_distance, gps_port, ref_lat, ref_lon)
controller.update_waypoints(waypoints)

initial_heading = 0.0

v = 2.0
dt = 0.1

for _ in range(1000):
    controller.update_position_from_gps()
    steering_angle = controller.calculate_steering_angle()
    if steering_angle is not None:
        print(f"Steering angle: {math.degrees(steering_angle)} degrees")
    controller.update_kinematics(v, steering_angle, dt)
    current_position = controller.current_pos
    current_heading = controller.current_heading
    print(f"Position: {current_position}, Heading: {math.degrees(current_heading)} degrees")
