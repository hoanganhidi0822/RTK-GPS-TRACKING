import time
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from shapely.geometry import LineString, Point
import math
import config as cf
import IMU_F722
import GPS_module 
from utils.communication import STM32
#pythofrom GPS_module import get_gps_data
stm32 = STM32(port="COM3", baudrate=115200)
# Import your communication module here
from utils.communication import STM32
import cv2

lat, lon, sat_count, alt = None, None, None, None

# Define waypoints (latitude, longitude)
Waypoints = [
    [106.7715131967, 10.8532570333],
    [106.771513195, 10.8532568817],
    [106.7715122883, 10.8532464083],
    [106.7715102383, 10.8532280683],
    [106.7715070017, 10.8532027583],
    [106.77150365, 10.8531766583],
    [106.771500395, 10.853148555],
    [106.77149734, 10.8531191367],
    [106.7714936983, 10.85308817],
    [106.7714901683, 10.8530571067],
    [106.7714868067, 10.853026475],
    [106.77148359, 10.852995845],
    [106.7714788667, 10.8529635317],
    [106.7714765083, 10.85293495],
    [106.7714735633, 10.8529073683],
    [106.771470075, 10.8528799567],
    [106.7714652117, 10.8528484533],
    [106.7714605967, 10.8528135617],
    [106.7714563483, 10.8527778917],
    [106.7714543233, 10.8527395917],
    [106.7714501817, 10.852705015],
    [106.7714465617, 10.852672385],
    [106.77144277, 10.8526424983],
    [106.7714400833, 10.852615755],
    [106.7714367567, 10.852587345],
    [106.7714331367, 10.8525562717],
    [106.7714299233, 10.8525264117],
    [106.7714260517, 10.8524979233],
    [106.7714221517, 10.852468695],
    [106.7714181583, 10.85243665],
    [106.7714148, 10.852405225],
    [106.7714113783, 10.8523761733],
    [106.7714085867, 10.8523497233],
    [106.7714055483, 10.852321395],
    [106.7714021667, 10.8522952617],
    [106.771399845, 10.8522662467],
    [106.7713963667, 10.8522371033],
    [106.77139381, 10.85220912],
    [106.771390455, 10.852179605],
    [106.7713872967, 10.8521494567],
    [106.7713834983, 10.8521173133],
    [106.77138042, 10.852086915],
    [106.77137685, 10.8520551183],
    [106.7713729367, 10.8520218617],
    [106.7713691167, 10.8519898233],
    [106.77136613, 10.8519576667],
    [106.7713624, 10.8519242833],
    [106.7713597117, 10.8519019617],
    [106.7713597083, 10.85190204],
    [106.7713597383, 10.8519020583],
    [106.7713597717, 10.851901735],
    [106.7713593617, 10.8518956233],
    [106.77135934, 10.8518947383],
    [106.771359335, 10.851894725],
    [106.7713591233, 10.8518924567],
    [106.7713586383, 10.8518893367],
    [106.7713585967, 10.85188776],
    [106.7713582983, 10.851885475],
    [106.7713575883, 10.8518803483],
    [106.7713558067, 10.8518662017],
    [106.7713530083, 10.8518440067],
    [106.7713489467, 10.851815],
    [106.77134546, 10.8517857867],
    [106.7713417317, 10.8517601983],
    [106.7713385467, 10.8517336317],
    [106.7713349933, 10.8517072767],
    [106.7713309783, 10.851681245],
    [106.7713276067, 10.8516507067],
    [106.7713251633, 10.8516202883],
    [106.7713216417, 10.85159013],
    [106.7713180617, 10.8515568933],
    [106.7713153583, 10.8515197683],
    [106.77131881, 10.8514841317],
    [106.7713360217, 10.85145304],
    [106.7713654317, 10.85143012],
    [106.771400385, 10.8514144383],
    [106.7714367867, 10.8514025483],
    [106.7714701417, 10.8513938083],
    [106.7715027067, 10.85138352],
    [106.7715415817, 10.8513719783],
    [106.771583575, 10.85136075],
    [106.7716233067, 10.8513495317],
    [106.7716625033, 10.8513386767],
    [106.771702075, 10.851327855],
    [106.7717418583, 10.8513168233],
    [106.7717801967, 10.8513066933],
    [106.7718174033, 10.8512989033],
    [106.77185127, 10.8512941433],
    [106.771895355, 10.85128915],
    [106.7719407383, 10.8512840417],
    [106.77198257, 10.8512790567],
    [106.772020535, 10.851274945],
    [106.7720688167, 10.85126932],
    [106.77212178, 10.8512632683],
    [106.7721686533, 10.8512576917],
    [106.772207795, 10.8512536567],
    [106.7722565367, 10.8512487233],
    [106.7723154383, 10.85124161],
    [106.7723781567, 10.8512350533],
    [106.7724370883, 10.8512277517],
    [106.77249073, 10.8512212167],
    [106.772540975, 10.8512159683],
    [106.7725878717, 10.85121476],
    [106.772632395, 10.85122463],
    [106.7726701017, 10.851251045],
    [106.7726931217, 10.8512932217],
    [106.7727020283, 10.851342255],
    [106.772707105, 10.8513850917],
    [106.7727122583, 10.8514276433],
    [106.7727176, 10.8514750083],
    [106.77272351, 10.85152259],
    [106.7727266783, 10.85155464],
    [106.7727298883, 10.8515861183],
    [106.7727357233, 10.8516324533],
    [106.7727420783, 10.8516883017],
    [106.7727493683, 10.8517473683],
    [106.7727561667, 10.8518054867],
    [106.772763225, 10.8518595783],
    [106.77276753, 10.8519070933],
    [106.7727717967, 10.8519509083],
    [106.7727780833, 10.85200011],
    [106.772784785, 10.85205085],
    [106.7727895967, 10.8520976583],
    [106.772795585, 10.8521467017],
    [106.772803215, 10.8522013383],
    [106.7728105267, 10.8522575817],
    [106.77281802, 10.8523159067],
    [106.7728246117, 10.85237428],
    [106.772830795, 10.8524320817],
    [106.7728368233, 10.8524879583],
    [106.772842715, 10.852541695],
    [106.7728485467, 10.8525919167],
    [106.7728528967, 10.8526378167],
    [106.772855565, 10.8526669383],
    [106.7728594583, 10.8526987017],
    [106.7728660617, 10.852744235],
    [106.772872385, 10.852798375],
    [106.772878185, 10.8528527417],
    [106.7728840083, 10.852909235],
    [106.7728892633, 10.85296119],
    [106.7728975983, 10.8530155633],
    [106.7729047217, 10.8530711833],
    [106.7729126767, 10.85311981],
    [106.77291746, 10.8531632633],
    [106.7729100333, 10.85319963],
    [106.7728913567, 10.8532257983],
    [106.7728539967, 10.85323862],
    [106.7728090733, 10.8532433633],
    [106.7727675983, 10.8532483767],
    [106.7727245967, 10.8532545567],
    [106.7726813633, 10.8532593283],
    [106.7726348817, 10.8532644983],
    [106.7725912283, 10.85327602],
    [106.772564685, 10.8533095617],
    [106.7725641117, 10.8533561183],
    [106.7725696883, 10.8534032783],
    [106.77257489, 10.8534526183],
    [106.7725772033, 10.85350023],
    [106.77255707, 10.85353994],
    [106.7725140183, 10.8535578183],
    [106.7724633717, 10.85356742],
    [106.772411205, 10.85357779],
    [106.7723590483, 10.8535867383],
    [106.772305735, 10.8535975283],
    [106.7722500217, 10.8536056483],
    [106.7721963817, 10.8536143933],
    [106.77214861, 10.8536207267],
    [106.77209963, 10.853626295],
    [106.7720539817, 10.85363201],
    [106.7720110933, 10.8536366117],
    [106.771959875, 10.8536417083],
    [106.7719166233, 10.853646455],
    [106.7718854583, 10.8536522283],
    [106.77186075, 10.8536557783],
    [106.771816305, 10.8536609133],
    [106.7717686917, 10.8536666117],
    [106.7717397017, 10.8536696767],
    [106.7717143733, 10.8536719633],
    [106.7716906133, 10.85367422],
    [106.7716518283, 10.8536796133],
    [106.7716115517, 10.853684625],
    [106.771579355, 10.8536790633],
    [106.7715599783, 10.85365947],
    [106.77154891, 10.8536273383],
    [106.771545215, 10.8535886017],
    [106.771540315, 10.85354635],
    [106.77153206, 10.8535041317],
    [106.7715265167, 10.8534620183],
    [106.7715223, 10.8534188233],
    [106.7715176917, 10.8533757467],
    
    
]

cf.bearing = None
# cf.latitude = None
# cf.longitude = None

# Base reference for conversion
base_latitude = Waypoints[0][1]  # Latitude of the first waypoint
base_longitude = Waypoints[0][0]  # Longitude of the first waypoint
scaling_factor = 100000  # Scale factor for converting degrees to meters (example)

# PID control parameters
pre_t = time.time()
error_arr = np.zeros(5)
brake = 0
def PID(error, p, i, d):
    global pre_t, error_arr 
    # Shift and store error history
    error_arr[1:] = error_arr[:-1]
    error_arr[0] = error 
    # Calculate Proportional term
    P = error * p
    # Calculate delta time
    delta_t = time.time() - pre_t
    pre_t = time.time() 
    # Calculate Integral term
    I = np.sum(error_arr) * delta_t * i
    # Calculate Derivative term (if error_arr[1] exists)
    if delta_t > 0:
        D = (error - error_arr[1]) / delta_t * d
    else:
        D = 0
    # Compute the total PID output
    angle = P + I + D
    # Apply output limit
    if abs(angle) > 30:
        angle = np.sign(angle) * 30
    
    return float(angle)

def course_to_waypoint(current_lat, current_long, target_lat, target_long):
    """Calculate the heading to the target waypoint."""
    dlon = math.radians(float(target_long) - float(current_long))
    c_lat = math.radians(float(current_lat))
    t_lat = math.radians(float(target_lat))
    a1 = math.sin(dlon) * math.cos(t_lat)
    a2 = math.sin(c_lat) * math.cos(t_lat) * math.cos(dlon)
    a2 = math.cos(c_lat) * math.sin(t_lat) - a2
    a2 = math.atan2(a1, a2)
    if a2 < 0.0:
        a2 += math.pi * 2
    return math.degrees(a2)

def lat_lon_to_xy(lat, lon):
    """Convert latitude and longitude to Cartesian coordinates (X, Y)."""
    x = (lon - base_longitude) * scaling_factor
    y = (lat - base_latitude) * scaling_factor
    return x, y

def xy_to_lat_lon(x, y):
    """Convert Cartesian coordinates (X, Y) back to latitude and longitude."""
    lon = (x / scaling_factor) + base_longitude
    lat = (y / scaling_factor) + base_latitude
    return lat, lon

def waypoints_to_xy(waypoints):
    """Convert all waypoints from lat/lon to X/Y."""
    return np.array([lat_lon_to_xy(lat, lon) for lon, lat in waypoints])


def find_intersection(circle_center, lookahead_distance, path_points):
    """Find intersection points between path and a lookahead circle."""
    circle = Point(circle_center).buffer(lookahead_distance).boundary
    path = LineString(path_points)
    intersection = path.intersection(circle)
    
    # Return intersection points based on geometry type
    if intersection.is_empty:
        return None
    elif intersection.geom_type == 'Point':
        return [intersection]  # Single point
    elif intersection.geom_type == 'MultiPoint':
        return list(intersection.geoms)  # Multiple points
    else:
        return None  # Unexpected geometry type

wheelbase = 1.7  # Length of the vehicle (for steering calculations)

# Function to rotate a point around another point
def rotate_point(point, angle, origin=(0, 0)):
    """Rotate point around the origin by a given angle."""
    angle_rad = math.radians(angle)
    ox, oy = origin
    px, py = point.x, point.y

    qx = ox + math.cos(angle_rad) * (px - ox) - math.sin(angle_rad) * (py - oy)
    qy = oy + math.sin(angle_rad) * (px - ox) + math.cos(angle_rad) * (py - oy)
    
    return Point(qx, qy)


def main():
    steering_angles = []  # List to store steering angles
    times = []  # List to store time values for plotting

    start_time = time.time()  # Start time for plotting x-axis
    count = 0 
    #with open("gps_data.txt", "a") as file:
    while True:
        ##-- Car State --##  
        # Replace with actual GPS reading
        #car_heading = cf.bearing
        # print(f"Heading: {car_heading}")
        #lon, lat = 106.771413, 10.851485  # Example position; replace with actual GPS reading
        #lat, lon = 106.7715131967, 10.8532570333
        lat, lon, car_heading, sat_count = GPS_module.get_gps_data(port = "COM17", baudrate  = 115200)
        # file.write(f"{lat}, {lon}\n") 
        #print(lat)
        # print(car_heading)
        car_heading = car_heading + 2
        print(f"heading: {car_heading}")
        # Convert lat, lon to x, y for the car's position
        car_x, car_y = lat_lon_to_xy(float(lat), float(lon))
        path_points = waypoints_to_xy(Waypoints)
        # print(f"dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd: {lat, lon}")
        # Find intersection points
        lookahead_distance =  3.8 # Adjust as needed
        intersection_points = find_intersection((car_x, car_y), lookahead_distance, path_points)
        # print(intersection_points)
        if intersection_points and car_heading is not None:
            if len(intersection_points) == 2:
                front_intersection = None
                distances = [point.distance(Point(car_x, car_y)) for point in intersection_points]
                
                for point in intersection_points:
                    # Compute the course angle to the intersection point (lat/lon)
                    point_lat, point_lon = xy_to_lat_lon(point.x, point.y)
                    course_angle = course_to_waypoint(lat, lon, point_lat, point_lon)
                    
                    # Compare course angle with the car's heading angle
                    angle_diff = abs(course_angle - car_heading)
                    
                    # Normalize the difference to the range 0-180 degrees
                    if angle_diff > 180:
                        angle_diff = 360 - angle_diff
                    # Select the point if it's within a 90-degree field of view in front of the car
                    if angle_diff < 70:
                        front_intersection = point
                        break

                if front_intersection is None:
                    # If no point is in front, choose the closest one
                    front_intersection = intersection_points[np.argmin(distances)]
            else:
                # If only one intersection point exists, use it
                front_intersection = intersection_points[0]
            
            
            lookahead_lat, lookahead_lon = xy_to_lat_lon(front_intersection.x, front_intersection.y)
            car_point = Point(car_x, car_y)
            distance_to_path = car_point.distance(LineString(path_points))

            # Calculate the course to the waypoint
            alpha = course_to_waypoint(lat, lon, lookahead_lat, lookahead_lon) - car_heading
            alpha = (alpha + 180) % 360 - 180  # Normalize to [-180, 180]
            alpha = math.radians(alpha)
        
            ##### TESTING #######
            
            # error = distance_to_path
            # steering = PID(error=error, p=0.01, i=0.00, d=0.001)
            # # Calculate steering angle
            # steering_angle = (math.atan((2 * L * math.sin(alpha))) / lookahead_distance) * 100 + steering
            
            error = alpha
            pid_alpha = PID(error, p = 1.0, i=0, d = 0.2)
            # print(pid_alpha)
            # Calculate steering angle
            steering_angle = (math.atan((2 * wheelbase * math.sin(error))) / lookahead_distance) * 100 

            #steering_angles.append(steering_angle)
            current_time = time.time() - start_time
            times.append(current_time)
            print(steering_angle)
            count += 1
            if count == 1:
                #steering_angle = PID(error=error, p=0.3, i=0.01, d=0.02) # > 0 Right, < 0 Left
                stm32(angle= int(steering_angle), speed=0, brake_state=0)
                count = 0    

            print(f"Dis2P: {distance_to_path:.2f}, Alpha: {alpha:.2f}, Steering Angle: {steering_angle:.2f}")
            lookahead_point = Point(front_intersection.x, front_intersection.y)
            line_between = LineString([(car_x, car_y), (lookahead_point.x, lookahead_point.y)])

            # # Example: plot the results
            # plt.figure()
            # plt.xlim(car_x - 20, car_x + 20)
            # plt.ylim(car_y - 20, car_y + 20)

            # filtered_path_points = [point for point in path_points if Point(point).distance(car_point) <= 50]

            # if len(filtered_path_points) > 1:
            #     path_line = LineString(filtered_path_points)
            #     x, y = path_line.xy
            #     plt.plot(x, y, '--', label="Path Line", color="lightblue")

            # for point in filtered_path_points:
            #     plt.plot(point[0], point[1], 'yo')
            # plt.plot(car_x, car_y, 'ro', label="Car Position")
            # plt.plot(front_intersection.x, front_intersection.y, 'go', label="Look ahead Point")

            # x_line, y_line = line_between.xy
            # plt.plot(x_line, y_line, 'b-', label="Line to Lookahead Point")

            # plt.xlabel('X (meters)')
            # plt.ylabel('Y (meters)')
            # plt.title('Car Position and Nearby Waypoint Path')
            # plt.legend()
            # plt.grid()
            # plt.axis('equal')
            
            # # Save the plot as an image
            # plt.savefig('output.jpg')
            # plt.close()

            # # Display the plot image using OpenCV
            # img = cv2.imread('output.jpg')
            # img = cv2.resize(img, (1080, 720))
            # cv2.imshow('Car Path Visualization', img)

            # # Wait for 1 millisecond to update the OpenCV window and check for key press
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

            ##----------Control---------## 
            # Uncomment to send the steering angle to STM32
            # stm32(angle=steering_angle, speed=7, brake_state=0)
        else:
            print("No intersection points found.")

def main1():
    steering_angles = []  # List to store steering angles
    times = []  # List to store time values for plotting
    start_time = time.time()  # Start time for plotting x-axis

    # Initial car position (simulated)
    car_lat, car_lon = 10.8521467017, 106.772795585 # Initial car position
    #car_lat, car_lon,_,a = GPS_module.get_gps_data(port = "COM17", baudrate  = 115200)
    car_x, car_y = lat_lon_to_xy(car_lat, car_lon)

    # Convert waypoints to XY (Cartesian coordinates)
    path_points = waypoints_to_xy(Waypoints)

    # Filter path points based on distance from car position
    filtered_path_points = [point for point in path_points if Point(point).distance(Point(car_x, car_y)) <= 50]

    # Print filtered path points for debugging
    print(f"Filtered Path Points: {filtered_path_points}")

    # Initialize Matplotlib figure and axes
    fig, ax = plt.subplots()
    ax.set_xlim(-20, 20)  # Initial limits
    ax.set_ylim(-20, 20)
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title('Car Position and Nearby Waypoint Path')
    ax.grid()
    ax.axis('equal')

    # Plot filtered path points if any exist
    if filtered_path_points:
        for point in filtered_path_points:
            ax.plot(point[0], point[1], 'yo')  # Mark filtered path points
    else:
        print("No filtered path points to plot.")

    # Initialize plot elements
    #path_line, = ax.plot([], [], '--', label="Path Line", color="lightblue")
    car_point_plot, = ax.plot([], [], 'ro', label="Car Position")
    lookahead_point_plot, = ax.plot([], [], 'go', label="Lookahead Point")
    line_to_lookahead, = ax.plot([], [], 'b-', label="Line to Lookahead Point")

    # Update function for the animation
    def update(frame):
        # Simulate car's state (heading, GPS data)
        #car_heading = 185  # Simulated car heading
        ##-- Car State --##  
        # Replace with actual GPS reading
        car_heading = cf.bearing
        # print(f"Heading: {car_heading}")
        #lon, lat = 106.771413, 10.851485  # Example position; replace with actual GPS reading
        #lat, lon = 106.7715131967, 10.8532570333
        lat, lon,_,a = GPS_module.get_gps_data(port = "COM17", baudrate  = 115200)
        print(lat)
        # Convert lat, lon to x, y for the car's position
        car_x, car_y = lat_lon_to_xy(float(lat), float(lon))
        path_points = waypoints_to_xy(Waypoints)
        # print(f"dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd: {lat, lon}")
        # Find intersection points
        lookahead_distance = 8  # Adjust as needed
        intersection_points = find_intersection((car_x, car_y), lookahead_distance, path_points)
        # print(intersection_points)
        if intersection_points and car_heading is not None:
            
            if len(intersection_points) == 2:
                front_intersection = None
                distances = [point.distance(Point(car_x, car_y)) for point in intersection_points]
                
                for point in intersection_points:
                    # Compute the course angle to the intersection point (lat/lon)
                    point_lat, point_lon = xy_to_lat_lon(point.x, point.y)
                    course_angle = course_to_waypoint(lat, lon, point_lat, point_lon)
                    
                    # Compare course angle with the car's heading angle
                    angle_diff = abs(course_angle - car_heading)
                    
                    # Normalize the difference to the range 0-180 degrees
                    if angle_diff > 180:
                        angle_diff = 360 - angle_diff
                    # Select the point if it's within a 90-degree field of view in front of the car
                    if angle_diff < 70:
                        front_intersection = point
                        break

                if front_intersection is None:
                    # If no point is in front, choose the closest one
                    front_intersection = intersection_points[np.argmin(distances)]
            else:
                # If only one intersection point exists, use it
                front_intersection = intersection_points[0]
            
            
            lookahead_lat, lookahead_lon = xy_to_lat_lon(front_intersection.x, front_intersection.y)
            car_point = Point(car_x, car_y)
            distance_to_path = car_point.distance(LineString(path_points))

            # Calculate the course to the waypoint
            alpha = course_to_waypoint(lat, lon, lookahead_lat, lookahead_lon) - car_heading
            alpha = (alpha + 180) % 360 - 180  # Normalize to [-180, 180]
            alpha = math.radians(alpha)
            

            # Calculate steering angle
            steering_angle = (math.atan((2 * L * math.sin(alpha))) / lookahead_distance) * 100
            steering_angles.append(steering_angle)
            current_time = time.time() - start_time
            times.append(current_time)

            print(f"Dis2P: {distance_to_path:.2f}, Alpha: {alpha:.2f}, Steering Angle: {steering_angle:.2f}")
            lookahead_point = Point(front_intersection.x, front_intersection.y)
            line_between = LineString([(car_x, car_y), (lookahead_point.x, lookahead_point.y)])

            
            filtered_path_points = [point for point in path_points if Point(point).distance(car_point) <= 50]

            # Only update path line if there are filtered points
            # if filtered_path_points:
            #     path_line.set_data(*zip(*filtered_path_points))
            # else:
            #     path_line.set_data([], [])  # Clear path line if no points

            # Update car position
            car_point_plot.set_data(car_y, car_x)

            # Dynamically adjust plot limits based on car position
            ax.set_xlim(car_x - 20, car_x + 20)
            ax.set_ylim(car_y - 20, car_y + 20)

            return  car_point_plot, lookahead_point_plot, line_to_lookahead

    # Start the animation
    ani = animation.FuncAnimation(fig, update, interval=10, blit=True)
    plt.legend()
    plt.show()
if __name__ == "__main__":
    #thread1 = threading.Thread(target=IMU_F722.read_imu_and_gps)
    #thread2 = threading.Thread(target=main1)

    # Bắt đầu các luồng
    #thread1.start()
    #thread2.start()

    # # Đợi các luồng hoàn thành (thực tế, chúng sẽ chạy mãi mãi vì vòng lặp while True)
    #thread1.join()
    #thread2.join()
    
    main()
