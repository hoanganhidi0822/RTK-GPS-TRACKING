import cv2
import copy
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import threading
import config as cf

from visualize import *
from RTK_GPS.GPS_module import *
from HD_MAP.HDMAP import *
from utils.communication import STM32
from OPTIMAL_TRAJECTORY.frenet_optimal_trajectory import *
from OBSTACLES.obstacle import process_depth
# from Assistance_Astar import main_assistance

cf.latitude = None
cf.longitude = None
cf.heading = None

cf.image = np.zeros((480, 1280, 3))
cf.obstacles = []

cf.tx                 = []
cf.ty                 = []
cf.tyaw               = []
cf.ob                 = []
cf.paths              = []
cf.optimal_path       = []
cf.x                  = 0
cf.y                  = 0
cf.yaw                = 0 
cf.steering_angle     = 0 
cf.lookahead_distance = (0,0) 
cf.lookahead_point    = (0,0) 
cf.projected_point    = (0,0) 
cf.car_speed          = 0 
cf.car_steer          = 0 
cf.area               = 20
cf.MAX_ROAD_WIDTH     = 6

def update_state(ser):
    # lat, lon, car_heading, sat_count = get_gps_data(port = "COM17", baudrate  = 115200)
    
    lat, lon, car_heading, sat_count = get_gps_data(ser)
    
    # print(f"lat {lat}, lon {lon}, heading {car_heading}")
    # lat, lon, car_heading = 10.8532568817,106.7715131950, 185
    # lat, lon, car_heading = 10.8526012233,106.7714274617, 186
    x, y= lat_lon_to_xy(float(lat), float(lon))
    yaw_c = np.deg2rad(convert_yaw(float(car_heading), yaw_offset=90))
    
    return lat, lon, x, y, yaw_c 


# def gps_thread(ser):
   
#     lat, lon, car_heading, sat_count = get_gps_data(ser)
#     # Update global values or process GPS data as needed here
#     cf.latitude = lat
#     cf.longitude = lon
#     cf.heading = car_heading
#     # time.sleep(0.01)  # Delay to avoid overloading the CPU


# def gps_thread(ser):
#     """ Luồng đọc dữ liệu GPS liên tục """
#     # lat, lon, car_heading, _ = get_gps_data(ser)
#     get_gps_data(ser)
    
        
        # time.sleep(0.01)  # Giảm tải CPU

# def update_state():
#     """ Hàm cập nhật trạng thái từ biến toàn cục """
#     lat, lon, car_heading = cf.latitude, cf.longitude, cf.heading
#     if lat is None or lon is None or car_heading is None:
#         return None, None, None, None, None  # Trả về None nếu chưa có dữ liệu
    
#     x, y = lat_lon_to_xy(float(lat), float(lon))
#     yaw_c = np.deg2rad(convert_yaw(float(car_heading), yaw_offset=90))
#     return lat, lon, x, y, yaw_c



def depth_thread():
    process_depth()
    # time.sleep(0.01)

def plot_obstacle(x, y, yaw, width=1.0, length=2.0, color='blue'):
    """
    Vẽ obstacle hình chữ nhật với vị trí (x, y), góc yaw (radian),
    kích thước width x length.
    """
    
    # Tọa độ góc của hình chữ nhật trước khi xoay (tọa độ tương đối)
    corners = np.array([
        [-length / 2, -width / 2],
        [ length / 2, -width / 2],
        [ length / 2,  width / 2],
        [-length / 2,  width / 2]
    ])

    # Ma trận xoay
    rotation_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw), np.cos(yaw)]
    ])

    # Xoay và tịnh tiến hình chữ nhật
    rotated_corners = np.dot(corners, rotation_matrix.T) + np.array([x, y])

    # Vẽ hình chữ nhật sử dụng Polygon
    rect = patches.Polygon(rotated_corners, closed=True, color=color, alpha=0.5, edgecolor='black')
    plt.gca().add_patch(rect)

def slow_down_speed(distance, max_speed):
    return int(max_speed * (1 / (1 + math.exp(5 - 1 * distance))))  # 5 và 1 là các hệ số điều chỉnh
   
def main():
    print(__file__ + " start!!")
    
    fps = 0
    time_start = 0
    
    stm32 = STM32(port="COM5", baudrate=115200)
    gps_ser = connect_to_serial("COM17", 115200)
    
    
    # Initial GPS state
    lat, lon, car_heading = 10.8532568817,106.7715131950, 185
    x, y = 16.993362287481073, 152.51032455731195
     
    while True:
        
        lat, lon, x, y, yaw = update_state(gps_ser)
        
        try:
            lat = float(lat)         # Chuyển đổi lat sang số thực
            lon = float(lon)
            
            if not math.isnan(lat):  # Nếu lat không phải NaN thì thoát vòng lặp
                break
        except ValueError:
            pass  # Nếu không thể chuyển đổi, tiếp tục lấy dữ liệu GPS.

    
    # # Tạo và chạy Thread 1
    # t1 = threading.Thread(target=main_assistance.task1)
    # t1.start()

    # # Tạo Thread 2 và 3 nhưng chúng sẽ đợi event từ Task 1
    # t2 = threading.Thread(target=main_assistance.task2)
    # t2.start()

    # # Đợi tất cả các thread kết thúc
    # t1.join()
    # t2.join()
    
    # Waypoints
    wx, wy = XY_WAYPOINTS_MAP(input_file)

    # Obstacles
    obstacles = []
    new_obstacles = []
    ob = np.array([[0, 0]])
    obs = [[999, 999]]

    # Initial state
    c_speed = 0                  # current speed [m/s]
    c_d     = 0.0                # current lateral position [m]
    c_d_d   = 0.0                # current lateral speed [m/s]
    c_d_dd  = 0.0                # current lateral acceleration [m/s]
    s0      = 0.0                # current course position
    
    yaw = np.deg2rad(85)         # Convert yaw to radians
    car_speed = 2.8              # Car speed [m/s]
    car_steer = 0                # Car steering angle [degree]
    
    # Generate target course (assuming `generate_target_course` is defined elsewhere)
    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)

    # Tracking Algorithm Parameters
    lookahead_distance = 5       # Lookahead distance [m]
    L = 1.8                      # Wheelbase of the vehicle [m]
    count = 0
    
    # Start GPS thread
    # gps_threading = threading.Thread(target=gps_thread, args=(gps_ser,))
    # gps_threading.daemon = True
    # gps_threading.start()
    
    # Start thread for depth processing
    depth_thread_instance = threading.Thread(target=depth_thread)
    depth_thread_instance.daemon = True
    depth_thread_instance.start()

    # --- Animation --- #
    area = 20.0                   # Animation area length [m]
    show_animation = 0         # Enable animation
    history_x = []
    history_y = []
    while True:
        
        # Update vehicle state using RTK GPS
        lat, lon, x, y, yaw = update_state(gps_ser)
        if x == None:
            continue

        image = cf.image
        new_obstacles = cf.obstacles
        
        # Chỉ cập nhật nếu có dữ liệu mới hợp lệ
        if len(new_obstacles):
            obstacles = []
            obstacles = new_obstacles
            new_obstacles = []
            
        # Check if image data is valid
        if image is None or len(image) == 0:
            continue

        for obstacle in obstacles:
            obs.append(transform_obstacle_to_global(x, y, yaw, obstacle[1], obstacle[0]))
            
        obstacles = []
        ob = np.array(obs)
        
        # Project the current position onto the target course
        # projected_point = project_onto_path(x, y, tx, ty)
        # s0 , c_d, c_d_d, c_d_dd = cartesian_to_frenet(projected_point[0], projected_point[1], yaw, csp)
        
        optimal_path, paths = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)

        while optimal_path is None:
            
            print("optimal_path is None !!!")
            lat, lon, x, y, yaw = update_state(gps_ser)
        
            # Project the current position onto the target course
            projected_point = project_onto_path(x, y, tx, ty)
            s0, c_d, c_d_d, c_d_dd = cartesian_to_frenet(projected_point[0], projected_point[1], yaw, csp)
            # c_d_d = optimal_path.d_d[1]
            # c_d_dd = optimal_path.d_dd[1]
            # c_speed = car_speed    
            
            optimal_path, paths = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)
            # continue

        if x is not None:
            # Pure Pursuit control: use the optimal path from Frenet
            car_steer,  lookahead_point = pure_pursuit_control_frenet(float(lat), float(lon),
                optimal_path, x, y,yaw, lookahead_distance, L)

            # Project the current position onto the target course
            # projected_point = project_onto_path(x, y, tx, ty)
            # s0 , _, __, ___ = cartesian_to_frenet(projected_point[0], projected_point[1], yaw, csp)
            s0 , c_d, c_d_d, c_d_dd = cartesian_to_frenet(x, y, yaw, csp)
            
            # c_d = optimal_path.d[1]
            c_d_d = optimal_path.d_d[1]
            c_d_dd = optimal_path.d_dd[1]
            c_speed = car_speed    
            
            distance_to_goal = np.hypot(tx[-1] - x, ty[-1] - y)
            if distance_to_goal < 10:
                speed = slow_down_speed(distance_to_goal, 10)
            else:
                speed = 10
            # ----------- Control Block ------------- #
            steering_angle = car_steer
            
            count += 1
            if count == 1:
                # print(steering_angle)
                stm32(angle= int(steering_angle), speed=0, brake_state=0)
                count = 0    
            
            
            history_x.append(x)
            history_y.append(y)    
            # Check if the goal is reached
            if np.isclose(x, tx[-1], atol=1.0) and np.isclose(y, ty[-1], atol=1.0):
                print("Goal reached!")
                break

        # # Visualization
        alpha = 0.1
        delta_t = time.time() - time_start
        if delta_t > 0:
            fps = (1 - alpha) * fps + alpha * (1 / delta_t)
        time_start = time.time()
        # print(f"\r Speed: {speed}, Steering angle: {steering_angle:.2f}, Brake: {0} FPS: {round(fps, 2)}", end="")
    
        if show_animation:
            plt.cla()
            plt.plot(tx, ty, "--y", label="Target Course")

            tyaw = np.array(tyaw)
            
            # Tính toán đường song song
            left_x  = tx + (MAX_ROAD_WIDTH * 0.75 + 0.5) * np.cos(tyaw + np.pi / 2)
            left_y  = ty + (MAX_ROAD_WIDTH * 0.75 + 0.5) * np.sin(tyaw + np.pi / 2)
            right_x = tx + (MAX_ROAD_WIDTH / 4 + 0.5)    * np.cos(tyaw - np.pi / 2)
            right_y = ty + (MAX_ROAD_WIDTH / 4 + 0.5)    * np.sin(tyaw - np.pi / 2)
            
            plt.plot(left_x , left_y,  "black", label="Left Boundary")
            plt.plot(right_x, right_y, "black", label="Right Boundary")

            # Vẽ obstacles dạng hình chữ nhật
            for ob in obs:
                plot_obstacle(ob[0], ob[1], yaw=(yaw))
                
            # # Các phần vẽ khác
            for traj in paths:
                plt.plot(traj.x, traj.y, "black", alpha=0.05)
            # x_data = np.array([traj.x for traj in paths])
            # y_data = np.array([traj.y for traj in paths])
            # plt.plot(x_data.T, y_data.T, color="black", alpha=0.05)


            plt.plot(optimal_path.x[1:], optimal_path.y[1:], "-g", alpha=0.95, label="Optimal Trajectory")

            circle = plt.Circle((x, y), lookahead_distance, color='b', fill=False, linestyle='--')
            plt.gca().add_artist(circle)
            plt.scatter(lookahead_point[0], lookahead_point[1], color='red', s=20, label="Lookahead Point")

            plot_car(x, y, yaw, np.deg2rad(steering_angle), cabcolor="-r")
            plt.plot(history_x, history_y, 'r-', label="Path traveled")  # Vẽ đường xe đã chạy
            
            plt.scatter(x, y, color='blue', label="Current Position")

            # if projected_point is not None:
            #     plt.scatter(projected_point[0], projected_point[1], color='green', s=20, label="Projected Point")

            plt.xlim(x - area, x + area)
            plt.ylim(y - area, y + area)
            plt.title(f"Speed: {car_speed * 3.6:.2f} km/h | Steering Angle: {(car_steer):.2f}°")
            plt.grid(True)
            plt.pause(1 / 60)
            obs = []
        else:
            obs = []

    if show_animation:
        plt.show()
        
if __name__ == '__main__':
    main()
    

