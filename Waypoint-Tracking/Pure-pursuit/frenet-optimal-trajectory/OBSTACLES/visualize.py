import matplotlib.pyplot as plt
import matplotlib.patches as patches
import threading

import numpy as np
import cv2
import math
import config as cf

cf.obstacles = None
cf.combined_frame = None

# Constants for vehicle dimensions
LENGTH = 2.4  # Vehicle length [m]
WIDTH = 1.0   # Vehicle width [m]
BACKTOWHEEL = 0.4  # Distance from back to wheel [m]
WHEEL_LEN = 0.25  # Wheel length [m]
WHEEL_WIDTH = 0.1  # Wheel width [m]
TREAD = 0.7  # Wheel tread [m]
WB = 1.5  # Wheelbase [m]

def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):
    """
    Plot a car representation on the radar grid.
    """
    # Define car outline
    outline = np.array([[-BACKTOWHEEL, LENGTH - BACKTOWHEEL, LENGTH - BACKTOWHEEL, -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, -WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    # Define wheel positions
    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rr_wheel = np.copy(fr_wheel)
    rl_wheel = np.copy(fl_wheel)

    # Apply rotation for yaw and steering
    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])
    fr_wheel = (fr_wheel.T @ Rot2).T
    fl_wheel = (fl_wheel.T @ Rot2).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB
    fr_wheel = (fr_wheel.T @ Rot1).T
    fl_wheel = (fl_wheel.T @ Rot1).T
    outline = (outline.T @ Rot1).T
    rr_wheel = (rr_wheel.T @ Rot1).T
    rl_wheel = (rl_wheel.T @ Rot1).T

    # Translate to position
    for shape in [outline, fr_wheel, fl_wheel, rr_wheel, rl_wheel]:
        shape[0, :] += x
        shape[1, :] += y

    # Plot the car
    plt.plot(outline[0, :], outline[1, :], truckcolor)
    plt.plot(fr_wheel[0, :], fr_wheel[1, :], truckcolor)
    plt.plot(fl_wheel[0, :], fl_wheel[1, :], truckcolor)
    plt.plot(rr_wheel[0, :], rr_wheel[1, :], truckcolor)
    plt.plot(rl_wheel[0, :], rl_wheel[1, :], truckcolor)
    plt.plot(x, y, "*")  # Car position marker

def plot_radar_grid(max_distance, resolution=1):
    """
    Plot radar-like grid with concentric circles.
    """
    for r in range(1, max_distance + 1, resolution):
        circle = plt.Circle((0, 0), r, color="gray", fill=False, linestyle="--", alpha=0.5)
        plt.gca().add_artist(circle)
    plt.axhline(0, color="gray", linestyle="--", alpha=0.5)
    plt.axvline(0, color="gray", linestyle="--", alpha=0.5)

def plot_obstacles(obstacles, width=1, height=2):
    """
    Plot obstacles as rectangles.
    """
    ax = plt.gca()
    for obs in obstacles:
        x, y = obs
        rect = patches.Rectangle((x - width / 2, y - height / 2), width, height,
                                  linewidth=1, edgecolor='r', facecolor='none')
        ax.add_patch(rect)

def visualize():
    # plt.figure()
    # plt.ion()  # Turn on interactive mode
    max_distance = 20  # Adjust as needed
    cf.obstacles = None
    cf.combined_frame = None
    while True:
        # Plot các vật thể
        
        print(cf.obstacles)
        # plt.ion()
        # plt.clf()
        # plot_radar_grid(max_distance=15)
        # plot_car(0, 0, np.deg2rad(90.0)  , truckcolor="-b")
        # # plot_obstacles(cf.obstacles)  # Vẽ tất cả vật thể trong danh sách

        # plt.xlim(-4, 4)
        # plt.ylim(0, 15)
        # plt.gca().set_aspect('equal', adjustable='box')
        # plt.title("Vehicle and Obstacles")
        # plt.xlabel("X [m]")
        # plt.ylabel("Y [m]")
        # plt.pause(0.01)
        # # Xóa danh sách sau khi đã vẽ
        
        
        # cv2.imshow('Depth - Distance Estimation', cf.combined_frame)

        # Exit on 'q' key
        
        
   
    
        
   
   
