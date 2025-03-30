import matplotlib.pyplot as plt
import numpy as np

# Car position
car_x = 5
car_y = 5

# Create a figure and axis
fig, ax = plt.subplots()

# Draw the car as a rectangle
car_width = 1
car_length = 2
car = plt.Rectangle((car_x - car_width / 2, car_y - car_length / 2), 
                    car_width, car_length, color='blue', label="Car")

# Add the car to the plot
ax.add_patch(car)

# Add the car position point
plt.plot(car_x, car_y, 'ro', label="Car Position")

# Set limits and labels
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Car Position on Plot')
plt.legend()
plt.grid()

# Show the plot
plt.show()
