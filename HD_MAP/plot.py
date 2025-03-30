import matplotlib.pyplot as plt

# Step 1: Read the GPS data from the text file
file_path = 'gps_data_map.txt'  # Change this to your actual file path

latitudes = []
longitudes = []

with open(file_path, 'r') as file:
    for line in file:
        line = line.strip()
        if line:  # Only process non-empty lines
            try:
                lat, lon = map(float, line.split(','))
                latitudes.append(lat)
                longitudes.append(lon)
            except ValueError as e:
                print(f"Error processing line: '{line}'. Error: {e}")

# Step 2: Plot the GPS waypoints
plt.figure(figsize=(10, 6))
plt.scatter(longitudes, latitudes, color='blue', marker='o', label='Waypoints')
plt.title('GPS Waypoints Plot')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.grid(True)
plt.legend()
plt.axis('equal')  # Equal aspect ratio ensures that the scale is consistent

# Enable zooming and panning
plt.gca().set_aspect('equal', adjustable='box')
plt.tight_layout()  # Adjust layout to fit all elements

plt.show()
