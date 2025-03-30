import matplotlib.pyplot as plt
import matplotlib.animation as animation
import GPS_module  # Replace with your GPS module import
import config as cf
# Initialize the GPS module
gps = GPS_module# Adjust according to your GPS module

# cf.bearing = None
# cf.latitude = None
# cf.longitude = None

# Function to read historical GPS data from file
def read_gps_data(filename):
    latitudes = []
    longitudes = []
    try:
        with open(filename, 'r') as file:
            for line in file:
                try:
                    lat, lon = map(float, line.strip().split(','))
                    latitudes.append(lat)
                    longitudes.append(lon)
                except ValueError:
                    print(f"Skipping invalid line: {line.strip()}")
    except FileNotFoundError:
        print("File not found. Make sure to run the data collection script first.")
    return latitudes, longitudes

# Initialize the plot
fig, ax = plt.subplots()
latitudes, longitudes = read_gps_data('MAP5.txt')

# Plot historical data if available
if latitudes and longitudes:
    ax.plot(longitudes, latitudes, color='black', label='Historical Data')

# Initialize the marker for the real-time position
current_marker, = ax.plot([], [], 'ro', label='Current Position')

# Set plot labels and legend
ax.set_xlabel('Longitude')
ax.set_ylabel('Latitude')
ax.set_title('GPS Data Map')
ax.legend()

# Set initial plot limits
if latitudes and longitudes:
    ax.set_xlim(min(longitudes) - 0.01, max(longitudes) + 0.01)
    ax.set_ylim(min(latitudes) - 0.01, max(latitudes) + 0.01)
else:
    ax.set_xlim(-180, 180)
    ax.set_ylim(-90, 90)

def update_plot(frame):
    try:
        # Get the current GPS data
        lat, lon,_,a= GPS_module.get_gps_data(port = "COM17", baudrate  = 115200)

        # Ensure the data is numeric
        lat = float(lat)
        lon = float(lon)

        # Update the current position marker
        current_marker.set_data(lon, lat)

        # Update plot limits to keep the current position within view
        if latitudes and longitudes:
            ax.set_xlim(min(longitudes) - 0.0001, max(longitudes) + 0.0001)
            ax.set_ylim(min(latitudes) - 0.0001, max(latitudes) + 0.0001)
        else:
            ax.set_xlim(lon - 0.0001, lon + 0.0001)
            ax.set_ylim(lat - 0.0001, lat + 0.0001)
    
    except Exception as e:
        print(f"Error: {e}")
        current_marker.set_data([], [])  # Clear marker if there's an error

    return current_marker,

# Set up animation
ani = animation.FuncAnimation(fig, update_plot, interval=0.1, blit=True, repeat=True)

plt.show()
