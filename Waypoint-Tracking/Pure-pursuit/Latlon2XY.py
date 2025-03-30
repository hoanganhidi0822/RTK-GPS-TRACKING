import math

def lat_lon_to_xy(lat, lon, lat0, lon0):
    # Earth's radius in meters
    R = 6371000  
    
    # Convert degrees to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)

    # Calculate x and y
    x = R * (lon_rad - lon0_rad) * math.cos((lat0_rad + lat_rad) / 2)
    y = R * (lat_rad - lat0_rad)

    return x, y

# Example usage:
lat = 37.7749  # Latitude of the point
lon = -122.4194  # Longitude of the point
lat0 = 37.7749  # Latitude of the origin
lon0 = -122.4194  # Longitude of the origin

x, y = lat_lon_to_xy(lat, lon, lat0, lon0)
print(f"x: {x}, y: {y}")
