import math

# Function to convert lat/lon to x/y using equirectangular projection
# 10.8532570333,106.7715131967
def lat_lon_to_xy(lat, lon, lat0=10.8532570333, lon0=106.7715131967):
    """
    Convert latitude and longitude to x, y using an equirectangular projection.

    lat0 and lon0 represent the center point (origin) for the projection.
    """
    R = 6371e3  # Radius of the Earth in meters
    x = R * math.radians(lon0 - lon) * math.cos(math.radians(lat0))
    y = R * math.radians(lat0 - lat)
    return x, y
# Read lat/lon from input file
input_file = 'MAP5.txt'  # Change this to your input file path
output_file = 'outputMAP5.txt'    # Change this to your desired output file path

with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
    for line in infile:
        # Strip whitespace and check if the line contains valid data
        line = line.strip()
        if not line:
            continue  # Skip empty lines
        
        try:
            # Split by comma and check if we have both lat and lon
            lat_str, lon_str = line.split(',')
            lat = float(lat_str)
            lon = float(lon_str)
            
            # Convert to x, y coordinates
            x, y = lat_lon_to_xy(lat, lon)
            
            # Write the result to output file
            outfile.write(f'{x},{y}\n')
        except ValueError:
            print(f"Skipping invalid line: {line}")
            continue  # Skip invalid lines

print(f"Conversion complete! Results saved in {output_file}")
