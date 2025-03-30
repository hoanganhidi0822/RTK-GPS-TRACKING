from GPS_module import get_gps_data  # Import the function from gps_reader.py

def main():
    port = "COM23"  # Change to your actual port, e.g., "/dev/ttyUSB0" for Linux
    while 1:
        lat, lon, heading, sat_count = get_gps_data(port)
        
        print(f"Latitude: {lat}")
        print(f"Longitude: {lon}")
        print(f"Heading: {heading}°")
        print(f"Satellite Count: {sat_count}")

if __name__ == "__main__":
    main()
