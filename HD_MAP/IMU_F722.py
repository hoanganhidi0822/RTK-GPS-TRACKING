import time
from yamspy import MSPy
import math
import config as cf
# Define the serial port and baud rate
SERIAL_PORT = "COM4"
BAUD_RATE = 115200

cf.latitude = None
cf.longitude = None
cf.bearing = None

def calculate_bearing(magnetometer_data):
    mag_x = magnetometer_data[0]
    mag_y = magnetometer_data[1]
    
    # Calculate the bearing in degrees
    bearing = math.atan2(mag_y, mag_x) * (180 / math.pi)
    
    # Normalize the bearing to 0 - 360 degrees
    if bearing < 0:
        bearing += 360

    return bearing

def read_imu_and_gps():
    try:
        print("ssssssssssssssss")
        with MSPy(device=SERIAL_PORT, loglevel='ERROR', baudrate=BAUD_RATE) as board:
            if board == 1:  # an error occurred...
                print("Error: Unable to connect to the flight controller.")
                return None
            
            while True:
                # Reading IMU data
                board.fast_read_imu()
                magnetometer_data = board.SENSOR_DATA['magnetometer']
                bearing = calculate_bearing(magnetometer_data)
                
                # Reading attitude data
                board.fast_read_attitude()
                bearing_fus = board.SENSOR_DATA['kinematics'][2]
                #print(f"Bearing: {bearing}")
                # Reading GPS data
                board.send_RAW_msg(board.MSPCodes['MSP_RAW_GPS'], data=[])
                dataHandler = board.receive_msg()
                board.process_recv_data(dataHandler)
                
                latitude = board.GPS_DATA['lat']/10000000.0  # Convert to decimal degrees
                longitude = board.GPS_DATA['lon']/10000000.0   # Convert to decimal degrees
                satCount = board.GPS_DATA['numSat']

                # Print out the readings
                print(f"Latitude: {latitude}, Longitude: {longitude}, Satellites: {satCount}, Bearing: {bearing}, Fused Bearing: {bearing_fus}")
                
                cf.latitude = latitude
                cf.longitude = longitude
                cf.bearing = bearing
                
                # Return the data as a tuple (you can use it if needed)
                #return latitude, longitude, bearing_fus

    except Exception as e:
        print(f"An error occurred: {e}")
        return None

if __name__ == "__main__":
    # Calling the function to read IMU and GPS data
    read_imu_and_gps()