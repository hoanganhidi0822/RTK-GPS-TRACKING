import GPS_module
import time
import config as cf
import threading
import IMU_F722
cf.bearing = None
cf.latitude = None
cf.longitude = None


# Initialize GPS
gps = GPS_module

# Open a file to store GPS data

def main():
    with open('D:/Documents/Researches/2024_Project/RTK_GPS/HD_MAP/UTE_MAP/MAP_NGHICH/HD_MAP2.txt', 'w') as file:
        while True:
            try:
                lat, lon,_,a= GPS_module.get_gps_data(port = "COM17", baudrate  = 115200)
                # Write data to file in CSV format
                file.write(f"{lat},{lon}\n")
                file.flush()  # Ensure data is written to the file
                print(".")
                time.sleep(1)  # Collect data every 1 second
            except Exception as e:
                print(f"Error: {e}")

if __name__ == '__main__':

    main()