import serial
import datetime
import time

def dec2deg(value):
    if not value:
        return None
    # Convert NMEA format to decimal degrees
    dec = value / 100.00
    deg = int(dec)
    minutes = (dec - deg) * 100 / 60
    position = deg + minutes
    # Format to 10 decimal places for higher precision
    position = "{:.10f}".format(position)
    return position

def wait_for_com_port(port):
    while True:
        try:
            ser = serial.Serial(port, baudrate=115200, timeout=0.1)
            ser.close()  # Close the port if successfully opened
            #print(f"Port {port} is available.")
            return
        except serial.SerialException:
            print(f"Port {port} not available yet, waiting...")
            time.sleep(1)

ser = serial.Serial('/dev/ttyUSB0', baudrate  = 115200, timeout=0.1)


def get_gps_data(port, baudrate  = 115200):
    # wait_for_com_port(port)
    # ser = serial.Serial(port, baudrate, timeout=0.1)
    while True:
        gps_data = ser.readline().decode("utf8")
        #print(gps_data)
        if gps_data:
            try:
                gps_data = gps_data.split(',')
                if "$GPRMC" in gps_data[0]:
                    hrs, mins, sec = gps_data[1][0:2], gps_data[1][2:4], gps_data[1][4:6]
                    day, month, year = gps_data[9][0:2], gps_data[9][2:4], gps_data[9][4:6]
                    datetime_utc = "{}:{}:{} {}/{}/{}".format(hrs, mins, sec, day, month, year)
                    datetime_utc = datetime.datetime.strptime(datetime_utc, '%H:%M:%S %d/%m/%y')
                    speed = round(float(gps_data[7]) * 1.852, 2)
                    #message = "Speed={} kmph".format(datetime_utc, speed)
                    #print(message)
                    
                # Heading  $GPHDT 
                if "$GPGGA" in gps_data[0]:
                    if len(gps_data) > 9:
                        lat = dec2deg(float(gps_data[2])) if gps_data[2] else None
                        lon = dec2deg(float(gps_data[4])) if gps_data[4] else None
                        alt = gps_data[9] if gps_data[9] else None
                        sat_count = gps_data[7] if gps_data[7] else None
                        ser.close()
                        print(lat)
                        return lat, lon, sat_count, alt
            except Exception as e:
                print("Error processing GPS data:", e)

