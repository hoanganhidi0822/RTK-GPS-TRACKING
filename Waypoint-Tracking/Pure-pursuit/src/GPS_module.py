import serial
import time

def dec2deg(value):
    if not value:
        return None
    dec = value / 100.00
    deg = int(dec)
    minutes = (dec - deg) * 100 / 60
    position = deg + minutes
    return "{:.10f}".format(position)

def wait_for_com_port(port):
    while True:
        try:
            ser = serial.Serial(port, baudrate=115200, timeout=0.1)
            ser.close()
            #print(f"Port {port} is available.")
            return
        except serial.SerialException:
            print(f"Port {port} not available yet, waiting...")
            time.sleep(1)

def get_gps_data(port, baudrate=115200):
    wait_for_com_port(port)
    ser = serial.Serial(port, baudrate, timeout=0.1)
    
    lat = lon = sat_count = heading = None
    
    while True:
        gps_data = ser.readline().decode("utf8").strip()
        if gps_data:
            try:
                gps_data_split = gps_data.split(',')
                if gps_data.startswith("$GPHDT"):
                    heading = float(gps_data_split[1])
                elif gps_data.startswith("$GPGGA"):
                    if len(gps_data_split) > 9:
                        lat = dec2deg(float(gps_data_split[2])) if gps_data_split[2] else None
                        lon = dec2deg(float(gps_data_split[4])) if gps_data_split[4] else None
                        sat_count = int(gps_data_split[7]) if gps_data_split[7] else None
                if lat and lon and heading and sat_count:
                    ser.close()
                    return lat, lon, heading, sat_count
            except Exception as e:
                print("Error processing GPS data:", e)
