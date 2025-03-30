import serial
import time
import config as cf

cf.latitude = None
cf.longitude = None
cf.heading = None

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
            return
        except serial.SerialException:
            print(f"Port {port} not available yet, waiting...")
            time.sleep(1)

def connect_to_serial(port, baudrate=115200, timeout=0.1):
    """Handles the connection to the serial port."""
    wait_for_com_port(port)
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        return ser
    except serial.SerialException as e:
        print(f"Failed to connect to {port}: {e}")
        return None

def get_gps_data(ser):
    """Reads GPS data from an already connected serial port."""
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
                    # cf.latitude = lat
                    # cf.longitude = lon
                    # cf.heading = heading
                    return lat, lon, heading, sat_count
            except Exception as e:
                print("Error processing GPS data:", e)
