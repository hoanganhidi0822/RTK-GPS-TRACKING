from GPS_module import get_gps_data


while 1:
    lat, lon, _, _ = get_gps_data('/dev/ttyUSB0')
    print(lat, lon)