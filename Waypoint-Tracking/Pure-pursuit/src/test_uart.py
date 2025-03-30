from utils.communication import STM32
#pythofrom GPS_module import get_gps_data
stm32 = STM32(port="COM7", baudrate=115200)

while 1:
    stm32(angle= int(steering_angle), speed=0, brake_state=0)