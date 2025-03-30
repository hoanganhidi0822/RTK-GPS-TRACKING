import serial

class STM32:

    def __init__(self, port='', baudrate=115200):
        self.stm32 = serial.Serial(port, baudrate, timeout=0.1)

    def __call__(self, angle=0, speed=0, brake_state=False):
        angle = self.parse_angle(angle)
        speed = self.parse_speed(speed)
        brake = self.parse_brake(brake_state)
        
        data_to_send = self.preprocess(angle, speed, brake)
        print(data_to_send)
        self.stm32.write(data_to_send.encode())

    @staticmethod
    def parse_angle(angle):
        if -30 <= angle <= 30:
            return 100 + abs(angle) if angle < 0 else angle
        else:
            raise ValueError("Angle must be between -30 and 30.")

    @staticmethod
    def parse_speed(speed):
        if 0 <= speed <= 10:
            return speed
        else:
            raise ValueError("Speed must be between 0 and 10.")

    @staticmethod
    def parse_brake(brake_state):
        return 1 if brake_state else 0

    @staticmethod   
    def preprocess(angle, speed, brake):
        angle_str = f"{angle:03}"  # Format the angle to 3 digits
        speed_str = f"{speed:03}"  # Format the speed to 3 digits
        brake_str = f"{brake:01}"  # Format the brake state to 1 digit (0 or 1)
        return str(angle_str) + str(speed_str) + str(brake_str)
