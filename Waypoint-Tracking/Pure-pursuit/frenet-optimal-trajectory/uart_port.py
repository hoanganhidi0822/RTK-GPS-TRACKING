import serial.tools.list_ports

def list_uart_ports():
    ports = serial.tools.list_ports.comports()
    available_ports = []
    
    for port in ports:
        available_ports.append(port.device)
        print(f"Port: {port.device}, Description: {port.description}, HWID: {port.hwid}")
    
    return available_ports

if __name__ == "__main__":
    print("Available UART ports:")
    list_uart_ports()
