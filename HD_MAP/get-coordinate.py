import tkinter as tk
import GPS_module

class GPSApp:
    def __init__(self, master):
        self.master = master
        self.master.title("GPS Data Logger")
        
        # Create a label to display GPS data
        self.data_label = tk.Label(master, text="GPS Data: ", font=("Arial", 14))
        self.data_label.pack(pady=20)

        # Create a button to start logging GPS data
        self.start_button = tk.Button(master, text="Start Logging", command=self.start_logging)
        self.start_button.pack(pady=10)

        # Variable to control the data logging
        self.logging = False

        # Bind the key press event
        self.master.bind("<Key>", self.on_key_press)

    def start_logging(self):
        self.logging = True
        print("start_logging")
        self.update_gps_data()

    def update_gps_data(self):
        if self.logging:
            # Get GPS data
            print("update_gps_data")
            lat, lon, sat_count, alt = GPS_module.get_gps_data('COM17', baudrate=115200)
            gps_data = f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt}, Satellites: {sat_count}"
            self.data_label.config(text=gps_data)
            self.master.after(1000, self.update_gps_data)  # Update every second

    def on_key_press(self, event):
        if event.char == 's':
            print("save")
            self.save_gps_data()

    def save_gps_data(self):
        lat, lon, sat_count, alt = GPS_module.get_gps_data('/dev/ttyUSB0', baudrate=115200)
        with open("./HD_MAP1.txt", "a") as file:
            file.write(f"{lat}, {lon}\n")
        print(f"Saved: Latitude: {lat}, Longitude: {lon}")

if __name__ == "__main__":
    root = tk.Tk()
    app = GPSApp(root)
    root.mainloop()
