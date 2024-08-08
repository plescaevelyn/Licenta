import serial
import threading
import math 
import matplotlib.pyplot as plt
import numpy as np

class LidarDataProcessor:
    def __init__(self, port='/dev/ttyACM0', baud_rate=115200, file_path="point_cloud_data.pcd"):
        self.ser = serial.Serial(port, baud_rate)
        self.file_path = file_path

        # Open the file for writing
        self.file = open(file_path, "w")

        # Write header information to the file
        self.file.write("# .PCD v.7 - Point Cloud Data file format\n")
        self.file.write("VERSION .7\n")
        self.file.write("FIELDS x y z\n")
        self.file.write("SIZE 4 4 4\n")
        self.file.write("TYPE F F F\n")
        self.file.write("COUNT 1 1 1\n")
        self.file.write("WIDTH 640\n")
        self.file.write("HEIGHT 1\n")
        self.file.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        self.file.write("POINTS 640\n")
        self.file.write("DATA ascii\n")
        
        # Create the plot and axes
        self.fig, self.ax = plt.subplots()
        self.ax.set(xlim=(0, 8), xticks=np.arange(1, 8),
                    ylim=(0, 8), yticks=np.arange(1, 8))
        self.line, = self.ax.plot([], [], linewidth=2.0)
        self.points = []

    def process_data(self, data):
        # Extract quality, angle, and distance bytes
        quality = data[0]
        angle_bytes = data[1:3]
        distance_bytes = data[3:]
        
        hex_data = ' '.join(hex(byte) for byte in data[:5])
        print(hex_data)

        # Combine angle bytes and distance bytes
        angle = ((angle_bytes[1] << 8) | angle_bytes[0]) / 64.0
        distance = ((distance_bytes[1] << 8) | distance_bytes[0]) / 4.0 / 1000
        
        print(f"{quality} {angle} {distance}")
        
        angle_radians = math.radians(angle)
        x = distance * math.cos(angle_radians)
        y = distance * math.sin(angle_radians)

        # Append the point to the list of points
        self.points.append((x, y))
        
        # Write values to file
        # self.file.write(f"Quality: {quality}, Angle (angle): {angle}, Distance (mm): {distance}\n")
        self.file.write(f"{x} {y} 0\n")
        self.file.flush()  # Ensure data is written immediately

    def update_plot(self):
        try:
            while True:
                if self.points:
                    print("Updating plot with points:", self.points)  # Check the points being added to the plot
                    # Update the plot with the new points
                    x, y = zip(*self.points)
                    self.line.set_data(x, y)
                    self.ax.relim()
                    self.ax.autoscale_view(True, True, True)
                    self.fig.canvas.draw()
                    self.fig.canvas.flush_events()
                    # Clear the points list
                    self.points = []
                plt.pause(0.001)  # Pause for a short time to update the plot
        except Exception as e:
            print("Error in update_plot:", e)


    def read_and_process_data(self):
        while True:
            try:
                data = self.ser.read(5)  # Read 5 bytes
                if len(data) == 5:  # Check if data is not empty
                    self.process_data(data)
            except Exception as e:
                print("Error:", e)

    def start(self):
        # Start a separate thread to read and process data
        self.thread1 = threading.Thread(target=self.read_and_process_data)
        self.thread1.start()

        # Start a separate thread to update the plot
        self.thread2 = threading.Thread(target=self.update_plot)
        self.thread2.start()

    def stop(self):
        # Close serial port and file when done
        self.ser.close()
        self.file.close()

if __name__ == "__main__":
    processor = LidarDataProcessor()
    processor.start()
    plt.show()  # Ensure the plot window remains open
