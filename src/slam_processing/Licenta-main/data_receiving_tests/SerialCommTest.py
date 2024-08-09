import serial
import threading
import time

class SerialCommTest:
    def __init__(self, port='/dev/ttyACM1', baud_rate=115200, file_path="test.txt"):
        self.ser = serial.Serial(port, baud_rate)
        self.file_path = file_path
        self.file = open(file_path, "w")

    def process_data(self, data):
        # Print values on screen
        
        # Write values to file
        self.file.write(f"Read from NXP: {data}\n")
        self.file.flush()  # Ensure data is written immediately

    def read_and_process_data(self):
        while True:
            try:
                data = self.ser.readline().decode().strip()  # Read until newline character
                print("Received data:", data)  # Add this line
                if data:  # Check if data is not empty
                    self.process_data(data)
            except Exception as e:
                print("Error:", e)


    def start(self):
        # Start a separate thread to read and process data
        self.thread = threading.Thread(target=self.read_and_process_data)
        self.thread.start()

    def stop(self):
        # Close serial port and file when done
        self.ser.close()
        self.file.close()

if __name__ == "__main__":
    processor = SerialCommTest()
    processor.start()

    # Main thread can continue with other tasks if needed

    # Main loop to keep the program running
    try:
        while True:
            time.sleep(1)  # Add a small delay to avoid consuming too much CPU
    except KeyboardInterrupt:
        print("Terminating program...")
        processor.stop()

    # Join the thread (optional)
    # processor.thread.join()

    # Stop the processor and cleanup
    # processor.stop()
