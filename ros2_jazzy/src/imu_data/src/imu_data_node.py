#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import threading

class ImuDataProcessor(Node):
    def __init__(self):
        super().__init__('imu_data_processor')
        self.get_logger().info('IMU Data Processor Node has started')
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.publisher_ = self.create_publisher(Imu, 'imu_data', 10)
        self.thread = threading.Thread(target=self.read_and_process_data)
        self.thread.start()

    def read_and_process_data(self):
        while rclpy.ok():
            try:
                data = self.ser.readline().strip()
                if data.startswith(b'IMU: '):
                    data = data[5:]  # Strip "IMU: " prefix
                    self.get_logger().info(f"Received IMU data: {data}")
                    self.process_data(data)
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

    def process_data(self, data):
        try:
            values = list(map(float, data.decode().split()))
            if len(values) == 6:
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = values

                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'imu_frame'

                # Assign accelerometer values
                imu_msg.linear_acceleration.x = accel_x
                imu_msg.linear_acceleration.y = accel_y
                imu_msg.linear_acceleration.z = accel_z

                # Assign gyroscope values
                imu_msg.angular_velocity.x = gyro_x
                imu_msg.angular_velocity.y = gyro_y
                imu_msg.angular_velocity.z = gyro_z

                self.publisher_.publish(imu_msg)
                self.get_logger().info(f"Published IMU data: {imu_msg}")
        except ValueError as e:
            self.get_logger().error(f"Error processing IMU data: {e}")

def main(args=None):
    rclpy.init(args=args)
    processor = ImuDataProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
