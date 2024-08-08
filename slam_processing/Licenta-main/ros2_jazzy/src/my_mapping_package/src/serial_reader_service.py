#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialReaderService(Node):
    def __init__(self):
        super().__init__('serial_reader_service')
        self.lidar_publisher = self.create_publisher(String, 'lidar_serial_data', 10)
        self.imu_publisher = self.create_publisher(String, 'imu_serial_data', 10)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.timer = None
        self.get_logger().info('Serial Reader Service Node has started')
        self.start_serial_reading()

    def start_serial_reading(self):
        self.get_logger().info('Starting serial reading...')
        self.timer = self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        try:
            data = self.ser.readline().decode('utf-8').strip()
            if data:
                self.get_logger().info(f"Raw serial data: {data}")
            if data.startswith('Lidar:'):
                self.lidar_publisher.publish(String(data=data))
                self.get_logger().info(f"Published Lidar data: {data}")
            elif data.startswith('IMU:'):
                self.imu_publisher.publish(String(data=data))
                self.get_logger().info(f"Published IMU data: {data}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
