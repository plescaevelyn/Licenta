#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import serial
import threading
import math
import struct

class LidarDataProcessor(Node):
    def __init__(self):
        super().__init__('lidar_data_processor')
        self.get_logger().info('Lidar Data Processor Node has started')  # Add this line
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.publisher_ = self.create_publisher(PointCloud2, 'lidar_points', 10)
        self.thread = threading.Thread(target=self.read_and_process_data)
        self.thread.start()
        self.points = []

    def read_and_process_data(self):
        while rclpy.ok():
            try:
                data = self.ser.readline().strip()
                if data.startswith(b'Lidar: '):
                    data = data[7:]  # Strip "Lidar: " prefix
                    self.get_logger().info(f"Received data: {data}")
                    self.process_data(data)
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

    def process_data(self, data):
        data_bytes = bytes.fromhex(data.decode())
        if len(data_bytes) == 5:
            quality = data_bytes[0]
            angle_bytes = data_bytes[1:3]
            distance_bytes = data_bytes[3:]

            angle = ((angle_bytes[1] << 8) | angle_bytes[0]) / 64.0
            distance = ((distance_bytes[1] << 8) | distance_bytes[0]) / 4.0 / 1000

            angle_radians = math.radians(angle)
            x = distance * math.cos(angle_radians)
            y = distance * math.sin(angle_radians)
            z = 0.0  # Assuming 2D Lidar, z is always 0

            self.points.append((x, y, z))

            self.get_logger().info(f"Processed data: quality={quality}, angle={angle}, distance={distance}, x={x}, y={y}")

            if len(self.points) >= 640:  # Assuming a full scan
                self.publish_point_cloud()
                self.points = []

    def publish_point_cloud(self):
        header = self.get_clock().now().to_msg()
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_cloud = PointCloud2()
        point_cloud.header.stamp = header
        point_cloud.header.frame_id = 'lidar_frame'
        point_cloud.height = 1
        point_cloud.width = len(self.points)
        point_cloud.is_dense = False
        point_cloud.is_bigendian = False
        point_cloud.fields = fields
        point_cloud.point_step = 12  # 3 * 4 bytes for float32
        point_cloud.row_step = point_cloud.point_step * point_cloud.width
        point_cloud.data = b''.join([struct.pack('fff', *point) for point in self.points])

        self.publisher_.publish(point_cloud)
        self.get_logger().info(f"Published point cloud with {len(self.points)} points")
        self.get_logger().info(f"PointCloud2 data: {point_cloud}")

def main(args=None):
    rclpy.init(args=args)
    processor = LidarDataProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
