#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
import math
import struct

class LidarDataProcessor(Node):
    def __init__(self):
        super().__init__('lidar_data_processor')
        self.get_logger().info('Lidar Data Processor Node has started')
        self.publisher_ = self.create_publisher(PointCloud2, 'lidar_points', 10)
        self.subscription = self.create_subscription(String, 'lidar_serial_data', self.lidar_callback, 10)
        self.subscription  # prevent unused variable warning
        self.points = []

    def lidar_callback(self, msg):
        data = msg.data
        self.get_logger().info(f"Received Lidar data: {data}")
        try:
            cleaned_data = data.replace("Lidar: ", "").replace(" ", "")
            self.get_logger().info(f"Cleaned Lidar data: {cleaned_data}")
            
            if len(cleaned_data) < 10:
                cleaned_data = cleaned_data.zfill(10)  # Pad with leading zeros to ensure length is 10
            
            if len(cleaned_data) == 10:
                data_bytes = bytes.fromhex(cleaned_data)
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

                self.get_logger().info(f"Processed Lidar data: quality={quality}, angle={angle}, distance={distance}, x={x}, y={y}")

                if len(self.points) >= 10:  # Reduced threshold for debugging
                    self.get_logger().info("Threshold reached, calling publish_point_cloud()")
                    self.publish_point_cloud()
                    self.points = []
            else:
                self.get_logger().error(f"Error processing Lidar data: cleaned data length is not 10: {cleaned_data}")
        except ValueError as e:
            self.get_logger().error(f"Error processing Lidar data: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def publish_point_cloud(self):
        try:
            self.get_logger().info(f"Preparing to publish point cloud with {len(self.points)} points")
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

            self.get_logger().info(f"Publishing point cloud with {len(self.points)} points")
            self.publisher_.publish(point_cloud)
            self.get_logger().info(f"Published point cloud with {len(self.points)} points")
        except Exception as e:
            self.get_logger().error(f"Error while publishing point cloud: {e}")

def main(args=None):
    rclpy.init(args=args)
    processor = LidarDataProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
