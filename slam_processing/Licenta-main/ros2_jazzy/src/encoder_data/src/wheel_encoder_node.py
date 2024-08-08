#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from gpiozero import Button
import time
import math

class WheelEncoderNode(Node):

    def __init__(self):
        super().__init__('wheel_encoder_node')

        self.get_logger().info('Wheel Encoder Node has started')

        # Parameters
        self.wheel_diameter = 0.072  # in meters
        self.pulses_per_revolution = 20

        # Set up button (encoder channel A)
        self.pinA = 17
        self.encoder = Button(self.pinA)

        # Variables to keep track of pulses
        self.pulse_count = 0
        self.previous_time = time.time()
        self.position_x = 0.0

        # Callback function to increment pulse count
        self.encoder.when_pressed = self.pulse_callback

        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Timer to calculate speed and publish odometry
        self.timer = self.create_timer(0.1, self.publish_odometry)  # 10 Hz

    def pulse_callback(self):
        self.pulse_count += 1
        self.get_logger().info(f'Pulse count: {self.pulse_count}')

    def calculate_speed(self, elapsed_time):
        revolutions = self.pulse_count / self.pulses_per_revolution
        wheel_circumference = self.wheel_diameter * math.pi
        distance_traveled = revolutions * wheel_circumference  # in meters
        speed = distance_traveled / elapsed_time  # in m/s
        return speed, distance_traveled

    def publish_odometry(self):
        current_time = time.time()
        elapsed_time = current_time - self.previous_time
        speed, distance_traveled = self.calculate_speed(elapsed_time)

        # Update position
        self.position_x += distance_traveled

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set the position (1D motion for simplicity)
        odom.pose.pose.position.x = self.position_x
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0

        # Set the orientation (assuming no rotation for simplicity)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        # Set the velocity
        odom.twist.twist.linear.x = speed
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        # Log the speed and position
        self.get_logger().info(f'Publishing odometry: position_x={self.position_x}, speed={speed}')

        # Publish the odometry message
        self.odom_pub.publish(odom)

        # Reset for next interval
        self.pulse_count = 0
        self.previous_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = WheelEncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
