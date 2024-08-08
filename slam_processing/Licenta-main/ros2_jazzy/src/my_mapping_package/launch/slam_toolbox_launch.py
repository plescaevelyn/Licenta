import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_mapping_package',
            executable='serial_reader_service.py',
            name='serial_reader_service',
            output='screen'
        ),
        Node(
            package='my_mapping_package',
            executable='imu_data_node.py',
            name='imu_data_processor',
            output='screen'
        ),
        Node(
            package='my_mapping_package',
            executable='lidar_data_node.py',
            name='lidar_data_processor',
            output='screen'
        ),
        Node(
            package='my_mapping_package',
            executable='wheel_encoder_node.py',
            name='wheel_encoder_node',
            output='screen'
        ),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'slam_toolbox.mapping': True,
            }],
            remappings=[
                ('/scan', '/lidar_points'),
                ('/odom', '/odom')
            ]
        ),
        # Static transform from base_link to lidar_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_frame']
        )
    ])
