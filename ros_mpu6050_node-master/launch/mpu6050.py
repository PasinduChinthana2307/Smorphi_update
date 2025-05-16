# launch/mpu6050_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_mpu6050_node',
            executable='mpu6050_node',
            name='mpu6050_node',
            output='screen',
            parameters=[
                {'frequency': 10},
                {'frame_id': 'base_imu'},
                {'ax': 0},
                {'ay': 0},
                {'az': 0},
                {'gx': 0},
                {'gy': 0},
                {'gz': 0},
                {'ado': False},
                {'debug': False}
            ]
        )
    ])
