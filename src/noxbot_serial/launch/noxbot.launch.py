#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Controller node - converts cmd_vel to PWM
        Node(
            package='noxbot_controller',
            executable='controller',
            name='controller_node',
            output='screen',
            parameters=[]
        ),
        
        # Serial communication node - sends PWM to ESP32
        Node(
            package='noxbot_serial',
            executable='ros_serial',
            name='ros_serial_node',
            output='screen',
            parameters=[]
        ),
    ])
