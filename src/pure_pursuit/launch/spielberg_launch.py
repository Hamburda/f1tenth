"""Spielberg: un tur, în sensul liniei de centru.

    ros2 launch pure_pursuit spielberg_launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node

EXECUTABLE = 'pure_pursuit_node'

# valorile pentru parametrii pe care îi declară nodul, pentru harta asta
PARAMETERS = {
    'csv_name': 'Spielberg_centerline.csv',
    'lookahead': 2.0,
    'speed': 4.0,
}


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable=EXECUTABLE,
            output='screen',
            parameters=[PARAMETERS],
        ),
    ])
