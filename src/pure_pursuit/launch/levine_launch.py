"""Levine (levine_blocked): trei tururi în sens invers acelor de ceasornic.

    ros2 launch pure_pursuit levine_launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node

EXECUTABLE = 'pure_pursuit_node'

# valorile pentru parametrii pe care îi declară nodul, pentru harta asta
# viteză constantă 4 m/s (vmin = vmax) și lookahead fix 1.5 m (k = 0, L_min = L_max)
PARAMETERS = {
    'csv_name': 'levine_centerline.csv',
    'vmin': 4.0,
    'vmax': 4.0,
    'k': 0.0,
    'L0': 1.5,
    'L_min': 1.5,
    'L_max': 1.5,
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
