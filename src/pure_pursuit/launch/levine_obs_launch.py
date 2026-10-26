"""Levine cu obstacole (levine_obs), pe traseul generat care le ocolește.

    ros2 launch pure_pursuit levine_obs_launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node

EXECUTABLE = 'pure_pursuit_node'

# valorile pentru parametrii pe care îi declară nodul, pentru harta asta
PARAMETERS = {
    'csv_name': 'levine_obs_path.csv',
    'lookahead': 1.0,
    'speed': 2.0,
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
