"""Levine cu obstacole (levine_obs), pe traseul generat care le ocolește.

    ros2 launch pure_pursuit levine_obs_launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node

EXECUTABLE = 'pure_pursuit_node'

# valorile pentru parametrii pe care îi declară nodul, pentru harta asta
# viteză constantă 2 m/s (vmin = vmax) și lookahead fix 1.0 m (k = 0, L_min = L_max)
PARAMETERS = {
    'csv_name': 'levine_obs_path.csv',
    'vmin': 2.0,
    'vmax': 2.0,
    'k': 0.0,
    'L0': 1.0,
    'L_min': 1.0,
    'L_max': 1.0,
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
