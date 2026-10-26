"""Spielberg: raceline cu marjă 0.6 m, profil de viteză și lookahead variabil.

Cel mai bun tur curat: 38.8 s (vezi docs/rezultate_pure_pursuit.md).
Necesită startul pe traseu în sim.yaml: sx 14.76, sy 3.35, stheta -2.851.

    ros2 launch pure_pursuit spielberg_launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node

EXECUTABLE = 'pure_pursuit_node'

# valorile pentru parametrii pe care îi declară nodul, pentru harta asta
PARAMETERS = {
    'csv_name': 'Spielberg_raceline_safe_0.6.csv',
    'vmax': 12.0,
    'vmin': 4.0,
    'a_lat': 7.5,
    'a_brake': 3.0,
    'a_acc': 4.0,
    'k': 0.3,
    'L0': -0.6,
    'L_min': 1.5,
    'L_max': 4.0,
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
