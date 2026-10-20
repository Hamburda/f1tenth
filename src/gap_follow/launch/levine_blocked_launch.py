from launch import LaunchDescription
from launch_ros.actions import Node

EXECUTABLE = 'reactive_node'

PARAMETERS = {
    'max_speed': 6.0,
    'min_speed': 2.0,
    'max_range': 10.0,
}


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gap_follow',
            executable=EXECUTABLE,
            output='screen',
            parameters=[PARAMETERS],
        ),
    ])
