from launch import LaunchDescription
from launch_ros.actions import Node

EXECUTABLE = 'reactive_node'

PARAMETERS = {
    'max_speed': 3.0,
    'min_speed': 0.8,
    'max_range': 10.0,
    'car_half_width': 0.22,
    'bubble_radius': 0.3,
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