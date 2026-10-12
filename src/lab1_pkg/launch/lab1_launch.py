from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description ():
    talker = Node(
        package='lab1_pkg',
        executable='talker',
        parameters=[{'v':1.0,'d':0.5}],
    )

    relay = Node(
        package = 'lab1_pkg',
        executable='relay',
    )
    return LaunchDescription([talker, relay])