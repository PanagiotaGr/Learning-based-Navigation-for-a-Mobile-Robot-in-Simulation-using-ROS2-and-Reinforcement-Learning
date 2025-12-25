from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rl_nav',
            executable='random_walk',
            name='random_walk_node',
            output='screen',
            parameters=[
                {
                    'forward_speed': 0.15,
                    'turn_speed': 0.6,
                    'obstacle_distance': 0.5,
                }
            ],
        )
    ])
