from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
        ),
        Node(
            package='my_turtle_controllers',
            executable='draw_circle',
            name='turtle1_controller',
            parameters=[{'turtle_name': 'turtle1'}],
        ),
        Node(
            package='my_turtle_controllers',
            executable='draw_circle',
            name='turtle2_controller',
            parameters=[{'turtle_name': 'turtle2'}],
        ),
    ])
