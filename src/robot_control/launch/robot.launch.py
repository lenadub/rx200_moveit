from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rx200_moveit_control',
            executable='input_manager_node',
            name='input_manager_node',
            output='screen'
        ),
        Node(
            package='rx200_moveit_control',
            executable='action_client',
            name='action_client',
            output='screen'
        ),
        Node(
            package='rx200_moveit_control',
            executable='rx200_gui',
            name='rx200_gui',
            output='screen'
        ),
    ])