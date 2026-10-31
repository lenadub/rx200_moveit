import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    hardware_type_arg = DeclareLaunchArgument(
        'hardware_type',
        default_value='fake',
        description='Robot hardware type (fake or real)'
    )

    interbotix_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('interbotix_xsarm_moveit'), 'launch', 'xsarm_moveit.launch.py')),
        launch_arguments={'robot_model': 'rx200', 'hardware_type': LaunchConfiguration('hardware_type')}.items())

    input_manager = Node(
        package='rx200_moveit_control',
        executable='input_manager_node',
        name='input_manager_node',
        output='screen'
    )
    action_client = Node(
        package='rx200_moveit_control',
        executable='action_client',
        name='action_client',
        output='screen'
    )
    rx200_gui = Node(
        package='rx200_moveit_control',
        executable='rx200_gui',
        name='rx200_gui',
        output='screen'
    )
    return LaunchDescription([
        hardware_type_arg,
        interbotix_launch,
        input_manager,
        action_client,
        rx200_gui
    ])