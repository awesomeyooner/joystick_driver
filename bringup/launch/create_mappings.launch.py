from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch
import launch_ros

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(get_package_share_directory('joystick_driver'),'config','joystick.yaml'),
            description="The path to the YAML config file."
        )
    )

    joy_params = LaunchConfiguration("params_file")

    # Basic Joystick node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_params]
    )

    # Mappings node to create the mappings file
    mappings_node = Node(
        package='joystick_driver',
        executable='create_mappings',
        name='create_mappings',
        output="both"
    )

    nodes = [
        joy_node,
        # mappings_node
    ]

    return LaunchDescription(declared_arguments + nodes)