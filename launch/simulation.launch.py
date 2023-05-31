#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('asv_path_planner'),
        'config',
        'params.yaml'
        )


    simulation_node = Node(
        package="asv_path_planner",
        executable="ros_script",
        parameters=[config]
    )

    ld.add_action(simulation_node)
    return ld