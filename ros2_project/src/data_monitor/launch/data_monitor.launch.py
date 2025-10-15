#!/usr/bin/env python3
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                name="get_end_position",
                package='control_server',
                executable='get_end_position',
                output="both",
            ),
        ]
    )