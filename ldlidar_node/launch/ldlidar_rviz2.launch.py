#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Set LOG format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{severity}] ({name}) {message}'
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

    node_name = LaunchConfiguration('node_name')

    # Launch arguments
    declare_node_name_cmd = DeclareLaunchArgument(
        'node_name',
        default_value='ldlidar_node',
        description='Name of the node'
    )

    # RViZ2 settings
    rviz2_config = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'rviz2',
        'ldlidar.rviz'
    )

    # Lifecycle manager node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[rviz2_config]
    )

    # Include LDLidar with lifecycle manager launch
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ldlidar_node'),
            '/launch/ldlidar_with_mgr.launch.py'
        ]),
        launch_arguments={
            'node_name': node_name
        }.items()
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(declare_node_name_cmd)

    # Launch Nav2 Lifecycle Manager
    ld.add_action(rviz2_node)

    # Call LDLidar launch
    ld.add_action(ldlidar_launch)

    return ld
    