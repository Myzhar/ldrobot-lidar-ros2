# Copyright 2024 Walter Lucetti
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###########################################################################

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    
    node_name = LaunchConfiguration('node_name')

    # Lifecycle manager configuration file
    lc_mgr_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'lifecycle_mgr_slam.yaml'
    )

    # SLAM Toolbox configuration for LDLidar
    slam_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'slam_toolbox.yaml'
    )

    # Lifecycle manager node
    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            # YAML files
            lc_mgr_config_path  # Parameters
        ]
    )

    # SLAM Toolbox node in async mode
    slam_toolbox_node = LifecycleNode(
          package='slam_toolbox',
          executable='async_slam_toolbox_node',
          namespace='',
          name='slam_toolbox',
          output='screen',
          parameters=[
            # YAML files
            slam_config_path # Parameters
          ],
          remappings=[
              ('/scan', '/ldlidar_node/scan')
          ]          
    )

    # Include LDLidar launch
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ldlidar_node'),
            '/launch/ldlidar_bringup.launch.py'
        ]),
        launch_arguments={
            'node_name': 'ldlidar_node'
        }.items()
    )

    # Fake odom publisher
    fake_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'ldlidar_base']
    )

    # RVIZ2 settings
    rviz2_config = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'config',
        'ldlidar_slam.rviz'
    )

    # RVIZ2node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[["-d"], [rviz2_config]]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Launch Nav2 Lifecycle Manager
    ld.add_action(lc_mgr_node)

    # Launch SLAM Toolbox node
    ld.add_action(slam_toolbox_node)

    # Launch fake odom publisher node
    ld.add_action(fake_odom)

    # Call LDLidar launch
    ld.add_action(ldlidar_launch)

    # Start RVIZ2
    ld.add_action(rviz2_node)

    return ld
