# Copyright 2026 Walter Lucetti
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
#
# Same as `ldlidar_slam.launch.py`, but the `odom -> ldlidar_base` TF is
# provided by the YALIO PL-ICP laser odometry component instead of a fake
# static transform. The YALIO component is loaded in the same container as
# the lidar and SLAM Toolbox.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
    LoadComposableNodes
)
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

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

    # YALIO odometry configuration
    yalio_config_path = os.path.join(
        get_package_share_directory('yalio'),
        'config',
        'yalio.yaml'
    )

    # ROS 2 Component Container
    container_name='slam_demo_container'
    distro = os.environ['ROS_DISTRO']
    if distro == 'foxy':
        # Foxy does not support the isolated mode
        container_exec='component_container'
    else:
        container_exec='component_container_isolated'
    demo_container = ComposableNodeContainer(
                name=container_name,
                namespace='',
                package='rclcpp_components',
                executable=container_exec,
                composable_node_descriptions=[
                ],
                output='screen',
        )

    # Lifecycle manager node
    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            # YAML files
            lc_mgr_config_path,  # Parameters
            # Manage YALIO too; activation order: lidar -> odometry -> SLAM
            {'node_names': ['ldlidar_node', 'yalio', 'slam_toolbox']}
        ]
    )

    # YALIO PL-ICP laser odometry node: replaces the fake `odom -> ldlidar_base`
    # static TF of `ldlidar_slam.launch.py` with real scan-matching odometry.
    yalio_component = ComposableNode(
        package='yalio_component',
        namespace='',
        plugin='yalio::YalioComponent',
        name='yalio',
        parameters=[
            # YAML files
            yalio_config_path,  # Parameters
            # Match the LDLidar TF tree: URDF publishes
            # `ldlidar_base -> ldlidar_link`; YALIO closes the chain with
            # `odom -> ldlidar_base` (the lidar is at the base origin in 2D)
            {
                'frames.odom_frame': 'odom',
                'frames.base_frame': 'ldlidar_base',
                'frames.laser_frame': 'ldlidar_link',
                'frames.publish_tf': True,
                'frames.laser_x': 0.0,
                'frames.laser_y': 0.0,
                'frames.laser_yaw': 0.0
            }
        ],
        remappings=[
            ('scan', '/ldlidar_node/scan'),
            ('odom', '/odom_icp'),
            ('reset_odometry', '/yalio/reset_odometry'),
            ('set_pose', '/yalio/set_pose')
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # SLAM Toolbox node in async mode
    slam_toolbox_component = ComposableNode(
        package='slam_toolbox',
        namespace='',
        plugin='slam_toolbox::AsynchronousSlamToolbox',
        name='slam_toolbox',
        parameters=[
            # YAML files
            slam_config_path, # Parameters
        ],
        remappings=[
            ('/scan', '/ldlidar_node/scan')
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # YALIO and SLAM Toolbox Lifecycle nodes in container
    full_container_name = '/' + container_name

    load_composable_node = LoadComposableNodes(
        target_container=full_container_name,
        composable_node_descriptions=[
            yalio_component,
            slam_toolbox_component
        ]
    )

    # Include LDLidar launch
    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ldlidar_node'),
            '/launch/ldlidar_bringup.launch.py'
        ]),
        launch_arguments={
            'node_name': 'ldlidar_node',
            'container_name': container_name
        }.items()
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

    # Node Container
    ld.add_action(demo_container)

    # Load YALIO and SLAM Toolbox nodes in the container
    ld.add_action(load_composable_node)

    # Call LDLidar launch
    ld.add_action(ldlidar_launch)

    # Start RVIZ2
    ld.add_action(rviz2_node)

    return ld
