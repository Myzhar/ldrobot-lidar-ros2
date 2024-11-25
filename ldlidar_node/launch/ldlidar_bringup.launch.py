# Copyright 2022 Walter Lucetti
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import (
    Node,
    ComposableNodeContainer
)
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    node_ns = LaunchConfiguration('node_namespace')
    node_name = LaunchConfiguration('node_name')

    # Lidar node configuration file
    lidar_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'ldlidar.yaml'
    )

    # Launch arguments
    declare_node_namespace_cmd = DeclareLaunchArgument(
        'node_namespace',
        default_value='',
        description='Namespace of the node'
    )
    declare_node_name_cmd = DeclareLaunchArgument(
        'node_name',
        default_value='ldlidar_node',
        description='Name of the node'
    )

    # LDLidar component
    ldlidar_component = ComposableNode(
            package='ldlidar_component',
            namespace=node_ns,
            plugin='ldlidar::LdLidarComponent',
            name=node_name,
            parameters=[
                # YAML files
                lidar_config_path  # Parameters
            ],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    # ROS 2 Component Container
    distro = os.environ['ROS_DISTRO']
    if distro == 'foxy':
        # Foxy does not support the isolated mode
        container_exec='component_container'
    else:
        container_exec='component_container_isolated'
    
    ldlidar_container = ComposableNodeContainer(
            name='zed_container',
            namespace=node_ns,
            package='rclcpp_components',
            executable=container_exec,
            composable_node_descriptions=[
                ldlidar_component
            ],
            output='screen',
    )

    # URDF path
    urdf_file_name = 'ldlidar_descr.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=node_ns,
        name='ldlidar_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(declare_node_namespace_cmd)
    ld.add_action(declare_node_name_cmd)

    # Launch Robot State Publisher node
    ld.add_action(rsp_node)

    # LDLidar Lifecycle node in container
    ld.add_action(ldlidar_container)

    return ld
