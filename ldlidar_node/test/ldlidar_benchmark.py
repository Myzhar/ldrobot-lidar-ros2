# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0
"""
Live benchmarking LD Lidar node.

The graph consists of the following:
- Graph under Test:
    1. LdLidarComponent: LD Lidar

Required:
- Packages:
    - ldlidar_component
"""

import os

from ament_index_python.packages import get_package_share_directory


from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ros2_benchmark import BasicPerformanceCalculator, BenchmarkMode
from ros2_benchmark import ROS2BenchmarkConfig, ROS2BenchmarkTest
from ros2_benchmark import MonitorPerformanceCalculatorsInfo

def launch_setup(container_prefix, container_sigterm_timeout):
    """Generate launch description for live benchmarking LD Lidar"""
    # Lifecycle manager configuration file
    lc_mgr_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'lifecycle_mgr.yaml'
    )

    # Lifecycle manager node
    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        namespace=TestLDLidarNode.generate_namespace(),
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            # YAML files
            lc_mgr_config_path  # Parameters
        ]
    )

    # LD Lidar
    ldlidar_config_file_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params', 'ldlidar.yaml')

    ldlidar_node = ComposableNode(
        name='ldlidar_node',
        namespace=TestLDLidarNode.generate_namespace(),
        package='ldlidar_component',
        plugin='ldlidar::LdLidarComponent',
        parameters=[ldlidar_config_file_path]
    )

    scan_monitor_node = ComposableNode(
        name='ScanMonitorNode',
        namespace=TestLDLidarNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::MonitorNode',
        parameters=[{
            'monitor_index': 0,
            'monitor_data_format': 'sensor_msgs/msg/LaserScan',
        }],
        remappings=[('output', 'ldlidar_node/scan')]
    )

    composable_node_container = ComposableNodeContainer(
        name='container',
        namespace=TestLDLidarNode.generate_namespace(),
        package='rclcpp_components',
        executable='component_container_mt',
        prefix=container_prefix,
        sigterm_timeout=container_sigterm_timeout,
        composable_node_descriptions=[
            ldlidar_node,
            scan_monitor_node
        ],
        output='screen'
    )

    return [lc_mgr_node,composable_node_container]

def generate_test_description():
    return TestLDLidarNode.generate_test_description_with_nsys(launch_setup)


class TestLDLidarNode(ROS2BenchmarkTest):
    """Live performance test for LD Lidar."""

    # Custom configurations
    config = ROS2BenchmarkConfig(
        benchmark_name='LD Lidar Live Benchmark',
        benchmark_mode=BenchmarkMode.LIVE,
        benchmark_duration=5,
        test_iterations=5,
        collect_start_timestamps_from_monitors=True,
        monitor_info_list=[
            MonitorPerformanceCalculatorsInfo(
                'monitor_node0',
                [BasicPerformanceCalculator({
                    'report_prefix': 'Scan',
                    'message_key_match': True
                })])
        ]
    )

    def test_benchmark(self):
        self.run_benchmark()