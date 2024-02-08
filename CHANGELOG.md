CHANGELOG
=========

2024-02-08
----------
* Add `lidar.bins` parameter to set a fixed scan bin size for improved compatibility with the SLAM Toolbox package
* Add parameter `comm.timeout_msec` 
* Change `lidar.rot_verse` parameter from integer to string
* Change `lidar.units` parameter from integer to string
* Add new `ldlidar_slam.launch.py` launch file as a demo for SLAM Toolbox integration
* Add new `slam_toolbox.yaml` parameter file for SLAM Toolbox
* Add parameters `lidar.range_min` and `lidar.range_max`
* Add parameters `lidar.enable_angle_crop`,`lidar.angle_crop_min`, and `lidar.angle_crop_max`

2024-02-03
----------
* Add support for QoS override parameters -> https://design.ros2.org/articles/qos_configurability.html#adding-a-new-mechanism-vs-using-parameters

2022-11-27
----------
* Add direct serial port connection feature

v0.2.0 - Humble
---------------
* Change license from MIT to Apache-2.0
* Code refactoring to meet [ROS2 rules](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
* Add CONTRIBUTING.md
* Add udev rules and install scripts
* Fix issue with Rviz2 config files
* Fix `nav2_lifecycle_manager` interaction in ROS2 Humble
  * Derive `ldlidar::LdLidarComponent` from `nav2_util::LifecycleNode` instead of `rclcpp::LifecycleNode` 
* Improve parameter handling
* Set default QoS to `rclcpp::SensorDataQoS`

v0.1.0
------
* ROS2 Foxy and Ubuntu 20.04
* First working version of the node based on ROS2 lifecycle architecture
* Python launch scripts
* Parameters customization
* Examples
