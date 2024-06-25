<h1 align="center">
  ldrobot-lidar-ros2
</h1>

<h4 align="center">ROS 2 package for LDRobot lidar. Based on Nav2 Lifecycle nodes</h4>

<p align="center">
  <a href="#get-the-lidar">Get the lidar</a> •
  <a href="#the-node-in-action">YouTube videos</a> •
  <a href="#install-the-node">Install</a> •
  <a href="#start-the-node">Start the Node</a> •
  <a href="#parameters">Parameters</a> •
  <a href="#display-scan-on-rviz2">RViz2</a> •  
  <a href="#integrate-the-node-in-your-robot">Robot integration</a> • 
  <a href="#benchmarking">Benchmarking</a>
</p>
<br>

This node is designed to work with the DToF 2D Lidar sensors [LD19](https://www.ldrobot.com/product/en/112) made by [LDRobot](https://www.ldrobot.com/en).

It can work also with the [LD06](https://www.ldrobot.com/product/en/98) model, but no tests have been performed with it. LD06 cannot work outdoor.

LD19             |  LD06
:-------------------------:|:-------------------------:
![ld19](https://user-images.githubusercontent.com/3648617/204473718-803d25d9-605a-4eaa-a047-d5d3524eead8.png)  |  ![ld06](https://user-images.githubusercontent.com/3648617/204473720-97f72c31-188e-4f5c-b98b-1033a5afe91e.png)

## Get the lidar

My lidar (LD19) comes from the [LDRobot kickstarter campaing](https://www.kickstarter.com/projects/ldrobot/ld-air-lidar-360-tof-sensor-for-all-robotic-applications) ended in 2021.

LDRobot then created also an [Indiegogo campaign](https://www.indiegogo.com/projects/ld-air-lidar-tof-sensor-for-robotic-applications--3#/) for the LD19.

LDRobot today distributes the Lidar through third-party resellers:

* Waveshare: [LD19](https://www.waveshare.com/wiki/DTOF_LIDAR_LD19)
* Innomaker: [LD19](https://www.inno-maker.com/product/lidar-ld06/)
* Other: [Search on Google](https://www.google.com/search?q=ld19+lidar&newwindow=1&sxsrf=ALiCzsb2xd4qTTA78N00mP9-PP5HY4axZw:1669710673586&source=lnms&tbm=shop&sa=X&ved=2ahUKEwjYns78_NL7AhVLVfEDHf2PDk8Q_AUoA3oECAIQBQ&cshid=1669710734415350&biw=1862&bih=882&dpr=1)

## The node in action

LD19 Lifecycle            |  LD19 outdoor
:-------------------------:|:-------------------------:
[![LD19 Lifecycle](https://img.youtube.com/vi/mbKwmK3Yjus/mqdefault.jpg)](https://youtu.be/mbKwmK3Yjus) | [![LD19 outdoor](https://img.youtube.com/vi/zyggXjW6cDo/mqdefault.jpg)](https://youtu.be/zyggXjW6cDo)

## Install the node

The node is designed to work in [ROS2 Humble](https://docs.ros.org/en/humble/index.html).

Clone the repository in your ROS2 workspace:

    cd ~/ros2_ws/src/ #use your current ros2 workspace folder
    git clone https://github.com/Myzhar/ldrobot-lidar-ros2.git

Add dependencies:

    sudo apt install libudev-dev

Install the udev rules

    cd ~/ros2_ws/src/ldrobot-lidar-ros2/scripts/
    ./create_udev_rules.sh

Build the packages:

    cd ~/ros2_ws/
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

Update the environment variables:

    echo source $(pwd)/install/local_setup.bash >> ~/.bashrc
    source ~/.bashrc

## Start the node

### Default parameters

Open a terminal console and enter the following command:

    ros2 run ldlidar_node ldlidar_node # <--- not recommended. Better using the launch file 

the `ldlidar` node is based on the [`ROS2 lifecycle` architecture](https://design.ros2.org/articles/node_lifecycle.html), hence it starts in the `UNCONFIGURED` state.
To configure the node, setting all the parameters to the default value, trying to estabilish a connection, and activating the scan publisher, the lifecycle services must be called.

Open a new terminal console and enter the following command: 

    ros2 lifecycle set /lidar_node configure

`Transitioning successful` is returned if the node is correctly configured and the connection is estabilished, `Transitioning failed` in case of errors. Look at the node log for information about eventual connection problems.

The node is now in the `INACTIVE` state, enter the following command to activate:

    ros2 lifecycle set /lidar_node activate
    
The node is now activated and the `/ldlidar_node/scan` topic of type `sensor_msgs/msg/LaserScan` is available to be subscribed.

### Launch file with YAML parameters

The default values of the [parameters of the node](#parameters) can be modified by editing the file [`ldlidar.yaml`](ldlidar_node/config/ldlidar.yaml).

Open a terminal console and enter the following command to start the node with customized parameters:

    ros2 launch ldlidar_node ldlidar.launch.py
    
The [`ldlidar.yaml`](ldlidar_node/config/ldlidar.yaml) script also starts a `robot_state_publisher` node that provides the static TF transform of the LDLidar [`ldlidar_base`->`ldlidar_link`], and provides the ldlidar description in the `/robot_description`.

![](./images/ldlidar_tf.png)

### Launch file with YAML parameters and Lifecycle manager

Thanks to the [NAV2](https://navigation.ros.org/index.html) project it is possible to launch a [`lifecycle_manager`](https://navigation.ros.org/configuration/packages/configuring-lifecycle.html) node that is taking care of processing the state transitions described above.

An example launch file is provided, [`ldlidar_with_mgr.launch.py`](ldlidar_node/launch/ldlidar_with_mgr.launch.py), that illustrates how to start a `ldlidar_node` that loads the parameters from the `ldlidar.yaml` file, and starts the `lifecycle_manager` correctly configured with the file [`lifecycle_mgr.yaml`](ldlidar_node/config/lifecycle_mgr.yaml) to manage the lifecycle processing:

    ros2 launch ldlidar_node ldlidar_with_mgr.launch.py

The `ldlidar_with_mgr.launch.py` script automatically starts the `ldlidar_node` by including the `ldlidar.launch.py` launch file.

## Parameters

Following the list of node parameters:

* **`general.debug_mode`**: set to `true` to activate debug messages
* **`comm.serial_port`**: the serial port path
* **`comm.baudrate`**: the serial port baudrate
* **`comm.timeout_msec`**: the serial communication timeout in milliseconds
* **`lidar.model`**: Lidar model [LDLiDAR_LD06, LDLiDAR_LD19, LDLiDAR_STL27L]
* **`lidar.rot_verse`**: The rotation verse. Use clockwise if the lidar is mounted upsidedown. [CW, CCW]
* **`lidar.units`**: distance measurement units [M, CM, MM]
* **`lidar.frame_id`**: TF frame name for the lidar
* **`lidar.bins`**: set to 0 for dinamic scan size according to rotation speed, set to a fixed value [e.g. 455] for compatibility with SLAM Toolbox
* **`lidar.range_min`**: minimum scan distance
* **`lidar.range_max`**: maximum scan distance
* **`lidar.enable_angle_crop`**: enable angle cropping
* **`lidar.angle_crop_min`**: minimum cropping angle
* **`lidar.angle_crop_max`**: maximum cropping angle

## Display scan on RVIZ2

The launch file `ldlidar_rviz2.launch.py` starts the `ldlidar_node` node, the `lifecycle_manager` node, and a precofigured instance of RViz2 to display the 2D laser scan provided by the LDRobot sensors. This is an example to demonstrate how to correctly setup RViz2 to be used with the `ldlidar_node` node.

Open a terminal console and enter the following command:

    ros2 launch ldlidar_node ldlidar_rviz2.launch.py

![](./images/ldlidar_rviz2.png)

## Integrate the node in your robot

Follow the following procedure, to integrate the `ldlidar_node` in a robot configuration:

* Provide a TF transform from `base_link` to `ldlidar_base`, that is placed in the center of the base of the lidar scanner. The `ldlidar_base` -> `ldlidar_link` transform is provided by the `robot_state_publisher` started by the `ldlidar.launch.py` launch file.
* Modify the [`ldlidar.yaml`](ldlidar_node/config/ldlidar.yaml) to match the configuration of the robot.
* Include the [`ldlidar.launch.py`](ldlidar_node/launch/ldlidar.launch.py) in the bringup launch file of the robot. Follow the [provided example](#launch-file-with-yaml-parameters-and-lifecycle-manager).
* Handle lifecycle to correctly start the node. You can use the Nav2 `lifecycle_manager`, by including it in the bringup launch file. Follow the [provided example](#launch-file-with-yaml-parameters-and-lifecycle-manager).
* Enjoy your working system

## SLAM Toolbox example

The launch file `ldlidar_slam.launch.py` shows how to use the node with the [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) package to generate a 2D map for robot navigation.

![](./images/ld19_slam.png)

## Benchmarking

It is possible to benchmark the node to evaluate the overall performance by using the [NVIDIA® ISAAC ROS ros2_benchmark package](https://github.com/NVIDIA-ISAAC-ROS/ros2_benchmark).

First of all install the [ros2_benchmark package](https://github.com/NVIDIA-ISAAC-ROS/ros2_benchmark/tree/main?tab=readme-ov-file#quickstart).

Launch the benchmark:

    cd ~/ros2_ws/src/ldrobot-lidar-ros2/test/
    launch_test ldlidar_benchmark.py

the final result should be similar to 

```
+--------------------------------------------------------------------------------------------+
|                                  LD Lidar Live Benchmark                                   |
|                                        Final Report                                        |
+--------------------------------------------------------------------------------------------+
| [Scan] Delta between First & Last Received Frames (ms) : 4910.833                          |
| [Scan] Mean Playback Frame Rate (fps) : 10.182                                             |
| [Scan] Mean Frame Rate (fps) : 10.182                                                      |
| [Scan] # of Missed Frames : 0.000                                                          |
| [Scan] # of Frames Sent : 50.000                                                           |
| [Scan] First Sent to First Received Latency (ms) : 0.156                                   |
| [Scan] Last Sent to Last Received Latency (ms) : 0.235                                     |
| [Scan] First Frame End-to-end Latency (ms) : 0.156                                         |
| [Scan] Last Frame End-to-end Latency (ms) : 0.235                                          |
| [Scan] Max. End-to-End Latency (ms) : 0.265                                                |
| [Scan] Min. End-to-End Latency (ms) : 0.093                                                |
| [Scan] Mean End-to-End Latency (ms) : 0.189                                                |
| [Scan] Max. Frame-to-Frame Jitter (ms) : 0.594                                             |
| [Scan] Min. Frame-to-Frame Jitter (ms) : 0.000                                             |
| [Scan] Mean Frame-to-Frame Jitter (ms) : 0.218                                             |
| [Scan] Frame-to-Frame Jitter Std. Deviation (ms) : 0.145                                   |
+--------------------------------------------------------------------------------------------+
| Baseline Overall CPU Utilization (%) : 8.333                                               |
| Max. Overall CPU Utilization (%) : 66.667                                                  |
| Min. Overall CPU Utilization (%) : 0.000                                                   |
| Mean Overall CPU Utilization (%) : 0.856                                                   |
| Std Dev Overall CPU Utilization (%) : 3.420                                                |
+--------------------------------------------------------------------------------------------+
| [metadata] Test Name : LD Lidar Live Benchmark                                             |
| [metadata] Test File Path : /home/walter/devel/ros2/ros2_walt/src/ldrobot-lidar-ros2/ldlidar_node/test/ldlidar_benchmark.py |
| [metadata] Test Datetime : 2024-06-25T20:05:40Z                                            |
| [metadata] Device Hostname : walter-Legion-5-15ACH6H                                       |
| [metadata] Device Architecture : x86_64                                                    |
| [metadata] Device OS : Linux 6.5.0-25-generic #25~22.04.1-Ubuntu SMP PREEMPT_DYNAMIC Tue Feb 20 16:09:15 UTC 2 |
| [metadata] Idle System CPU Util. (%) : 0.500                                               |
| [metadata] Benchmark Mode : 3                                                              |
+--------------------------------------------------------------------------------------------+
```






