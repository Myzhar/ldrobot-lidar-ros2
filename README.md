# ldrobot-lidar-ros2
ROS2 package for LDRobot lidar. Based on ROS2 Lifecycle nodes

## Start the node

### Default parameters

Open a terminal console and enter the following command:

    $ ros2 run ldlidar_node ldlidar_node

the `ldlidar` node is based on the [`ROS2 lifecycle` architecture](https://design.ros2.org/articles/node_lifecycle.html), hence it starts in `UNCONFIGURED` state.
To configure the node, set all the parameters to the default value, activate the publisher,and try to estabilish a connection, the lifecycle services must be called.

Open a new terminal console and enter the following command: 

    $ ros2 lifecycle set /lidar_node configure

`Transitioning successful` is returned if the node is correctly configured and the connection is estabilished, `Transitioning failed` in case of errors. Look at the node log for information about eventual connection problems.

The node is now in `INACTIVE` state, enter the following command to activate:

    $ ros2 lifecycle set /lidar_node activate
    
The node is now activated and the `/lidar_node/scan` topic of type `sensor_msgs/msg/LaserScan` is available to be subscribed.

### Load parameter from launch file and start a Lifecycle manager

The [parameter of the node](#parameters) can be modified by editing the [`lidar_node/config/ldlidar.yaml`](ldlidar_node/config/ldlidar.yaml) file.

Furthermore, thanks to the [NAV2](https://navigation.ros.org/index.html) project it is possible to launch a [`lifecycle_manager`](https://navigation.ros.org/configuration/packages/configuring-lifecycle.html) that will take care of processing the state transitions described above.

An example Python launch file is provided in [`lidar_node/launch/ldlidar_with_mgr.launch.py`](ldlidar_node/launch/ldlidar_with_mgr.launch.py) that illustrates how to start a `ldlidar_node` that loads the parameters from the`'ldlidar.yaml` file and starts the `lifecycle_manager` correctly configure with the `lifecycle_mgr.yaml` file to manage the lifecycle processing:

    $ ros2 launch ldlidar_node ldlidar_with_mgr.launch.py

## Parameters

Following the list of node parameters:

* `general.debug_mode`: set to `true` to activate debug messages
* `comm.direct_serial`: set to `false` to use the USB <-> UART converter, `true` for direct UART connection 
* `comm.serial_port`: the serial port path if using direct UART connection
* `lidar.frame_id`: TF frame name for the lidar
* `lidar.rot_verse`: 0 -> `CLOCKWISE` / 1 -> `COUNTERCLOCKWISE` [ROS STANDARD]
* `lidar.units`: 0 -> `MILLIMETERS` / 1 -> `CENTIMETERS` / 2 -> `METERS` [ROS STANDARD]
* `lidar.qos_history`: 0 -> `RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT` / 1 -> `RMW_QOS_POLICY_HISTORY_KEEP_LAST` / 2 -> `RMW_QOS_POLICY_HISTORY_KEEP_ALL`
* `lidar.qos_depth`: History depth in case of `RMW_QOS_POLICY_HISTORY_KEEP_LAST` 
* `lidar.qos_reliability`: 0 -> `RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT` / 1 -> `RMW_QOS_POLICY_RELIABILITY_RELIABLE` / 2 -> `RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT`
* `lidar.qos_durability`: 0 -> `RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT` / 1 -> `RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL` / 2 -> `RMW_QOS_POLICY_DURABILITY_VOLATILE`

# TODO
* Rviz2 launch
* URDF/xacro
* Node diagnostic






