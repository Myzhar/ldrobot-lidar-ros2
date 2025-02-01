//  Copyright 2024 Walter Lucetti
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
////////////////////////////////////////////////////////////////////////////////

#include "ldlidar_component.hpp"
#include "ldlidar_tools.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rmw/types.h"
#include "rclcpp/parameter.hpp"
#include "rclcpp/exceptions.hpp"

using namespace std::placeholders;

namespace ldlidar
{

const int QOS_QUEUE_SIZE = 10;

LdLidarComponent::LdLidarComponent(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("lidar_node", "", options), _diagUpdater(this)
{
  RCLCPP_INFO(get_logger(), "****************************************");
  RCLCPP_INFO(get_logger(), " LDRobot DToF Lidar Lifecycle Component ");
  RCLCPP_INFO(get_logger(), "****************************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "****************************************");
  RCLCPP_INFO(
    get_logger(),
    " + State: 'unconfigured [1]'. Use lifecycle commands "
    "to configure [1] or shutdown [5]");

  // ----> Diagnostic
  _diagUpdater.add("LDLidar Diagnostic", this, &LdLidarComponent::callback_updateDiagnostic);
  _diagUpdater.setHardwareID("LDRobot lidar");
  // <---- Diagnostic
}

LdLidarComponent::~LdLidarComponent()
{
}

void LdLidarComponent::getParameters()
{
  RCLCPP_INFO_STREAM(get_logger(), "****** NODE PARAMETERS ******");

  // DEBUG parameters
  getDebugParams();

  // COMMUNICATION parameters
  getCommParams();

  // LIDAR parameters
  getLidarParams();
}

void LdLidarComponent::getDebugParams()
{
  // ----> Debug mode initialization from parameters
  getParam("general.debug_mode", _debugMode, _debugMode, "Enable debug messages");
  if (_debugMode) {
    rcutils_ret_t res = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (res != RCUTILS_RET_OK) {
      RCLCPP_INFO(get_logger(), "Error setting DEBUG level for logger");
    } else {
      RCLCPP_INFO(get_logger(), "*** Debug Mode enabled ***");
    }
  } else {
    rcutils_ret_t res = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

    if (res != RCUTILS_RET_OK) {
      RCLCPP_INFO(get_logger(), "Error setting INFO level for logger");
    }
  }

  RCLCPP_DEBUG(
    get_logger(), "[ROS2] Using RMW_IMPLEMENTATION = %s", rmw_get_implementation_identifier());
  // <---- Debug mode initialization from parameters
}

void LdLidarComponent::getCommParams()
{
  RCLCPP_INFO(get_logger(), "+++ COMMUNICATION PARAMETERS +++");

  // ----> Communication
  getParam(
    "comm.serial_port", _serialPort, _serialPort, "Communication serial port path", true,
    " * Serial port: ");

  getParam(
    "comm.timeout_msec", _readTimeOut_msec, _readTimeOut_msec, "Data reading timeout in msec",
    false, " * Timeout [msec]: ");
  // <---- Communication
}

void LdLidarComponent::getLidarParams()
{
  RCLCPP_INFO(get_logger(), "+++ LIDAR PARAMETERS +++");
  nav2_util::LifecycleNode::integer_range limits_int;

  // ----> Lidar config
  getParam("lidar.model", _lidarModel, _lidarModel, " * Lidar frame: ", "Name of the lidar frame");
  if (_lidarModel == "LDLiDAR_LD06") {
    _lidarType = ldlidar::LDType::LD_06;
  } else if (_lidarModel == "LDLiDAR_LD19") {
    _lidarType = ldlidar::LDType::LD_19;
  } else if (_lidarModel == "LDLiDAR_STL27L") {
    _lidarType = ldlidar::LDType::STL_27L;
  } else {
    RCLCPP_ERROR_STREAM(
      get_logger(), " !!! The parameter 'lidar.model' is not valid! -> " << _lidarModel.c_str() );
    exit(EXIT_FAILURE);
  }

  getParam(
    "lidar.frame_id", _frameId, _frameId, "Name of the lidar frame", true,
    " * Lidar frame: ");

  std::string units_str = "M";
  getParam("lidar.units", units_str, units_str, "Measure units", true, " * Units: ");

  if (units_str == "MM") {
    _distScale = 1.0f;
  } else if (units_str == "CM") {
    _distScale = 0.1f;
  } else {
    if (units_str != "M") {
      RCLCPP_WARN(get_logger(), "Parameter 'lidar.units' not valid, using the default value: 'M'");
    }
    _distScale = 0.001f;
  }

  std::string rot_verse = "CCW";
  getParam(
    "lidar.rot_verse", rot_verse, rot_verse, " * Rotation verse: ",
    "Rotation verse, use 'CW' if upsidedown");
  if (rot_verse == "CW") {
    _counterclockwise = false;
  } else {
    if (rot_verse != "CCW") {
      RCLCPP_WARN(
        get_logger(), "Parameter 'lidar.rot_verse' not valid, using the default value: 'CCW'");
    }
    _counterclockwise = true;
  }

  getParam(
    "lidar.bins", _bins, _bins, "Fixed number of data beams. 0 for dynamic", true,
    " * Bins: ");
  getParam(
    "lidar.range_min", _rangeMin, _rangeMin, "Minimum distance in units", false,
    " * Min. distance: ");
  getParam(
    "lidar.range_max", _rangeMax, _rangeMax, "Maximum distance in units", false,
    " * Max. distance: ");
  getParam(
    "lidar.enable_angle_crop", _enableAngleCrop, _enableAngleCrop, "Angle cropping", false,
    " * Angle cropping: ");
  getParam(
    "lidar.angle_crop_min", _angleCropMin, _angleCropMin, "Angle cropping minimum angle",
    false, " * Angle cropping min angle: ");
  getParam(
    "lidar.angle_crop_max", _angleCropMax, _angleCropMax, "Angle cropping maximum angle",
    false, " * Angle cropping max angle: ");
  // <---- Lidar config
}

nav2_util::CallbackReturn LdLidarComponent::on_configure(const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "on_configure: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                     << "] -> Inactive");

  // Initialize parameters from yaml file
  getParameters();

  // ----> Initialize topics
  _scanTopic = _topicRoot + std::string("scan");
  // <---- Initialize topics

  // ----> Initialize publisher
  rclcpp::QoS qos(QOS_QUEUE_SIZE);

  auto pub_opt = rclcpp::PublisherOptions();
  pub_opt.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

  if (!_scanPub) {
    _scanPub = this->create_publisher<sensor_msgs::msg::LaserScan>(_scanTopic, qos, pub_opt);
  }
  // <---- Initialize publisher

  // ----> Connect to Lidar
  if (!initLidar()) {
    return nav2_util::CallbackReturn::ERROR;  // Transition to Finalized state
    // Note: we could use FAILURE instead of ERROR to remain in unconfigured
    // state and try again to connect
  }
  // <---- Connect to Lidar

  RCLCPP_INFO(
    get_logger(),
    " + State: 'inactive [2]'. Use lifecycle commands to "
    "activate [3], cleanup [2] or shutdown "
    "[6]");
  return nav2_util::CallbackReturn::SUCCESS;
}  // namespace ldlidar

nav2_util::CallbackReturn LdLidarComponent::on_activate(const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(), "on_activate: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                  << "] -> Active");

  // create bond connection
  createBond();

  // Activate publisher
  _scanPub->on_activate();

  // Start lidar thread
  startLidarThread();

  RCLCPP_INFO(
    get_logger(),
    " + State: 'active [3]'. Use lifecycle commands to "
    "deactivate [4] or shutdown [7]");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LdLidarComponent::on_deactivate(const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "on_deactivate: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                      << "] -> Inactive");

  // destroy bond connection
  destroyBond();

  // Dectivate publisher
  _scanPub->on_deactivate();

  // Stop Lidar THread
  stopLidarThread();

  RCLCPP_INFO(
    get_logger(),
    " + State: 'inactive [2]'. Use lifecycle commands to "
    "activate [3], cleanup [2] or shutdown "
    "[6]");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LdLidarComponent::on_cleanup(const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(), "on_cleanup: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                 << "] -> Unconfigured");

  _scanPub.reset();

  RCLCPP_INFO(
    get_logger(),
    " + State: 'unconfigured [1]'. Use lifecycle commands "
    "to configure [1] or shutdown [5]");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LdLidarComponent::on_shutdown(const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(), "on_shutdown: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                  << "] -> Finalized");

  _scanPub.reset();

  RCLCPP_INFO_STREAM(get_logger(), " + State: 'finalized [4]'. Press Ctrl+C to kill...");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LdLidarComponent::on_error(const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(), "on_error: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                               << "] -> Finalized");

  RCLCPP_INFO_STREAM(get_logger(), " + State: 'finalized [4]'. Press Ctrl+C to kill...");
  return nav2_util::CallbackReturn::FAILURE;
}

void LdLidarComponent::publishLaserScan(ldlidar::Points2D & src, double lidar_spin_freq)
{
  float angle_min, angle_max, angle_increment;
  double scan_time;
  rclcpp::Time start_scan_time;
  static rclcpp::Time end_scan_time;
  static bool first_scan = true;

  int beam_size = 0;

  // With a fixed number of bins the compatibility with SlamToolbox is guaranteed
  if (_bins > 0) {
    beam_size = _bins;
  } else {
    beam_size = static_cast<int>(src.size());
  }

  start_scan_time = this->now();
  scan_time = (start_scan_time.seconds() - end_scan_time.seconds());

  if (first_scan) {
    first_scan = false;
    end_scan_time = start_scan_time;
    return;
  }
  // Adjust the parameters according to the demand
  angle_min = 0;
  angle_max = (2 * M_PI);
  angle_increment = (angle_max - angle_min) / (float)(beam_size - 1);
  // Calculate the number of scanning points
  if (lidar_spin_freq > 0) {
    std::unique_ptr<sensor_msgs::msg::LaserScan> msg =
      std::make_unique<sensor_msgs::msg::LaserScan>();
    msg->header.stamp = start_scan_time;
    msg->header.frame_id = _frameId;
    msg->angle_min = angle_min;
    msg->angle_max = angle_max;
    msg->range_min = _rangeMin;
    msg->range_max = _rangeMax;
    msg->angle_increment = angle_increment;
    if (beam_size <= 1) {
      msg->time_increment = 0;
    } else {
      msg->time_increment = static_cast<float>(scan_time / (double)(beam_size - 1));
    }
    msg->scan_time = scan_time;
    // First fill all the data with Nan
    msg->ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    msg->intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    for (auto point : src) {
      float range = point.distance * _distScale;  // distance unit transform to meters
      float intensity = point.intensity;      // laser receive intensity
      float dir_angle = point.angle;

      if ((point.distance == 0) && (point.intensity == 0)) { // filter is handled to  0, Nan will be assigned variable.
        range = std::numeric_limits<float>::quiet_NaN();
        intensity = std::numeric_limits<float>::quiet_NaN();
      }

      if (_enableAngleCrop) { // Angle crop setting, Mask data within the set angle range
        if ((dir_angle >= _angleCropMin) && (dir_angle <= _angleCropMax)) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }
      }

      float angle = ANGLE_TO_RADIAN(dir_angle); // Lidar angle unit form degree transform to radian
      int index = static_cast<int>(ceil((angle - angle_min) / angle_increment));
      if (index < beam_size) {
        if (index < 0) {
          RCLCPP_ERROR(
            get_logger(),
            "error index: %d, beam_size: %d, angle: %f, msg->angle_min: %f, msg->angle_increment: %f",
            index, beam_size, angle, angle_min, angle_increment);
        }

        if (_counterclockwise) {
          int index_anticlockwise = beam_size - index - 1;
          // If the current content is Nan, it is assigned directly
          if (std::isnan(msg->ranges[index_anticlockwise])) {
            msg->ranges[index_anticlockwise] = range;
          } else { // Otherwise, only when the distance is less than the current
                   //   value, it can be re assigned
            if (range < msg->ranges[index_anticlockwise]) {
              msg->ranges[index_anticlockwise] = range;
            }
          }
          msg->intensities[index_anticlockwise] = intensity;
        } else {
          // If the current content is Nan, it is assigned directly
          if (std::isnan(msg->ranges[index])) {
            msg->ranges[index] = range;
          } else { // Otherwise, only when the distance is less than the current
                   //   value, it can be re assigned
            if (range < msg->ranges[index]) {
              msg->ranges[index] = range;
            }
          }
          msg->intensities[index] = intensity;
        }
      }
    }
    _scanPub->publish(std::move(msg));
    end_scan_time = start_scan_time;
  }
}

bool LdLidarComponent::initLidar()
{
  _lidar = std::make_unique<ldlidar::LDLidarDriver>();

  _lidar->RegisterGetTimestampFunctional(std::bind(&tools::GetSystemTimeStamp));
  _lidar->EnableFilterAlgorithnmProcess(true);

  if (!initLidarComm()) {
    return false;
  }

  return true;
}

bool LdLidarComponent::initLidarComm()
{
  if (_lidar->Start(_lidarType, _serialPort, _baudrate, ldlidar::COMM_SERIAL_MODE)) {
    RCLCPP_INFO_STREAM(get_logger(), "***** LDLidar opened on port '" << _serialPort << "' *****");
  } else {
    RCLCPP_ERROR(get_logger(), "!!! LDLidar not opened !!!");
    exit(EXIT_FAILURE);
  }

  if (_lidar->WaitLidarCommConnect(3000)) {
    RCLCPP_INFO(get_logger(), " * LDLidar communication OK");
  } else {
    RCLCPP_ERROR(get_logger(), " !!! LDLidar communication KO !!!");
    exit(EXIT_FAILURE);
  }

  return true;
}

void LdLidarComponent::startLidarThread()
{
  _lidarThread = std::thread(&LdLidarComponent::lidarThreadFunc, this);
}

void LdLidarComponent::stopLidarThread()
{
  if (!_threadStop) {
    _threadStop = true;
    RCLCPP_DEBUG(get_logger(), "Stopping lidar thread...");
    try {
      if (_lidarThread.joinable()) {
        _lidarThread.join();
      }
    } catch (std::system_error & e) {
      RCLCPP_WARN(get_logger(), "Lidar thread joining exception: %s", e.what());
    }
  }
}

void LdLidarComponent::lidarThreadFunc()
{
  RCLCPP_DEBUG(get_logger(), "Lidar thread started");

  _threadStop = false;
  _publishing = false;

  ldlidar::Points2D laser_scan_points;
  double lidar_scan_freq;

  while (1) {
    // ----> Interruption check
    if (!rclcpp::ok()) {
      RCLCPP_DEBUG(get_logger(), "Ctrl+C received: stopping grab thread");
      _threadStop = true;
    }

    if (_threadStop) {
      RCLCPP_DEBUG(get_logger(), "Lidar thread stopped");
      break;
    }
    // <---- Interruption check

    int nSub = count_subscribers(_scanTopic);
    if (nSub > 0) {
      _publishing = true;
      switch (_lidar->GetLaserScanData(laser_scan_points, _readTimeOut_msec)) {
        case ldlidar::LidarStatus::NORMAL:
          _lidar->GetLidarScanFreq(lidar_scan_freq);
          publishLaserScan(laser_scan_points, lidar_scan_freq);
          break;
        case ldlidar::LidarStatus::DATA_TIME_OUT:
          RCLCPP_ERROR(
            get_logger(), "get ldlidar data is time out, please check your lidar device.");
          break;
        case ldlidar::LidarStatus::DATA_WAIT:
          break;
        default:
          break;
      }
    } else {
      _publishing = false;
    }

    // Sleep until the next scan is ready
    using std::chrono::nanoseconds;
    if (_lidar->GetLidarScanFreq(lidar_scan_freq) && lidar_scan_freq != 0.0) {
      rclcpp::sleep_for(nanoseconds(int64_t(1e9 / lidar_scan_freq)));
    } else {
      rclcpp::sleep_for(nanoseconds(int64_t(1e9 / 10)));
    }
  }

  RCLCPP_DEBUG(get_logger(), "Lidar thread finished");
}

void LdLidarComponent::callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp_lifecycle::State state = get_current_state();

  if (state.id() == 3) {  // ACTIVE
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK, std::string(
        "Node state: ") + state.label());

    double spin_hz;
    _lidar->GetLidarScanFreq(spin_hz);
    stat.addf("Single measure", "%.3f Hz", spin_hz);

    if (_publishing) {
      stat.addf("Publishing", "%.3f Hz", _pubFreq);
    } else {
      stat.add("Publishing", "NO SUBSCRIBERS");
    }
  } else if (state.id() == 1 || state.id() == 2) {  // UNCONFIGURED || INACTIVE
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::STALE, std::string(
        "Node state: ") + state.label());
  } else {  // SHUTDOWN || ERROR
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::string(
        "Node state: ") + state.label());
  }
}

}  // namespace ldlidar

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(ldlidar::LdLidarComponent)
