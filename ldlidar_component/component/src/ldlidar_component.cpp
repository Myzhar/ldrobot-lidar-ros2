//  Copyright 2022 Walter Lucetti
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
LdLidarComponent::LdLidarComponent(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("lidar_node", "", options), mDiagUpdater(this)
{
  RCLCPP_INFO(get_logger(), "****************************************");
  RCLCPP_INFO(get_logger(), " LDRobot DToF Lidar Lifecycle Component ");
  RCLCPP_INFO(get_logger(), "****************************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "****************************************");
  RCLCPP_INFO(
    get_logger(),
    "State: 'unconfigured [1]'. Use lifecycle commands "
    "to configure [1] or shutdown [5]");

  // ----> Diagnostic
  mDiagUpdater.add("LDLidar Diagnostic", this, &LdLidarComponent::callback_updateDiagnostic);
  mDiagUpdater.setHardwareID("LDRobot lidar");
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
  getParam("general.debug_mode", mDebugMode, mDebugMode, "Enable debug messages");
  if (mDebugMode) {
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
    "comm.direct_serial", mUseDirectSerial, mUseDirectSerial,
    "Set to 'true' if not using the UART<->USB converted [not yet available]");
  RCLCPP_INFO_STREAM(
    get_logger(), " * Direct Serial comm: " << (mUseDirectSerial ? "TRUE" : "FALSE"));

  if (mUseDirectSerial) {
    getParam("comm.serial_port", mUseDirectSerial, mUseDirectSerial, "* Serial port: ");
  }
  // <---- Communication
}

void LdLidarComponent::getLidarParams()
{
  RCLCPP_INFO(get_logger(), "+++ LIDAR PARAMETERS +++");
  nav2_util::LifecycleNode::integer_range limits_int;

  // ----> Lidar config
  getParam("lidar.frame_id", mFrameId, mFrameId, " * Lidar frame: ", "Name of the lidar frame");
  int rotVerse = static_cast<int>(mRotVerse);

  limits_int = {0, static_cast<int>(ROTATION::COUNTERCLOCKWISE), 1};
  getParam(
    "lidar.rot_verse", rotVerse, rotVerse, limits_int,
    "Change rotation verse. To be used if the lidar is mounted upside down.", true);
  mRotVerse = static_cast<ROTATION>(rotVerse);
  RCLCPP_INFO_STREAM(get_logger(), " * Rotation verse: " << tools::to_string(mRotVerse));

  limits_int = {0, static_cast<int>(UNITS::METERS), 1};
  int units = static_cast<int>(mUnits);
  getParam("lidar.units", units, units, limits_int, "Set measurement units");
  mUnits = static_cast<UNITS>(units);
  RCLCPP_INFO_STREAM(get_logger(), " * Measure units: " << tools::to_string(mUnits));
  // <---- Lidar config
}

template<typename T>
void LdLidarComponent::getParam(
  std::string paramName, T defValue, T & outVal,
  const std::string & description, bool read_only, std::string log_info)
{
  try {
    add_parameter(paramName, rclcpp::ParameterValue(defValue), description, "", read_only);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Exception: " << ex.what());
  }

  if (!get_parameter(paramName, outVal)) {
    RCLCPP_WARN_STREAM(
      get_logger(), "The parameter '"
        << paramName << "' is not available or is not valid, using the default value: "
        << defValue);
  }

  if (!log_info.empty()) {
    RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
  }
}

void LdLidarComponent::getParam(
  std::string paramName, int defValue, int & outVal,
  const nav2_util::LifecycleNode::integer_range & range,
  const std::string & description,
  bool read_only,
  std::string log_info)
{
  try {
    add_parameter(
      paramName, rclcpp::ParameterValue(defValue), range,
      description, "", read_only);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Exception: " << ex.what());
  }

  if (!get_parameter(paramName, outVal)) {
    RCLCPP_WARN_STREAM(
      get_logger(), "The parameter '"
        << paramName << "' is not available or is not valid, using the default value: "
        << defValue);
  }

  if (!log_info.empty()) {
    RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
  }
}

void LdLidarComponent::getParam(
  std::string paramName, float defValue, float & outVal,
  const nav2_util::LifecycleNode::floating_point_range & range,
  const std::string & description,
  bool read_only,
  std::string log_info)
{
  try {
    add_parameter(
      paramName, rclcpp::ParameterValue(defValue), range,
      description, "", read_only);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Exception: " << ex.what());
  }

  if (!get_parameter(paramName, outVal)) {
    RCLCPP_WARN_STREAM(
      get_logger(), "The parameter '"
        << paramName << "' is not available or is not valid, using the default value: "
        << defValue);
  }

  if (!log_info.empty()) {
    RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
  }
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
  mScanTopic = mTopicRoot + std::string("scan");
  // <---- Initialize topics

  // ----> Initialize publisher
  if (!mScanPub) {
    mScanPub = this->create_publisher<sensor_msgs::msg::LaserScan>(mScanTopic, mLidarQos);
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
    "State: 'inactive [2]'. Use lifecycle commands to "
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
  mScanPub->on_activate();

  // Start lidar thread
  startLidarThread();

  RCLCPP_INFO(
    get_logger(),
    "State: 'active [3]'. Use lifecycle commands to "
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
  mScanPub->on_deactivate();

  // Stop Lidar THread
  stopLidarThread();

  RCLCPP_INFO(
    get_logger(),
    "State: 'inactive [2]'. Use lifecycle commands to "
    "activate [3], cleanup [2] or shutdown "
    "[6]");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LdLidarComponent::on_cleanup(const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(), "on_cleanup: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                 << "] -> Unconfigured");

  mScanPub.reset();

  RCLCPP_INFO(
    get_logger(),
    "State: 'unconfigured [1]'. Use lifecycle commands "
    "to configure [1] or shutdown [5]");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LdLidarComponent::on_shutdown(const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(), "on_shutdown: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                  << "] -> Finalized");

  mScanPub.reset();

  RCLCPP_INFO_STREAM(get_logger(), "State: 'finalized [4]'. Press Ctrl+C to kill...");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LdLidarComponent::on_error(const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(), "on_error: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                               << "] -> Finalized");

  RCLCPP_INFO_STREAM(get_logger(), "State: 'finalized [4]'. Press Ctrl+C to kill...");
  return nav2_util::CallbackReturn::FAILURE;
}

void LdLidarComponent::publishLaserScan()
{
  static size_t count = 0;

  // Print the current state for demo purposes
  if (!mScanPub->is_activated()) {
    auto & clk = *this->get_clock();
    RCLCPP_INFO_THROTTLE(
      get_logger(), clk, 5000,
      "Lifecycle publisher is currently inactive. Messages "
      "are not published.");
    return;
  }

  auto msg = mLidar->GetLaserScan();
  mLidar->ResetFrameReady();
  // We independently from the current state call publish on the lifecycle
  // publisher.
  // Only if the publisher is in an active state, the message transfer is
  // enabled and the message actually published.
  mScanPub->publish(std::move(msg));
}

bool LdLidarComponent::initLidar()
{
  mLidar = std::make_unique<LiPkg>(get_clock());

  if (!initLidarComm()) {
    return false;
  }

  // Set serial reading callback
  mLidarComm->SetReadCallback(std::bind(&LdLidarComponent::lidarReadCallback, this, _1, _2));

  if (mLidarComm->Open(mSerialPort)) {
    RCLCPP_INFO_STREAM(get_logger(), "LDLidar connection successful");
  }

  return true;
}

void LdLidarComponent::lidarReadCallback(const char * byte, size_t len)
{
  if (mLidar->Parse(reinterpret_cast<const uint8_t *>(byte), len)) {
    mLidar->AssemblePacket();
  }
}

bool LdLidarComponent::initLidarComm()
{
  if (!mUseDirectSerial) {
    // USB <-> UART converter
    mLidarComm = std::make_unique<CmdInterfaceLinux>();

    std::vector<std::pair<std::string, std::string>> device_list;

    mLidarComm->GetCmdDevices(device_list);
    for (auto n : device_list) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Device found: " << n.first << "    " << n.second);
      if (strstr(n.second.c_str(), "CP2102")) {
        mSerialPort = n.first;
      }
    }

    if (mSerialPort.empty()) {
      RCLCPP_ERROR(get_logger(), "Can't find CP2102 USB<->UART converter.");
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Found CP2102 USB<->UART converter. Trying to "
      "connect to lidar device...");
  } else {
    // TODO(Myzhar) test direct serial port connection
  }

  return true;
}

void LdLidarComponent::startLidarThread()
{
  mLidarThread = std::thread(&LdLidarComponent::lidarThreadFunc, this);
}

void LdLidarComponent::stopLidarThread()
{
  if (!mThreadStop) {
    mThreadStop = true;
    RCLCPP_DEBUG(get_logger(), "Stopping lidar thread...");
    try {
      if (mLidarThread.joinable()) {
        mLidarThread.join();
      }
    } catch (std::system_error & e) {
      RCLCPP_WARN(get_logger(), "Lidar thread joining exception: %s", e.what());
    }
  }
}

void LdLidarComponent::lidarThreadFunc()
{
  RCLCPP_DEBUG(get_logger(), "Lidar thread started");

  mThreadStop = false;
  rclcpp::Time prev_ts = get_clock()->now();
  mPublishing = false;

  while (1) {
    // ----> Interruption check
    if (!rclcpp::ok()) {
      RCLCPP_DEBUG(get_logger(), "Ctrl+C received: stopping grab thread");
      mThreadStop = true;
    }

    if (mThreadStop) {
      RCLCPP_DEBUG(get_logger(), "Lidar thread stopped");
      break;
    }
    // <---- Interruption check

    int nSub = count_subscribers(mScanTopic);
    if (nSub > 0) {
      mPublishing = true;
      if (mLidar->IsFrameReady()) {
        rclcpp::Time ts = get_clock()->now();
        double dt = (ts - prev_ts).nanoseconds();

        mPubFreq = (1e9 / dt);
        // RCLCPP_DEBUG_STREAM(get_logger(), "Lidar thread freq: " << mPubFreq << " Hz");

        publishLaserScan();
        prev_ts = ts;
      }
    } else {
      mPublishing = false;
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

    stat.addf("Single measure", "%.3f Hz", mLidar->GetSpeed());

    if (mPublishing) {
      stat.addf("Publishing", "%.3f Hz", mPubFreq);
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
