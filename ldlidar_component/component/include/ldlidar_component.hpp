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

#ifndef LDLIDAR_COMPONENT_HPP_
#define LDLIDAR_COMPONENT_HPP_

#include <rcutils/logging_macros.h>

#include <string>
#include <memory>

#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <nav2_util/lifecycle_node.hpp>

#include "visibility_control.hpp"
#include "defines.hpp"
#include "ldlidar_driver.h"

namespace lc = rclcpp_lifecycle;

namespace ldlidar
{
class LdLidarComponent : public nav2_util::LifecycleNode
{
public:
  /// \brief Default constructor
  LDLIDAR_COMPONENTS_EXPORT
  explicit LdLidarComponent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~LdLidarComponent();

  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  nav2_util::CallbackReturn on_configure(const lc::State & prev_state) override;

  /// \brief Callback from transition to "activating" state.
  /// \param[in] state The current state that the node is in.
  nav2_util::CallbackReturn on_activate(const lc::State & prev_state) override;

  /// \brief Callback from transition to "deactivating" state.
  /// \param[in] state The current state that the node is in.
  nav2_util::CallbackReturn on_deactivate(const lc::State & prev_state) override;

  /// \brief Callback from transition to "unconfigured" state.
  /// \param[in] state The current state that the node is in.
  nav2_util::CallbackReturn on_cleanup(const lc::State & prev_state) override;

  /// \brief Callback from transition to "shutdown" state.
  /// \param[in] state The current state that the node is in.
  nav2_util::CallbackReturn on_shutdown(const lc::State & prev_state) override;

  /// \brief Callback from transition to "error" state.
  /// \param[in] state The current state that the node is in.
  nav2_util::CallbackReturn on_error(const lc::State & prev_state) override;

  /// \brief Callback for diagnostic updater
  /// \param[in] stat The current diagnostic status
  void callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);

protected:
  // ----> Node Parameters
  template<typename T>
  void getParam(
    std::string paramName, T defValue, T & outVal,
    std::string log_info = std::string(),
    bool dynamic = false,
    std::string description = std::string(),
    T min = std::numeric_limits<T>::min(), T max = std::numeric_limits<T>::max());

  void getParameters();
  void getDebugParams();
  void getCommParams();
  void getLidarParams();
  // <---- Node Parameters

  // ----> Log functions
  void info(const std::string & msg);
  void debug(const std::string & msg);
  void warning(const std::string & msg);
  void error(const std::string & msg);
  // <---- Log functions

  bool initLidar();
  bool initLidarComm();
  void startLidarThread();
  void stopLidarThread();
  void lidarThreadFunc();

  void publishLaserScan(ldlidar::Points2D & src, double lidar_spin_freq);

private:
  // ----> Parameters
  bool _debugMode = true;
  std::string _lidarModel;        // Lidar Model: LDLiDAR_LD06, LDLiDAR_LD19, LDLiDAR_STL27L
  std::string _serialPort;        // Serial port name
  int _baudrate = 230400;         // Serial baudrate
  int _readTimeOut_msec = 1000;   // Serial read timeout in msec
  bool _counterclockwise = true;  // Rotation verse
  bool _enableAngleCrop = true;   // Enable angle cropping
  double _angleCropMin = 90.0;    // Angle cropping minimum value
  double _angleCropMax = 270.0;   // Angle cropping maximum value
  int _bins = 455;                // Fixed number of bins
  double _rangeMin = 0.03;        // Minimum range
  double _rangeMax = 15.0;        // Maximum range
  std::string _frameId = "ldlidar_link";
  // <---- Parameters

  // Processing
  float _distScale = 0.001; // Scale factor to match the units setting

  // Publisher
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>> _scanPub;

  // ----> Topics
  std::string _topicRoot = "~/";
  std::string _scanTopic = "scan";
  // <---- Topics

  // Diagnostic updater
  diagnostic_updater::Updater _diagUpdater;

  // Lidar communication
  std::unique_ptr<ldlidar::LDLidarDriver> _lidar;
  ldlidar::LDType _lidarType;

  // Lidar Thread
  std::thread _lidarThread;
  bool _threadStop = false;

  // Diagnostic
  double _pubFreq;
  bool _publishing;
};

// ----> Template Function definitions
template<typename T>
void LdLidarComponent::getParam(
  std::string paramName, T defValue, T & outVal,
  std::string description, bool dynamic,
  std::string log_info,
  T minVal, T maxVal)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = !dynamic;

  std::stringstream ss;
  ss << description << " - Default value: ";
  if constexpr (std::is_same<T, bool>::value) {
    ss << (defValue ? "TRUE" : "FALSE");
  } else {
    ss << defValue;
  }
  descriptor.description = ss.str();

  if constexpr (std::is_same<T, double>::value) {
    descriptor.additional_constraints = "Range: [" + std::to_string(minVal) + ", " + std::to_string(
      maxVal) + "]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = minVal;
    range.to_value = maxVal;
    descriptor.floating_point_range.push_back(range);
  } else if constexpr (std::is_same<T, int>::value) {
    descriptor.additional_constraints = "Range: [" + std::to_string(minVal) + ", " + std::to_string(
      maxVal) + "]";
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = minVal;
    range.to_value = maxVal;
    descriptor.integer_range.push_back(range);
  }


  declare_parameter(paramName, rclcpp::ParameterValue(defValue), descriptor);

  if (!get_parameter(paramName, outVal)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The parameter '"
        << paramName
        << "' is not available or is not valid, using the default value: "
        << defValue);
  }

  if (!log_info.empty()) {
    if constexpr (std::is_same<T, bool>::value) {
      RCLCPP_INFO_STREAM(get_logger(), log_info << (outVal ? "TRUE" : "FALSE"));
    } else {
      RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
    }
  }
}
// <---- Template Function definitions

}  // namespace ldlidar

#endif  // LDLIDAR_COMPONENT_HPP_
