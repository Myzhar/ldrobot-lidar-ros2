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
#include "cmd_interface_linux.hpp"
#include "lipkg.hpp"

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
    const std::string & description = "",
    bool read_only = true,
    std::string log_info = std::string());

  void getParam(
    std::string paramName, int defValue, int & outVal,
    const nav2_util::LifecycleNode::integer_range & range,
    const std::string & description = "",
    bool read_only = true,
    std::string log_info = std::string());

  void getParam(
    std::string paramName, float defValue, float & outVal,
    const nav2_util::LifecycleNode::floating_point_range & range,
    const std::string & description = "",
    bool read_only = true,
    std::string log_info = std::string());

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
  void lidarReadCallback(const char * byte, size_t len);

  void publishLaserScan();

private:
  // ----> Parameters
  bool _debugMode = true;
  bool _useDirectSerial = false;  // Set to true if using a direct uart connection
  std::string _serialPort;        // Serial port to use when @ref _useDirectSerial is true

  UNITS _units = UNITS::METERS;
  ROTATION _rotVerse = ROTATION::CLOCKWISE;
  std::string _frameId = "ldlidar_link";
  // <---- Parameters

  // Publisher
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>> _scanPub;

  // ----> Topics
  std::string _topicRoot = "~/";
  std::string _scanTopic = "scan";
  // <---- Topics

  // Diagnostic updater
  diagnostic_updater::Updater _diagUpdater;

  // Lidar
  std::unique_ptr<LiPkg> _lidar;

  // Lidar communication
  std::unique_ptr<CmdInterfaceLinux> _lidarComm;

  // Lidar Thread
  std::thread _lidarThread;
  bool _threadStop = false;

  // Diagnostic
  double _pubFreq;
  bool _publishing;
};

}  // namespace ldlidar

#endif  // LDLIDAR_COMPONENT_HPP_
