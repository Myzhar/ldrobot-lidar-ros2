/**
 * @file ldlidar_component.hpp
 * @brief Header file for the LdLidarComponent class.
 *
 * This file contains the definition of the LdLidarComponent class, which is a ROS2 lifecycle node
 * for interfacing with LDLidar devices. It includes methods for configuring, activating, deactivating,
 * cleaning up, shutting down, and handling errors in the lifecycle of the node. Additionally, it provides
 * functionality for parameter handling, diagnostic updates, and lidar data processing and publishing.
 *
 * @copyright Copyright 2024 Walter Lucetti
 * @license Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 *
 * @see http://www.apache.org/licenses/LICENSE-2.0
 */

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

/**
 * @class LdLidarComponent
 * @brief A ROS2 lifecycle node for interfacing with LDLidar devices.
 *
 * This class provides methods for managing the lifecycle of the node, including configuration,
 * activation, deactivation, cleanup, shutdown, and error handling. It also includes functionality
 * for parameter handling, diagnostic updates, and lidar data processing and publishing.
 */
class LdLidarComponent : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for LdLidarComponent.
   * @param options Node options for configuring the node.
   */
  LDLIDAR_COMPONENTS_EXPORT
  explicit LdLidarComponent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for LdLidarComponent.
   */
  virtual ~LdLidarComponent();

  /**
   * @brief Callback for configuring the node.
   * @param prev_state The previous state of the node.
   * @return The result of the configuration.
   */
  nav2_util::CallbackReturn on_configure(const lc::State & prev_state) override;

  /**
   * @brief Callback for activating the node.
   * @param prev_state The previous state of the node.
   * @return The result of the activation.
   */
  nav2_util::CallbackReturn on_activate(const lc::State & prev_state) override;

  /**
   * @brief Callback for deactivating the node.
   * @param prev_state The previous state of the node.
   * @return The result of the deactivation.
   */
  nav2_util::CallbackReturn on_deactivate(const lc::State & prev_state) override;

  /**
   * @brief Callback for cleaning up the node.
   * @param prev_state The previous state of the node.
   * @return The result of the cleanup.
   */
  nav2_util::CallbackReturn on_cleanup(const lc::State & prev_state) override;

  /**
   * @brief Callback for shutting down the node.
   * @param prev_state The previous state of the node.
   * @return The result of the shutdown.
   */
  nav2_util::CallbackReturn on_shutdown(const lc::State & prev_state) override;

  /**
   * @brief Callback for handling errors in the node.
   * @param prev_state The previous state of the node.
   * @return The result of the error handling.
   */
  nav2_util::CallbackReturn on_error(const lc::State & prev_state) override;

  /**
   * @brief Callback for updating diagnostic information.
   * @param stat The diagnostic status wrapper.
   */
  void callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);

protected:
  /**
   * @brief Template function for getting a parameter.
   * @tparam T The type of the parameter.
   * @param paramName The name of the parameter.
   * @param defValue The default value of the parameter.
   * @param outVal The output value of the parameter.
   * @param log_info Optional log information.
   * @param dynamic Whether the parameter is dynamic.
   * @param description The description of the parameter.
   * @param min The minimum value of the parameter.
   * @param max The maximum value of the parameter.
   */
  template<typename T>
  void getParam(
    std::string paramName, T defValue, T & outVal,
    std::string log_info = std::string(),
    bool dynamic = false,
    std::string description = std::string(),
    T min = std::numeric_limits<T>::min(), T max = std::numeric_limits<T>::max());

  /**
   * @brief Get all parameters for the node.
   */
  void getParameters();

  /**
   * @brief Get debug parameters for the node.
   */
  void getDebugParams();

  /**
   * @brief Get communication parameters for the node.
   */
  void getCommParams();

  /**
   * @brief Get lidar parameters for the node.
   */
  void getLidarParams();

  /**
   * @brief Initialize the lidar device.
   * @return True if successful, false otherwise.
   */
  bool initLidar();

  /**
   * @brief Initialize the lidar communication.
   * @return True if successful, false otherwise.
   */
  bool initLidarComm();

  /**
   * @brief Start the lidar thread.
   */
  void startLidarThread();

  /**
   * @brief Stop the lidar thread.
   */
  void stopLidarThread();

  /**
   * @brief Function for the lidar thread.
   */
  void lidarThreadFunc();

  /**
   * @brief Publish laser scan data.
   * @param src The source points.
   * @param lidar_spin_freq The lidar spin frequency.
   */
  void publishLaserScan(ldlidar::Points2D & src, double lidar_spin_freq);

private:
  // ----> Parameters
  bool _debugMode = true; ///< Debug mode flag.
  std::string _lidarModel; ///< Lidar model.
  std::string _serialPort; ///< Serial port name.
  int _baudrate = 230400; ///< Serial baudrate.
  int _readTimeOut_msec = 1000; ///< Serial read timeout in milliseconds.
  bool _counterclockwise = true; ///< Rotation direction.
  bool _enableAngleCrop = false; ///< Enable angle cropping.
  double _angleCropMin = 90.0; ///< Angle cropping minimum value.
  double _angleCropMax = 270.0; ///< Angle cropping maximum value.
  int _bins = 455; ///< Fixed number of bins.
  double _rangeMin = 0.03; ///< Minimum range.
  double _rangeMax = 15.0; ///< Maximum range.
  std::string _frameId = "ldlidar_link"; ///< Frame ID.
  float _distScale = 0.001; ///< Scale factor for distance.
  // <---- Parameters

  // ----> Publishers
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>> _scanPub; ///< Laser scan publisher.
  // <---- Publishers

  // ----> Topics
  std::string _topicRoot = "~/"; ///< Topic root.
  std::string _scanTopic = "scan"; ///< Scan topic.
  // <---- Topics

  // ----> Diagnostic
  diagnostic_updater::Updater _diagUpdater; ///< Diagnostic updater.
  // <---- Diagnostic

  // ----> Lidar
  std::unique_ptr<ldlidar::LDLidarDriver> _lidar; ///< Lidar driver.
  ldlidar::LDType _lidarType; ///< Lidar type.
  // <---- Lidar

  // ----> Threads
  std::thread _lidarThread; ///< Lidar thread.
  bool _threadStop = false; ///< Thread stop flag.
  // <---- Threads

  // ----> Diagnostic
  double _pubFreq; ///< Publishing frequency.
  bool _publishing; ///< Publishing flag.
  // <---- Diagnostic
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
