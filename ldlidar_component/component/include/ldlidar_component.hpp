#ifndef LDLIDAR_COMPONENT_HPP
#define LDLIDAR_COMPONENT_HPP

#include "cmd_interface_linux.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lipkg.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "visibility_control.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace ldlidar
{
class LdLidarComponent : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// \brief Default constructor
  LDLIDAR_COMPONENTS_EXPORT
  explicit LdLidarComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  virtual ~LdLidarComponent();

  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_configure(const lc::State& prev_state) override;

  /// \brief Callback from transition to "activating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_activate(const lc::State& prev_state) override;

  /// \brief Callback from transition to "deactivating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_deactivate(const lc::State& prev_state) override;

  /// \brief Callback from transition to "unconfigured" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_cleanup(const lc::State& prev_state) override;

  /// \brief Callback from transition to "shutdown" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_shutdown(const lc::State& prev_state) override;

  /// \brief Callback from transition to "error" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_error(const lc::State& prev_state) override;

  /// \brief Callback for diagnostic updater
  /// \param[in] stat The current diagnostic status
  void callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

protected:
  // ----> Node Parameters
  template <typename T>
  void getParam(std::string paramName, T defValue, T& outVal, std::string log_info = std::string());

  void getParameters();
  void getDebugParams();
  void getCommParams();
  void getLidarParams();
  // <---- Node Parameters

  // ----> Log functions
  void info(const std::string& msg);
  void debug(const std::string& msg);
  void warning(const std::string& msg);
  void error(const std::string& msg);
  // <---- Log functions

  bool initLidar();
  bool initLidarComm();
  void startLidarThread();
  void stopLidarThread();
  void lidarThreadFunc();
  void lidarReadCallback(const char* byte, size_t len);

  void publishLaserScan();

private:
  // ----> Parameters
  bool mDebugMode = true;
  bool mUseDirectSerial = false;  // Set to true if using a direct uart connection
  std::string mSerialPort;        // Serial port to use when @ref mUseDirectSerial is true

  UNITS mUnits = UNITS::METERS;
  ROTATION mRotVerse = ROTATION::CLOCKWISE;
  std::string mFrameId = "ldlidar_link";
  // <---- Parameters

  // Publisher
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>> mScanPub;

  // ----> Topics
  std::string mTopicRoot = "~/";
  std::string mScanTopic = "scan";
  // <---- Topics

  // ----> QoS
  rclcpp::QoS mLidarQos;
  // <---- QoS

  // Diagnostic updater
  diagnostic_updater::Updater mDiagUpdater;

  // Lidar
  std::unique_ptr<LiPkg> mLidar;

  // Lidar communication
  std::unique_ptr<CmdInterfaceLinux> mLidarComm;

  // Lidar Thread
  std::thread mLidarThread;
  bool mThreadStop = false;

  // Diagnostic
  double mPubFreq;
  bool mPublishing;
};

}  // namespace ldlidar

#endif
