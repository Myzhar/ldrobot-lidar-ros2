#ifndef LDLIDAR_COMPONENT_HPP
#define LDLIDAR_COMPONENT_HPP

#include <rcutils/logging_macros.h>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
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
  explicit LdLidarComponent(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

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

protected:
  template <typename T>
  void getParam(std::string paramName, T defValue, T &outVal, std::string log_info = std::string());

  void readParameters();

private:
  // ----> Parameters
  bool mDebugMode = true;
  // <---- Parameters
};

}  // namespace ldlidar

#endif
