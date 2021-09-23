#include "ldlidar_component.hpp"

namespace ldlidar
{
LdLidarComponent::LdLidarComponent(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("lidar_node", options)
{
  RCLCPP_INFO(get_logger(), "****************************************");
  RCLCPP_INFO(get_logger(), " LDRobot DToF Lidar Lifecycle Component ");
  RCLCPP_INFO(get_logger(), "****************************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "****************************************");

  // ----> Parameters initialization
  getParam("general.debug_mode", mDebugMode, mDebugMode);
  if (mDebugMode)
  {
    rcutils_ret_t res = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (res != RCUTILS_RET_OK)
    {
      RCLCPP_INFO(get_logger(), "Error setting DEBUG level fot logger");
    }
    else
    {
      RCLCPP_INFO(get_logger(), "*** Debug Mode enabled ***");
    }
  }
  else
  {
    rcutils_ret_t res = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

    if (res != RCUTILS_RET_OK)
    {
      RCLCPP_INFO(get_logger(), "Error setting INFO level for logger");
    }
  }

  RCLCPP_DEBUG(get_logger(), "[ROS2] Using RMW_IMPLEMENTATION = %s", rmw_get_implementation_identifier());
  // <---- Parameters initialization
}

LdLidarComponent::~LdLidarComponent()
{
}

template <typename T>
void LdLidarComponent::getParam(std::string paramName, T defValue, T& outVal, std::string log_info)
{
  declare_parameter(paramName, rclcpp::ParameterValue(defValue));

  if (!get_parameter(paramName, outVal))
  {
    RCLCPP_WARN_STREAM(get_logger(), "The parameter '"
                                         << paramName << "' is not available or is not valid, using the default value: "
                                         << defValue);
  }

  if (!log_info.empty())
  {
    RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
  }
}

LNI::CallbackReturn LdLidarComponent::on_configure(const lc::State& prev_state)
{
  RCLCPP_DEBUG_STREAM(get_logger(),
                      "on_configure: " << prev_state.label() << " [" << static_cast<int>(prev_state.id()) << "] -> Inactive");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LdLidarComponent::on_activate(const lc::State& prev_state)
{
  RCLCPP_DEBUG_STREAM(get_logger(),
                      "on_activate: " << prev_state.label() << " [" << static_cast<int>(prev_state.id()) << "] -> Active");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LdLidarComponent::on_deactivate(const lc::State& prev_state)
{
  RCLCPP_DEBUG_STREAM(get_logger(),
                      "on_deactivate: " << prev_state.label() << " [" << static_cast<int>(prev_state.id()) << "] -> Inactive");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LdLidarComponent::on_cleanup(const lc::State& prev_state)
{
  RCLCPP_DEBUG_STREAM(get_logger(),
                      "on_cleanup: " << prev_state.label() << " [" << static_cast<int>(prev_state.id()) << "] -> Unconfigured");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LdLidarComponent::on_shutdown(const lc::State& prev_state)
{
  RCLCPP_DEBUG_STREAM(get_logger(),
                      "on_shutdown: " << prev_state.label() << " [" << static_cast<int>(prev_state.id()) << "] -> Finalized");
  RCLCPP_INFO_STREAM("Node is entered in the 'Finalized' state. Press Ctrl+C to kill...")
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LdLidarComponent::on_error(const lc::State& prev_state)
{
  RCLCPP_DEBUG_STREAM(get_logger(),
                      "on_error: " << prev_state.label() << " [" << static_cast<int>(prev_state.id()) << "] -> Finalized");
  return LNI::CallbackReturn::SUCCESS;
}

}  // namespace ldlidar

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(ldlidar::LdLidarComponent)