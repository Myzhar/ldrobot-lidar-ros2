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
  RCLCPP_INFO(get_logger(), "State: 'unconfigured [1]'. Use lifecycle commands to configure [1] or shutdown [5]");
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
}

void LdLidarComponent::getDebugParams()
{
  // ----> Debug mode initialization from parameters
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
  // <---- Debug mode initialization from parameters
}

void LdLidarComponent::getCommParams()
{
  // ----> Communication
  getParam("comm.direct_serial", mUseDirectSerial, mUseDirectSerial);
  RCLCPP_INFO_STREAM(get_logger(), " * Direct Serial comm: " << (mUseDirectSerial ? "TRUE" : "FALSE"));

  getParam("comm.serial_port", mUseDirectSerial, mUseDirectSerial);
  // <---- Communication
}

void LdLidarComponent::getLaserParams()
{
}

template <typename T>
void LdLidarComponent::getParam(std::string paramName, T defValue, T& outVal, std::string log_info)
{
  if (!get_parameter(paramName, outVal))
  {
    declare_parameter(paramName, rclcpp::ParameterValue(defValue));

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
  RCLCPP_DEBUG_STREAM(get_logger(), "on_configure: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                                     << "] -> Inactive");

  // Initialize parameters from yaml file
  getParameters();

  // ----> Initialize topics
  mScanTopic = mTopicRoot + std::string("scan");
  // <---- Initialize topics

  // ----> Initialize publisher
  if (!mScanPub)
  {
    rclcpp::QoS qos(1);  // TODO initialize with parameters
    mScanPub = this->create_publisher<sensor_msgs::msg::LaserScan>(mScanTopic, qos);
  }
  // <---- Initialize publisher

  // ----> Connect to Lidar
  if (!initLidar())
  {
    return LNI::CallbackReturn::ERROR;  // Transition to Finalized state
    // Note: we could use FAILURE instead of ERROR to remain in unconfigured state and try again to connect
  }
  // <---- Connect to Lidar

  RCLCPP_INFO(get_logger(), "State: 'inactive [2]'. Use lifecycle commands to activate [3], cleanup [2] or shutdown "
                            "[6]");
  return LNI::CallbackReturn::SUCCESS;
}  // namespace ldlidar

LNI::CallbackReturn LdLidarComponent::on_activate(const lc::State& prev_state)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "on_activate: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                                    << "] -> Active");

  RCLCPP_INFO(get_logger(), "State: 'active [3]'. Use lifecycle commands to deactivate [4] or shutdown [7]");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LdLidarComponent::on_deactivate(const lc::State& prev_state)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "on_deactivate: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                                      << "] -> Inactive");

  RCLCPP_INFO(get_logger(), "State: 'inactive [2]'. Use lifecycle commands to activate [3], cleanup [2] or shutdown "
                            "[6]");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LdLidarComponent::on_cleanup(const lc::State& prev_state)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "on_cleanup: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                                   << "] -> Unconfigured");

  mScanPub.reset();

  RCLCPP_INFO(get_logger(), "State: 'unconfigured [1]'. Use lifecycle commands to configure [1] or shutdown [5]");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LdLidarComponent::on_shutdown(const lc::State& prev_state)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "on_shutdown: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                                    << "] -> Finalized");

  mScanPub.reset();

  RCLCPP_INFO_STREAM(get_logger(), "State: 'finalized [4]'. Press Ctrl+C to kill...");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LdLidarComponent::on_error(const lc::State& prev_state)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "on_error: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                                 << "] -> Finalized");

  RCLCPP_INFO_STREAM(get_logger(), "State: 'finalized [4]'. Press Ctrl+C to kill...");
  return LNI::CallbackReturn::FAILURE;
}

void LdLidarComponent::publishLaserScan()
{
  static size_t count = 0;
  auto msg = std::make_unique<sensor_msgs::msg::LaserScan>();

  // Print the current state for demo purposes
  if (!mScanPub->is_activated())
  {
    auto& clk = *this->get_clock();
    RCLCPP_INFO_THROTTLE(get_logger(), clk, 5000,
                         "Lifecycle publisher is currently inactive. Messages are not published.");
  }

  int nSub = count_subscribers(mScanTopic);
  if (nSub > 0)
  {
    // We independently from the current state call publish on the lifecycle
    // publisher.
    // Only if the publisher is in an active state, the message transfer is
    // enabled and the message actually published.
    mScanPub->publish(std::move(msg));
  }
}

bool LdLidarComponent::initLidar()
{
  mLidar = std::make_unique<LiPkg>();

  if (!initLidarComm())
  {
    return false;
  }

  if (mLidarComm->Open(mSerialPort))
  {
    RCLCPP_INFO_STREAM(get_logger(), "LDLidar connection successful");
  }

  return true;
}

bool LdLidarComponent::initLidarComm()
{
  if (!mUseDirectSerial)
  {
    // USB <-> UART converter
    mLidarComm = std::make_unique<CmdInterfaceLinux>();

    std::vector<std::pair<std::string, std::string>> device_list;

    mLidarComm->GetCmdDevices(device_list);
    for (auto n : device_list)
    {
      RCLCPP_DEBUG_STREAM(get_logger(), "Device found: " << n.first << "    " << n.second);
      if (strstr(n.second.c_str(), "CP2102"))
      {
        mSerialPort = n.first;
      }
    }

    if (mSerialPort.empty())
    {
      RCLCPP_ERROR(get_logger(), "Can't find CP2102 USB<->UART converter.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Found CP2102 USB<->UART converter. Trying to connect to lidar device...");
  }
  else
  {
    // TODO test direct serial port connection
  }

  return true;
}

}  // namespace ldlidar

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(ldlidar::LdLidarComponent)