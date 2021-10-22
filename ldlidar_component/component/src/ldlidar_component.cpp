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
LdLidarComponent::LdLidarComponent(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("lidar_node", options), mLidarQos(1), mDiagUpdater(this)
{
  RCLCPP_INFO(get_logger(), "****************************************");
  RCLCPP_INFO(get_logger(), " LDRobot DToF Lidar Lifecycle Component ");
  RCLCPP_INFO(get_logger(), "****************************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "****************************************");
  RCLCPP_INFO(get_logger(),
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
  getParam("general.debug_mode", mDebugMode, mDebugMode);
  if (mDebugMode)
  {
    rcutils_ret_t res = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (res != RCUTILS_RET_OK)
    {
      RCLCPP_INFO(get_logger(), "Error setting DEBUG level for logger");
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
  RCLCPP_INFO(get_logger(), "+++ COMMUNICATION PARAMETERS +++");

  // ----> Communication
  getParam("comm.direct_serial", mUseDirectSerial, mUseDirectSerial);
  RCLCPP_INFO_STREAM(get_logger(), " * Direct Serial comm: " << (mUseDirectSerial ? "TRUE" : "FALSE"));

  if (mUseDirectSerial)
  {
    getParam("comm.serial_port", mUseDirectSerial, mUseDirectSerial, "* Serial port: ");
  }
  // <---- Communication
}

void LdLidarComponent::getLidarParams()
{
  RCLCPP_INFO(get_logger(), "+++ LIDAR PARAMETERS +++");

  // ----> Lidar config
  getParam("lidar.frame_id", mFrameId, mFrameId, " * Lidar frame: ");
  int rotVerse = static_cast<int>(mRotVerse);
  getParam("lidar.rot_verse", rotVerse, rotVerse);
  if (rotVerse < 0 || rotVerse > static_cast<int>(ROTATION::COUNTERCLOCKWISE))
  {
    RCLCPP_WARN_STREAM(get_logger(), "Rotation verse not valid (" << rotVerse << "). Using default value");
    mRotVerse = ROTATION::COUNTERCLOCKWISE;
  }
  else
  {
    mRotVerse = static_cast<ROTATION>(rotVerse);
  }
  RCLCPP_INFO_STREAM(get_logger(), " * Rotation verse: " << tools::to_string(mRotVerse));

  int units = static_cast<int>(mUnits);
  getParam("lidar.units", units, units);
  if (units < 0 || units > static_cast<int>(UNITS::METERS))
  {
    RCLCPP_WARN_STREAM(get_logger(), "Units value not valid (" << units << "). Using default value");
    mUnits = UNITS::METERS;
  }
  else
  {
    mUnits = static_cast<UNITS>(units);
  }
  RCLCPP_INFO_STREAM(get_logger(), " * Measure units: " << tools::to_string(mUnits));

  int qos_history = static_cast<int>(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  int qos_depth = 1;
  int qos_reliability = static_cast<int>(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  int qos_durability = static_cast<int>(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  getParam("lidar.qos_history", qos_history, qos_history);
  if (qos_history < 0 || qos_history >= static_cast<int>(RMW_QOS_POLICY_HISTORY_UNKNOWN))
  {
    RCLCPP_WARN_STREAM(get_logger(), "QoS History value not valid (" << qos_history << "). Using default value");
    mLidarQos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  }
  else
  {
    mLidarQos.history(static_cast<rmw_qos_history_policy_t>(qos_history));
  }
  RCLCPP_INFO_STREAM(get_logger(),
                     " * QoS History Policy: " << tools::qos2str(static_cast<rmw_qos_history_policy_t>(qos_history)));

  if (static_cast<rmw_qos_history_policy_t>(qos_history) == RMW_QOS_POLICY_HISTORY_KEEP_LAST)
  {
    getParam("lidar.qos_depth", qos_depth, qos_depth, " * QoS History Depth: ");
    mLidarQos.keep_last(qos_depth);
  }

  getParam("lidar.qos_reliability", qos_reliability, qos_reliability);
  if (qos_reliability < 0 || qos_reliability >= static_cast<int>(RMW_QOS_POLICY_RELIABILITY_UNKNOWN))
  {
    RCLCPP_WARN_STREAM(get_logger(),
                       "QoS Reliability value not valid (" << qos_reliability << "). Using default value");
    mLidarQos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  }
  else
  {
    mLidarQos.reliability(static_cast<rmw_qos_reliability_policy_t>(qos_reliability));
  }
  RCLCPP_INFO_STREAM(get_logger(), " * QoS Reliability: "
                                       << tools::qos2str(static_cast<rmw_qos_reliability_policy_t>(qos_reliability)));

  getParam("lidar.qos_durability", qos_durability, qos_durability);
  if (qos_durability < 0 || qos_durability >= static_cast<int>(RMW_QOS_POLICY_DURABILITY_UNKNOWN))
  {
    RCLCPP_WARN_STREAM(get_logger(), "QoS Durability value not valid (" << qos_durability << "). Using default value");
    mLidarQos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }
  else
  {
    mLidarQos.durability(static_cast<rmw_qos_durability_policy_t>(qos_durability));
  }
  RCLCPP_INFO_STREAM(get_logger(),
                     " * QoS Durability: " << tools::qos2str(static_cast<rmw_qos_durability_policy_t>(qos_durability)));
  // <---- Lidar config
}

template <typename T>
void LdLidarComponent::getParam(std::string paramName, T defValue, T& outVal, std::string log_info)
{
  try
  {
    declare_parameter(paramName, rclcpp::ParameterValue(defValue));
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException& ex)
  {
    RCLCPP_DEBUG_STREAM(get_logger(), "Exception: " << ex.what());
  }

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
    mScanPub = this->create_publisher<sensor_msgs::msg::LaserScan>(mScanTopic, mLidarQos);
  }
  // <---- Initialize publisher

  // ----> Connect to Lidar
  if (!initLidar())
  {
    return LNI::CallbackReturn::ERROR;  // Transition to Finalized state
    // Note: we could use FAILURE instead of ERROR to remain in unconfigured
    // state and try again to connect
  }
  // <---- Connect to Lidar

  RCLCPP_INFO(get_logger(),
              "State: 'inactive [2]'. Use lifecycle commands to "
              "activate [3], cleanup [2] or shutdown "
              "[6]");
  return LNI::CallbackReturn::SUCCESS;
}  // namespace ldlidar

LNI::CallbackReturn LdLidarComponent::on_activate(const lc::State& prev_state)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "on_activate: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                                    << "] -> Active");
  // Activate publisher
  mScanPub->on_activate();

  // Start lidar thread
  startLidarThread();

  RCLCPP_INFO(get_logger(),
              "State: 'active [3]'. Use lifecycle commands to "
              "deactivate [4] or shutdown [7]");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LdLidarComponent::on_deactivate(const lc::State& prev_state)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "on_deactivate: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                                      << "] -> Inactive");
  // Dectivate publisher
  mScanPub->on_deactivate();

  // Stop Lidar THread
  stopLidarThread();

  RCLCPP_INFO(get_logger(),
              "State: 'inactive [2]'. Use lifecycle commands to "
              "activate [3], cleanup [2] or shutdown "
              "[6]");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn LdLidarComponent::on_cleanup(const lc::State& prev_state)
{
  RCLCPP_DEBUG_STREAM(get_logger(), "on_cleanup: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                                   << "] -> Unconfigured");

  mScanPub.reset();

  RCLCPP_INFO(get_logger(),
              "State: 'unconfigured [1]'. Use lifecycle commands "
              "to configure [1] or shutdown [5]");
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

  // Print the current state for demo purposes
  if (!mScanPub->is_activated())
  {
    auto& clk = *this->get_clock();
    RCLCPP_INFO_THROTTLE(get_logger(), clk, 5000,
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

  if (!initLidarComm())
  {
    return false;
  }

  // Set serial reading callback
  mLidarComm->SetReadCallback(std::bind(&LdLidarComponent::lidarReadCallback, this, _1, _2));

  if (mLidarComm->Open(mSerialPort))
  {
    RCLCPP_INFO_STREAM(get_logger(), "LDLidar connection successful");
  }

  return true;
}

void LdLidarComponent::lidarReadCallback(const char* byte, size_t len)
{
  if (mLidar->Parse((uint8_t*)byte, len))
  {
    mLidar->AssemblePacket();
  }
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

    RCLCPP_INFO(get_logger(),
                "Found CP2102 USB<->UART converter. Trying to "
                "connect to lidar device...");
  }
  else
  {
    // TODO test direct serial port connection
  }

  return true;
}

void LdLidarComponent::startLidarThread()
{
  mLidarThread = std::thread(&LdLidarComponent::lidarThreadFunc, this);
}

void LdLidarComponent::stopLidarThread()
{
  if (!mThreadStop)
  {
    mThreadStop = true;
    RCLCPP_DEBUG(get_logger(), "Stopping lidar thread...");
    try
    {
      if (mLidarThread.joinable())
      {
        mLidarThread.join();
      }
    }
    catch (std::system_error& e)
    {
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

  while (1)
  {
    // ----> Interruption check
    if (!rclcpp::ok())
    {
      RCLCPP_DEBUG(get_logger(), "Ctrl+C received: stopping grab thread");
      mThreadStop = true;
    }

    if (mThreadStop)
    {
      RCLCPP_DEBUG(get_logger(), "Lidar thread stopped");
      break;
    }
    // <---- Interruption check

    int nSub = count_subscribers(mScanTopic);
    if (nSub > 0)
    {
      mPublishing = true;
      if (mLidar->IsFrameReady())
      {
        rclcpp::Time ts = get_clock()->now();
        double dt = (ts - prev_ts).nanoseconds();

        mPubFreq = (1e9 / dt);

        publishLaserScan();
        prev_ts = ts;
      }
    }
    else
    {
      mPublishing = false;
    }
  }

  RCLCPP_DEBUG(get_logger(), "Lidar thread finished");
}

void LdLidarComponent::callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  rclcpp_lifecycle::State state = get_current_state();

  if (state.id() == 3)  // ACTIVE
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, std::string("Node state: ") + state.label());

    stat.addf("Single measure", "%.3f Hz", mLidar->GetSpeed());

    if (mPublishing)
    {
      stat.addf("Publishing", "%.3f Hz", mPubFreq);
    }
    else
    {
      stat.add("Publishing", "NO SUBSCRIBERS");
    }
  }
  else if (state.id() == 1 || state.id() == 2)  // UNCONFIGURED || INACTIVE
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, std::string("Node state: ") + state.label());
  }
  else  // SHUTDOWN || ERROR
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::string("Node state: ") + state.label());
  }
}

}  // namespace ldlidar

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(ldlidar::LdLidarComponent)