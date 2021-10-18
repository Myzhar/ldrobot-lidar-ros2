#include "ldlidar_tools.hpp"

namespace tools
{
std::string qos2str(rmw_qos_history_policy_t qos)
{
  switch (qos)
  {
    case RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT:
      return "RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT";

    case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
      return "RMW_QOS_POLICY_HISTORY_KEEP_LAST";

    case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
      return "RMW_QOS_POLICY_HISTORY_KEEP_ALL";
  }

  return "Unknown QoS value";
}

std::string qos2str(rmw_qos_reliability_policy_t qos)
{
  switch (qos)
  {
    case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:
      return "RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT";

    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      return "RMW_QOS_POLICY_RELIABILITY_RELIABLE";

    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      return "RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT";
  }

  return "Unknown QoS value";
}

std::string qos2str(rmw_qos_durability_policy_t qos)
{
  switch (qos)
  {
    case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
      return "RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT";

    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      return "RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL";

    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      return "RMW_QOS_POLICY_DURABILITY_VOLATILE";
  }

  return "Unknown QoS value";
}

std::string to_string(ldlidar::UNITS val)
{
  switch (val)
  {
    case ldlidar::UNITS::MILLIMETERS:
      return "MILLIMETERS";

    case ldlidar::UNITS::CENTIMETERS:
      return "CENTIMETERS";

    case ldlidar::UNITS::METERS:
      return "METERS";
  }

  return "Unknown UNITS value";
}

std::string to_string(ldlidar::ROTATION val)
{
  switch (val)
  {
    case ldlidar::ROTATION::CLOCKWISE:
      return "CLOCKWISE";

    case ldlidar::ROTATION::COUNTERCLOCKWISE:
      return "COUNTERCLOCKWISE";
  }

  return "Unknown ROTATION value";
}
}  // namespace tools