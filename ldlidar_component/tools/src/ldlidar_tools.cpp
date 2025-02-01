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

#include "ldlidar_tools.hpp"

namespace tools
{
std::string qos2str(rmw_qos_history_policy_t qos)
{
  switch (qos) {
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
  switch (qos) {
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
  switch (qos) {
    case RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT:
      return "RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT";

    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      return "RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL";

    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      return "RMW_QOS_POLICY_DURABILITY_VOLATILE";
  }

  return "Unknown QoS value";
}

// std::string to_string(ldlidar::UNITS val)
// {
//   switch (val) {
//     case ldlidar::UNITS::MILLIMETERS:
//       return "MILLIMETERS";

//     case ldlidar::UNITS::CENTIMETERS:
//       return "CENTIMETERS";

//     case ldlidar::UNITS::METERS:
//       return "METERS";
//   }

//   return "Unknown UNITS value";
// }

// std::string to_string(ldlidar::ROTATION val)
// {
//   switch (val) {
//     case ldlidar::ROTATION::CLOCKWISE:
//       return "CLOCKWISE";

//     case ldlidar::ROTATION::COUNTERCLOCKWISE:
//       return "COUNTERCLOCKWISE";
//   }

//   return "Unknown ROTATION value";
// }

uint64_t GetSystemTimeStamp(void)
{
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp =
    std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return (uint64_t)tmp.count();
}
}  // namespace tools
