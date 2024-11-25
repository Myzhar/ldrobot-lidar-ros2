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

#ifndef LDLIDAR_TOOLS_HPP_
#define LDLIDAR_TOOLS_HPP_

#include <rclcpp/rclcpp.hpp>

namespace tools
{
/*! \brief qos value to string
 * \param qos the value to convert
 */
std::string qos2str(rmw_qos_history_policy_t qos);

/*! \brief qos value to string
 * \param qos the value to convert
 */
std::string qos2str(rmw_qos_reliability_policy_t qos);

/*! \brief qos value to string
 * \param qos the value to convert
 */
std::string qos2str(rmw_qos_durability_policy_t qos);

// /*! \brief units value to string
//  * \param units the value to convert
//  */
// std::string to_string(ldlidar::UNITS val);

// /*! \brief rotation value to string
//  * \param rotation the value to convert
//  */
// std::string to_string(ldlidar::ROTATION val);

uint64_t GetSystemTimeStamp(void);
}  // namespace tools


#endif  // LDLIDAR_TOOLS_HPP_
