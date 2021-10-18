#ifndef LDLIDAR_TOOLS_HPP
#define LDLIDAR_TOOLS_HPP

#include "lipkg.hpp"

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

/*! \brief units value to string
 * \param units the value to convert
 */
std::string to_string(ldlidar::UNITS val);

/*! \brief rotation value to string
 * \param rotation the value to convert
 */
std::string to_string(ldlidar::ROTATION val);
}  // namespace tools

#endif  // #define LDLIDAR_TOOLS_HPP
