//  Copyright 2022 Walter Lucetti
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

#ifndef TOFBF_HPP_
#define TOFBF_HPP_

#include <stdint.h>

#include <vector>

#include "lipkg.hpp"

namespace ldlidar
{
class Tofbf
{
private:
  const int CONFIDENCE_LOW = 15;  // Low confidence threshold
  const int CONFIDENCE_SINGLE = 220;  // Discrete points require higher confidence
  const int SCAN_FRE = 4500;  // Default scan frequency, to change, read according to radar protocol
  double offset_x, offset_y;
  double curr_speed;
  Tofbf() = delete;
  Tofbf(const Tofbf &) = delete;
  Tofbf & operator=(const Tofbf &) = delete;

public:
  explicit Tofbf(int speed);
  std::vector<PointData> NearFilter(const std::vector<PointData> & tmp) const;
  ~Tofbf();
};

}  // namespace ldlidar

#endif  // TOFBF_HPP_
