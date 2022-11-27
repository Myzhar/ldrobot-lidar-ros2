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

#ifndef LIPKG_HPP_
#define LIPKG_HPP_

#include <stdint.h>

#include <vector>
#include <array>
#include <utility>
#include <memory>
#include <string>
#include <iostream>

#include <rclcpp/clock.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


#define ANGLE_TO_RADIAN(angle) ((angle) * M_PI / 180.)

namespace ldlidar
{
enum
{
  PKG_HEADER = 0x54,
  PKG_VER_LEN = 0x2C,
  POINT_PER_PACK = 12,
};

typedef struct __attribute__((packed))
{
  uint16_t distance;
  uint8_t confidence;
} LidarPointStructDef;

typedef struct __attribute__((packed))
{
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

struct PointData
{
  float angle;
  uint16_t distance;
  uint8_t confidence;
  double x;
  double y;
  PointData(float angle, uint16_t distance, uint8_t confidence, double x = 0, double y = 0)
  {
    this->angle = angle;
    this->distance = distance;
    this->confidence = confidence;
    this->x = x;
    this->y = y;
  }
  PointData()
  {
  }
  friend std::ostream & operator<<(std::ostream & os, const PointData & data)
  {
    os << data.angle << " " << data.distance << " " << static_cast<int>(data.confidence) << " "
       << data.x << " " << data.y;
    return os;
  }
};

/**
  \enum UNITS
  \brief Lists available unit for measures.
   */
enum class UNITS
{
  MILLIMETERS, /**< International System, 1/1000 m. */
  CENTIMETERS, /**< International System, 1/100 m. */
  METERS       /**< International System, 1 m [ROS standard]*/
};

/**
  \enum ROTATION
  \brief Lists available rotation verses
   */
enum class ROTATION
{
  CLOCKWISE,       /**< Right thumb looking down  */
  COUNTERCLOCKWISE /**< Right thumb looking up [ROS standard]*/
};

class LiPkg
{
public:
  LiPkg(
    rclcpp::Clock::SharedPtr clock, UNITS unit = UNITS::METERS,
    ROTATION rotVerse = ROTATION::COUNTERCLOCKWISE,
    std::string lidarFrame = "ldlidar_link");
  double GetSpeed(void); /*Lidar spin speed (Hz)*/
  uint16_t GetTimestamp(void)
  {
    return mTimestamp;
  } /*time stamp of the packet */
  bool IsPkgReady(void)
  {
    return mIsPkgReady;
  } /*a packet is ready */
  bool IsFrameReady(void)
  {
    return mFrameReady;
  } /*Lidar data frame is ready*/
  void ResetFrameReady(void)
  {
    mFrameReady = false;
  }
  int32_t GetErrorTimes(void)
  {
    return mErrorTimes;
  } /*the number of errors in parser process of lidar data frame*/
  const std::array<PointData, POINT_PER_PACK> & GetPkgData(void); /*original data package*/
  bool Parse(const uint8_t * data, int32_t len);                     /*parse single packet*/
  bool AssemblePacket(); /*combine stantard data into data frames and calibrate*/
  std::unique_ptr<sensor_msgs::msg::LaserScan> GetLaserScan()
  {
    return std::move(mOutScan);
  }

protected:
  void ToLaserscan(std::vector<PointData> src);

private:
  uint16_t mTimestamp;
  double mSpeed;
  std::vector<uint8_t> mDataTmp;
  int32_t mErrorTimes;
  std::array<PointData, POINT_PER_PACK> mOnePkg;
  std::vector<PointData> mFrameTmp;
  bool mIsPkgReady;
  bool mFrameReady;
  std::unique_ptr<sensor_msgs::msg::LaserScan> mOutScan;
  rclcpp::Clock::SharedPtr mClock;

  // ----> Parameters
  double mUnitScale = 1e-3;
  ROTATION mRotVerse = ROTATION::COUNTERCLOCKWISE;
  std::string mLidarFrame = "ldlidar_link";
  // <---- Parameters
};

}  // namespace ldlidar

#endif  // LIPKG_HPP_
