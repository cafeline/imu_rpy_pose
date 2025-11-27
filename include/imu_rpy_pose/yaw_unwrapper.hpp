// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef IMU_RPY_POSE__YAW_UNWRAPPER_HPP_
#define IMU_RPY_POSE__YAW_UNWRAPPER_HPP_

#include <cmath>

namespace imu_rpy_pose
{

// yaw角を連続的に扱うためのシンプルなアンラッパ
class YawUnwrapper
{
public:
  double unwrap(double yaw_rad)
  {
    if (!has_prev_) {
      has_prev_ = true;
      prev_unwrapped_ = yaw_rad;
      return prev_unwrapped_;
    }
    double delta = yaw_rad - prev_unwrapped_;
    // 差分を[-pi, pi]に正規化して連続なyawを得る
    delta = std::atan2(std::sin(delta), std::cos(delta));
    prev_unwrapped_ += delta;
    return prev_unwrapped_;
  }

  void reset()
  {
    has_prev_ = false;
    prev_unwrapped_ = 0.0;
  }

  bool initialized() const {return has_prev_;}
  double value() const {return prev_unwrapped_;}

private:
  bool has_prev_{false};
  double prev_unwrapped_{0.0};
};

}  // namespace imu_rpy_pose

#endif  // IMU_RPY_POSE__YAW_UNWRAPPER_HPP_
