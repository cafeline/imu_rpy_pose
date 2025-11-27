// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef LIVOX_IMU_TEST__IMU_PROCESSOR_HPP_
#define LIVOX_IMU_TEST__IMU_PROCESSOR_HPP_

#include <Eigen/Geometry>
#include <rclcpp/time.hpp>
#include <vector>

namespace livox_imu_test
{

struct ImuProcessorParams
{
  int bias_sample_count {200};

  bool clip_enable {true};
  double clip_max_rad_per_s {5.0};

  bool lpf_enable {true};
  double lpf_alpha {0.2};

  bool dt_check_enable {true};
  double dt_min {0.0005};
  double dt_max {0.05};
};

class ImuProcessor
{
public:
  explicit ImuProcessor(const ImuProcessorParams & params = ImuProcessorParams());

  void reset();
  void set_params(const ImuProcessorParams & params);

  // IMUデータを処理し、積分を実施した場合にtrueを返す
  bool process(
    const rclcpp::Time & stamp,
    const Eigen::Vector3d & angular_velocity,
    const Eigen::Vector3d & linear_acceleration);

  const Eigen::Quaterniond & orientation() const {return orientation_;}
  const Eigen::Vector3d & position() const {return position_;}
  const Eigen::Vector3d & bias() const {return bias_;}
  bool bias_ready() const {return bias_ready_;}

private:
  ImuProcessorParams params_;

  bool have_stamp_ {false};
  rclcpp::Time last_stamp_;

  bool bias_ready_ {false};
  Eigen::Vector3d bias_ {Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> bias_samples_;

  Eigen::Vector3d lpf_prev_ {Eigen::Vector3d::Zero()};
  bool lpf_initialized_ {false};

  Eigen::Quaterniond orientation_ {Eigen::Quaterniond::Identity()};
  Eigen::Vector3d position_ {Eigen::Vector3d::Zero()};
};

}  // namespace livox_imu_test

#endif  // LIVOX_IMU_TEST__IMU_PROCESSOR_HPP_
