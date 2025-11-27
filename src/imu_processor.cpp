// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "livox_imu_test/imu_processor.hpp"

#include <algorithm>
#include <cmath>

namespace livox_imu_test
{

ImuProcessor::ImuProcessor(const ImuProcessorParams & params)
: params_(params)
{
}

void ImuProcessor::reset()
{
  have_stamp_ = false;
  bias_ready_ = false;
  bias_ = Eigen::Vector3d::Zero();
  bias_sum_ = Eigen::Vector3d::Zero();
  bias_count_ = 0;
  lpf_initialized_ = false;
  lpf_prev_ = Eigen::Vector3d::Zero();
  orientation_ = Eigen::Quaterniond::Identity();
  position_ = Eigen::Vector3d::Zero();
}

void ImuProcessor::set_params(const ImuProcessorParams & params)
{
  params_ = params;
}

bool ImuProcessor::process(
  const rclcpp::Time & stamp,
  const Eigen::Vector3d & angular_velocity,
  const Eigen::Vector3d & linear_acceleration)
{
  // 初回は時刻を保持し、バイアス有効なら積算のみを行う
  if (!have_stamp_) {
    last_stamp_ = stamp;
    have_stamp_ = true;
    if (params_.bias_enable) {
      bias_sum_ += angular_velocity;
      bias_count_++;
      if (bias_count_ >= params_.bias_sample_count) {
        bias_ = bias_sum_ / static_cast<double>(bias_count_);
        bias_ready_ = true;
      } else {
        bias_ready_ = false;
      }
    } else {
      bias_ready_ = true;
    }
    if (params_.lpf_enable) {
      lpf_prev_ = angular_velocity;
      lpf_initialized_ = true;
    }
    return true;  // 初回からマーカーを出せるようにする
  }

  double dt = static_cast<double>((stamp - last_stamp_).nanoseconds()) / 1e9;

  // バイアス推定（初期Nサンプルのみ、dtチェックとは独立）
  if (params_.bias_enable) {
    bias_sum_ += angular_velocity;
    bias_count_++;
    if (bias_count_ >= params_.bias_sample_count) {
      bias_ = bias_sum_ / static_cast<double>(bias_count_);
      bias_ready_ = true;
    }
  }

  if (params_.dt_check_enable) {
    if (dt < params_.dt_min || dt > params_.dt_max) {
      // 範囲外は積分せず破棄し、基準時刻のみ更新
      last_stamp_ = stamp;
      return false;
    }
  }

  last_stamp_ = stamp;

  Eigen::Vector3d gyro = angular_velocity;
  if (params_.bias_enable) {
    if (bias_ready_) {
      gyro -= bias_;
    } else if (bias_count_ > 0) {
      const Eigen::Vector3d provisional_bias = bias_sum_ / static_cast<double>(bias_count_);
      gyro -= provisional_bias;
    }
  }

  if (params_.clip_enable) {
    gyro[0] = std::clamp(gyro[0], -params_.clip_max_rad_per_s, params_.clip_max_rad_per_s);
    gyro[1] = std::clamp(gyro[1], -params_.clip_max_rad_per_s, params_.clip_max_rad_per_s);
    gyro[2] = std::clamp(gyro[2], -params_.clip_max_rad_per_s, params_.clip_max_rad_per_s);
  }

  if (params_.lpf_enable) {
    if (!lpf_initialized_) {
      lpf_prev_ = gyro;
      lpf_initialized_ = true;
    } else {
      gyro = params_.lpf_alpha * gyro + (1.0 - params_.lpf_alpha) * lpf_prev_;
      lpf_prev_ = gyro;
    }
  }

  const Eigen::Vector3d angle_vec = gyro * dt;
  const double angle = angle_vec.norm();
  if (angle > 1e-9) {
    const Eigen::AngleAxisd delta(angle, angle_vec.normalized());
    orientation_ = orientation_ * Eigen::Quaterniond(delta);
    orientation_.normalize();
  }

  // 位置は常に原点に固定
  position_.setZero();

  return true;
}

}  // namespace livox_imu_test
