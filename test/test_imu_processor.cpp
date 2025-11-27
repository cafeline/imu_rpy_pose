// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "imu_rpy_pose/imu_processor.hpp"
#include "imu_rpy_pose/yaw_unwrapper.hpp"

using imu_rpy_pose::ImuProcessor;
using imu_rpy_pose::ImuProcessorParams;

TEST(ImuProcessor, BiasEstimationStopsIntegrationUntilReady)
{
  ImuProcessorParams params;
  params.bias_sample_count = 3;
  params.dt_check_enable = false;
  params.lpf_enable = false;
  params.clip_enable = false;

  ImuProcessor proc(params);
  Eigen::Vector3d gyro(0.1, 0.0, 0.0);
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();

  EXPECT_TRUE(proc.process(rclcpp::Time(0, 0, RCL_ROS_TIME), gyro, acc));   // 初回もpublish
  EXPECT_FALSE(proc.bias_ready());
  EXPECT_TRUE(proc.process(rclcpp::Time(0, 10000000, RCL_ROS_TIME), gyro, acc)); // bias蓄積
  EXPECT_TRUE(proc.process(rclcpp::Time(0, 20000000, RCL_ROS_TIME), gyro, acc));  // bias確定
  EXPECT_TRUE(proc.bias_ready());
  // バイアス補正が効いているので角度変化は非常に小さい
  EXPECT_NEAR(proc.orientation().angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-6);
}

TEST(ImuProcessor, ClippingLimitsAngularVelocity)
{
  ImuProcessorParams params;
  params.bias_sample_count = 1;
  params.clip_enable = true;
  params.clip_max_rad_per_s = 1.0;
  params.dt_check_enable = false;
  params.lpf_enable = false;
  ImuProcessor proc(params);

  Eigen::Vector3d gyro0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro = Eigen::Vector3d(2.0, 0.0, 0.0);
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();

  EXPECT_TRUE(proc.process(rclcpp::Time(0, 0, RCL_ROS_TIME), gyro0, acc));  // 初回は即出力
  EXPECT_TRUE(proc.process(rclcpp::Time(1, 0, RCL_ROS_TIME), gyro, acc));
  double angle = proc.orientation().angularDistance(Eigen::Quaterniond::Identity());
  EXPECT_NEAR(angle, 1.0, 1e-3);  // clipped to 1 rad/s for 1s
}

TEST(ImuProcessor, LowPassFiltersAngularVelocity)
{
  ImuProcessorParams params;
  params.bias_sample_count = 1;
  params.clip_enable = false;
  params.dt_check_enable = false;
  params.lpf_enable = true;
  params.lpf_alpha = 0.5;
  ImuProcessor proc(params);

  Eigen::Vector3d gyro0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro = Eigen::Vector3d(1.0, 0.0, 0.0);
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();

  EXPECT_TRUE(proc.process(rclcpp::Time(0, 0, RCL_ROS_TIME), gyro0, acc));
  EXPECT_TRUE(proc.process(rclcpp::Time(0, 100000000, RCL_ROS_TIME), gyro, acc));
  double angle = proc.orientation().angularDistance(Eigen::Quaterniond::Identity());
  // Filtered gyro is 0.5 rad/s for 0.1s -> 0.05 rad
  EXPECT_NEAR(angle, 0.05, 5e-3);
}

TEST(ImuProcessor, DtCheckSkipsOutOfRangeSamples)
{
  ImuProcessorParams params;
  params.clip_enable = false;
  params.lpf_enable = false;
  params.dt_check_enable = true;
  params.dt_min = 0.01;
  params.dt_max = 0.02;
  params.bias_sample_count = 1;
  ImuProcessor proc(params);

  Eigen::Vector3d gyro_init = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro = Eigen::Vector3d(1.0, 0.0, 0.0);
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();

  EXPECT_TRUE(proc.process(rclcpp::Time(0, 0, RCL_ROS_TIME), gyro_init, acc));            // init (bias計測)

  EXPECT_FALSE(proc.process(rclcpp::Time(0, 1000000, RCL_ROS_TIME), gyro, acc));          // dt too small -> skip
  EXPECT_NEAR(proc.orientation().angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-9);

  EXPECT_FALSE(proc.process(rclcpp::Time(0, 30000000, RCL_ROS_TIME), gyro, acc));         // dt too large -> skip
  EXPECT_NEAR(proc.orientation().angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-9);

  EXPECT_FALSE(proc.process(rclcpp::Time(0, 31000000, RCL_ROS_TIME), gyro, acc));         // dt too small -> skip
  EXPECT_NEAR(proc.orientation().angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1e-9);

  EXPECT_TRUE(proc.process(rclcpp::Time(0, 41000000, RCL_ROS_TIME), gyro, acc));          // within range
  // dt = 0.041 - 0.031 = 0.01 （範囲外サンプルは基準時刻更新のみで破棄）
  EXPECT_NEAR(proc.orientation().angularDistance(Eigen::Quaterniond::Identity()), 0.01, 1e-3);
}

TEST(ImuProcessor, IntegratesYawPitchRoll)
{
  ImuProcessorParams params;
  params.bias_sample_count = 1;
  params.clip_enable = false;
  params.lpf_enable = false;
  params.dt_check_enable = false;
  ImuProcessor proc(params);

  Eigen::Vector3d gyro0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro = Eigen::Vector3d(0.0, 0.0, 1.0);
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();

  EXPECT_TRUE(proc.process(rclcpp::Time(0, 0, RCL_ROS_TIME), gyro0, acc));
  for (int i = 1; i <= 10; ++i) {
    EXPECT_TRUE(proc.process(rclcpp::Time(0, i * 100000000, RCL_ROS_TIME), gyro, acc));
  }
  Eigen::Vector3d rpy = proc.orientation().toRotationMatrix().eulerAngles(0, 1, 2);
  // yaw ~1 rad, roll/pitch ~0
  EXPECT_NEAR(rpy[2], 1.0, 1e-2);
  EXPECT_NEAR(rpy[0], 0.0, 1e-3);
  EXPECT_NEAR(rpy[1], 0.0, 1e-3);
}

TEST(YawUnwrapper, KeepsYawContinuousAcrossNegativeWrap)
{
  imu_rpy_pose::YawUnwrapper unwrap;
  EXPECT_NEAR(unwrap.unwrap(3.10), 3.10, 1e-9);
  EXPECT_NEAR(unwrap.unwrap(3.12), 3.12, 1e-9);
  // -3.13rad は +3.153185...rad と連続になるはず（2πラップを除去）
  EXPECT_NEAR(unwrap.unwrap(-3.13), 3.153185307179587, 1e-6);
}

TEST(YawUnwrapper, KeepsYawContinuousAcrossPositiveWrap)
{
  imu_rpy_pose::YawUnwrapper unwrap;
  EXPECT_NEAR(unwrap.unwrap(-3.10), -3.10, 1e-9);
  EXPECT_NEAR(unwrap.unwrap(-3.12), -3.12, 1e-9);
  // +3.13rad は -3.153185...rad と連続になるはず
  EXPECT_NEAR(unwrap.unwrap(3.13), -3.153185307179587, 1e-6);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
