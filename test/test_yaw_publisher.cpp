// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>

#include "imu_rpy_pose/imu_processor.hpp"
#include "imu_rpy_pose/imu_rpy_pose_node.hpp"

using namespace std::chrono_literals;

class ImuRpyPosePublishTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(ImuRpyPosePublishTest, PublishesYawAfterBiasReady)
{
  auto node_options = rclcpp::NodeOptions()
    .append_parameter_override("bias_sample_count", 1)
    .append_parameter_override("dt_check_enable", false)
    .append_parameter_override("yaw_topic", "test_yaw");

  auto node = std::make_shared<imu_rpy_pose::ImuRpyPoseNode>(node_options);

  // Publish IMU message
  auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("livox/imu", 10);

  std::mutex m;
  std::condition_variable cv;
  bool received = false;
  double received_yaw = 1.0;

  auto yaw_sub = node->create_subscription<std_msgs::msg::Float64>(
    "test_yaw", rclcpp::SensorDataQoS(),
    [&](const std_msgs::msg::Float64::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(m);
      received = true;
      received_yaw = msg->data;
      cv.notify_one();
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  sensor_msgs::msg::Imu imu;
  imu.header.stamp = node->now();
  imu.angular_velocity.x = 0.0;
  imu.angular_velocity.y = 0.0;
  imu.angular_velocity.z = 0.0;
  imu.linear_acceleration.x = 0.0;
  imu.linear_acceleration.y = 0.0;
  imu.linear_acceleration.z = 9.8;

  imu_pub->publish(imu);

  auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline && !received) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  EXPECT_TRUE(received);
  if (received) {
    EXPECT_NEAR(received_yaw, 0.0, 1e-4);
  }
}
