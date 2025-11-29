// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/msg/tf_message.hpp>
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

TEST_F(ImuRpyPosePublishTest, PublishesYawTfWithYawOnly)
{
  auto node_options = rclcpp::NodeOptions()
    .append_parameter_override("bias_sample_count", 1)
    .append_parameter_override("dt_check_enable", false)
    .append_parameter_override("publish_yaw_tf", true)
    .append_parameter_override("yaw_tf_parent_frame", "odom")
    .append_parameter_override("yaw_tf_child_frame", "test_imu_heading");

  auto node = std::make_shared<imu_rpy_pose::ImuRpyPoseNode>(node_options);

  // Publish IMU message
  auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("livox/imu", 10);

  std::mutex m;
  std::condition_variable cv;
  bool received = false;
  geometry_msgs::msg::TransformStamped received_tf;
  double received_yaw = 0.0;

  auto listener = std::make_shared<rclcpp::Node>("tf_listener_test");
  auto tf_sub = listener->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::QoS(10),
    [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(m);
      for (const auto & t : msg->transforms) {
        if (t.child_frame_id == "test_imu_heading" && t.header.frame_id == "odom") {
          tf2::Quaternion q_tf(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w);
          tf2::Matrix3x3 m_tf(q_tf);
          double r_tf, p_tf, y_tf;
          m_tf.getRPY(r_tf, p_tf, y_tf);
          // yawが更新されるまで待つ（初回0radをスキップ）
          if (std::abs(y_tf) < 1e-4) {
            continue;
          }
          received_tf = t;
          received_yaw = y_tf;
          received = true;
          cv.notify_one();
          return;
        }
      }
    });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(listener);

  sensor_msgs::msg::Imu imu;
  imu.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  imu.angular_velocity.x = 0.0;
  imu.angular_velocity.y = 0.0;
  imu.angular_velocity.z = 0.0;
  imu.linear_acceleration.x = 0.0;
  imu.linear_acceleration.y = 0.0;
  imu.linear_acceleration.z = 9.8;

  imu_pub->publish(imu);  // bias確定用（bias_sample_count=1）
  imu.angular_velocity.z = 0.5;  // yawが0.5*0.01=0.005 rad になるはず

  auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline && !received) {
    imu.header.stamp = node->now();
    imu_pub->publish(imu);
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  EXPECT_TRUE(received);
  if (received) {
    EXPECT_NEAR(received_tf.transform.translation.x, 0.0, 1e-6);
    EXPECT_NEAR(received_tf.transform.translation.y, 0.0, 1e-6);
    EXPECT_NEAR(received_tf.transform.translation.z, 0.0, 1e-6);

    tf2::Quaternion q(
      received_tf.transform.rotation.x,
      received_tf.transform.rotation.y,
      received_tf.transform.rotation.z,
      received_tf.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    EXPECT_NEAR(roll, 0.0, 1e-6);
    EXPECT_NEAR(pitch, 0.0, 1e-6);
    EXPECT_GT(std::abs(yaw), 1e-4);
    EXPECT_GT(std::abs(received_yaw), 1e-4);
  }
}
