// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#include "imu_rpy_pose/imu_rpy_pose_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<imu_rpy_pose::ImuRpyPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
