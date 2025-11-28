// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: Apache-2.0

#ifndef IMU_RPY_POSE__IMU_RPY_POSE_NODE_HPP_
#define IMU_RPY_POSE__IMU_RPY_POSE_NODE_HPP_

#include "imu_rpy_pose/imu_processor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace imu_rpy_pose
{

class ImuRpyPoseNode : public rclcpp::Node
{
public:
  explicit ImuRpyPoseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("imu_rpy_pose", options)
  {
    declare_parameters();
    load_parameters();

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      marker_topic_, rclcpp::QoS(1).transient_local());
    yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      yaw_topic_, rclcpp::SensorDataQoS());

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ImuRpyPoseNode::imuCallback, this, std::placeholders::_1));
  }

private:
  void declare_parameters()
  {
    this->declare_parameter<std::string>("imu_topic", "livox/imu");
    this->declare_parameter<int>("bias_sample_count", 200);
    this->declare_parameter<bool>("clip_enable", true);
    this->declare_parameter<double>("clip_max_rad_per_s", 5.0);
    this->declare_parameter<bool>("lpf_enable", true);
    this->declare_parameter<double>("lpf_alpha", 0.2);
    this->declare_parameter<bool>("dt_check_enable", true);
    this->declare_parameter<double>("dt_min", 0.0005);
    this->declare_parameter<double>("dt_max", 0.05);
    this->declare_parameter<std::string>("marker_frame", "livox_frame");
    this->declare_parameter<std::string>("marker_topic", "imu_arrow");
    this->declare_parameter<std::string>("yaw_topic", "imu_yaw");
  }

  void load_parameters()
  {
    imu_topic_ = this->get_parameter("imu_topic").as_string();

    ImuProcessorParams params;
    params.bias_sample_count = this->get_parameter("bias_sample_count").as_int();
    params.clip_enable = this->get_parameter("clip_enable").as_bool();
    params.clip_max_rad_per_s = this->get_parameter("clip_max_rad_per_s").as_double();
    params.lpf_enable = this->get_parameter("lpf_enable").as_bool();
    params.lpf_alpha = this->get_parameter("lpf_alpha").as_double();
    params.dt_check_enable = this->get_parameter("dt_check_enable").as_bool();
    params.dt_min = this->get_parameter("dt_min").as_double();
    params.dt_max = this->get_parameter("dt_max").as_double();

    marker_frame_ = this->get_parameter("marker_frame").as_string();
    marker_topic_ = this->get_parameter("marker_topic").as_string();
    yaw_topic_ = this->get_parameter("yaw_topic").as_string();

    processor_.set_params(params);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    Eigen::Vector3d gyro(
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z);
    Eigen::Vector3d acc(
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z);

    const bool updated = processor_.process(msg->header.stamp, gyro, acc);

    if (processor_.bias_ready() && !calibration_announced_) {
      RCLCPP_INFO(this->get_logger(), "IMUバイアスのキャリブレーションが完了しました");
      calibration_announced_ = true;
    }

    if (!processor_.bias_ready() || !updated) {
      return;
    }

    const auto & ori = processor_.orientation();
    tf2::Quaternion q;
    q.setX(ori.x());
    q.setY(ori.y());
    q.setZ(ori.z());
    q.setW(ori.w());
    const double yaw = tf2::getYaw(q);

    std_msgs::msg::Float64 yaw_msg;
    yaw_msg.data = yaw;
    yaw_pub_->publish(yaw_msg);

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = msg->header.stamp;
    marker.header.frame_id = marker_frame_;
    marker.ns = "imu_rpy_pose";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = processor_.position().x();
    marker.pose.position.y = processor_.position().y();
    marker.pose.position.z = processor_.position().z();
    marker.pose.orientation = tf2::toMsg(q);
    marker.scale.x = 0.3;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.1;
    marker.color.g = 0.8;
    marker.color.b = 0.2;
    marker.lifetime = rclcpp::Duration(0, 0);
    marker_pub_->publish(marker);
  }

  std::string imu_topic_;
  std::string marker_frame_;
  std::string marker_topic_;
  std::string yaw_topic_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;
  bool calibration_announced_{false};
  ImuProcessor processor_;
};

}  // namespace imu_rpy_pose

#endif  // IMU_RPY_POSE__IMU_RPY_POSE_NODE_HPP_
