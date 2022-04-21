// Copyright 2015-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OFFICE_SCENARIO__OFFICE_SCENARIO_CORE_HPP_
#define OFFICE_SCENARIO__OFFICE_SCENARIO_CORE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

class OfficeScenario : public rclcpp::Node
{
public:
  OfficeScenario();
  ~OfficeScenario();

private:
  void callbackOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_ptr);
  void timerCallback();

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    goal_pose_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr
    engage_pub_;
  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimit>::SharedPtr
    vel_limit_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_ptr_;
};

#endif  // OFFICE_SCENARIO__OFFICE_SCENARIO_CORE_HPP_
