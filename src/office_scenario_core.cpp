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

#include "office_scenario/office_scenario_core.hpp"

#include <cmath>
#include <memory>
#include <string>

OfficeScenario::OfficeScenario()
: Node("office_scenario")
{
  goal_tolerance_ = 2.0;
  timer_dt_ = 0.1;
  going_to_point_a_ = true;
  lap_num_ = 0;

  goal_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("out_goal_pose", rclcpp::QoS{10});
  engage_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::Engage>("out_engage", rclcpp::QoS{10});
  vel_lim_pub_ = create_publisher<tier4_planning_msgs::msg::VelocityLimit>("out_velocity_limit", rclcpp::QoS{10});

	odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "in_odom", rclcpp::QoS{100}, std::bind(&OfficeScenario::callbackOdometry, this, std::placeholders::_1));

	odom_msg_ptr_ = nullptr;

	// publish velocity limit
	tier4_planning_msgs::msg::VelocityLimit vel_lim_msg;
	vel_lim_msg.max_velocity = 0.5;
	vel_lim_pub_->publish(vel_lim_msg);

	std::cout << "published velocity limit" << std::endl;

	// HARD CODE point a and b
	goal_a_px_ = -4.888422012329102;
	goal_a_py_ = 3.6404664516448975;
	goal_a_qz_ = 0.7043329698624418;
	goal_a_qw_ = 0.7098697539441673;

	goal_b_px_ = 5.498318672180176;
	goal_b_py_ = 5.696805000305176;
	goal_b_qz_ = -0.642459769470317;
	goal_b_qw_ = 0.7663194142210852;
	// point_a_pose_ptr_->position.x = -4.888422012329102;
	// point_a_pose_ptr_->position.y = 3.6404664516448975;
	// point_a_pose_ptr_->position.z = 0.0;
	// point_a_pose_ptr_->orientation.x = 0.0;
	// point_a_pose_ptr_->orientation.y = 0.0;
	// point_a_pose_ptr_->orientation.z = 0.7043329698624418;
	// point_a_pose_ptr_->orientation.w = 0.7098697539441673;

	// point_b_pose_ptr_->position.x = 5.498318672180176;
	// point_b_pose_ptr_->position.y = 5.696805000305176;
	// point_b_pose_ptr_->position.z = 0.0;
	// point_b_pose_ptr_->orientation.x = 0.0;
	// point_b_pose_ptr_->orientation.y = 0.0;
	// point_b_pose_ptr_->orientation.z = -0.642459769470317;
	// point_b_pose_ptr_->orientation.w = 0.7663194142210852;
  auto period_control_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(timer_dt_));
  timer_control_ = rclcpp::create_timer(
    this, get_clock(), period_control_ns, std::bind(&OfficeScenario::timerCallback, this));

}

OfficeScenario::~OfficeScenario() {}

void OfficeScenario::callbackOdometry(
  const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_ptr)
{
  odom_msg_ptr_ = odom_msg_ptr;
}

void OfficeScenario::timerCallback()
{
	if (odom_msg_ptr_ == nullptr) {return;}

	double distance_to_goal;
	double dx, dy;
	if (going_to_point_a_) {
		// dx = point_a_pose_ptr_->position.x - odom_msg_ptr_->pose.pose.position.x;
		// dy = point_a_pose_ptr_->position.y - odom_msg_ptr_->pose.pose.position.y;
		dx = goal_a_px_ - odom_msg_ptr_->pose.pose.position.x;
		dy = goal_a_py_ - odom_msg_ptr_->pose.pose.position.y;
	} else {
		// dx = point_b_pose_ptr_->position.x - odom_msg_ptr_->pose.pose.position.x;
		// dy = point_b_pose_ptr_->position.y - odom_msg_ptr_->pose.pose.position.y;
		dx = goal_b_px_ - odom_msg_ptr_->pose.pose.position.x;
		dy = goal_b_py_ - odom_msg_ptr_->pose.pose.position.y;
	}
	distance_to_goal = std::sqrt(dx * dx + dy * dy);

  if (distance_to_goal < goal_tolerance_) { // close enough to current coal
		std::cout << "reached goal" << std::endl;

		// change goal (a->b or b->a)
		going_to_point_a_ = !going_to_point_a_;

		// publish goal
		geometry_msgs::msg::PoseStamped goal_pose_msg;
		goal_pose_msg.header.stamp = this->now();
		goal_pose_msg.header.frame_id = "map";
		if (going_to_point_a_) {
			std::cout << "next goal: point a" << std::endl;

			goal_pose_msg.pose.position.x = goal_a_px_;
			goal_pose_msg.pose.position.y = goal_a_py_;
			goal_pose_msg.pose.orientation.z = goal_a_qz_;
			goal_pose_msg.pose.orientation.w = goal_a_qw_;
		} else {
			std::cout << "next goal: point b" << std::endl;

			goal_pose_msg.pose.position.x = goal_b_px_;
			goal_pose_msg.pose.position.y = goal_b_py_;
			goal_pose_msg.pose.orientation.z = goal_b_qz_;
			goal_pose_msg.pose.orientation.w = goal_b_qw_;
		}
		goal_pose_pub_->publish(goal_pose_msg);

    rclcpp::sleep_for(std::chrono::milliseconds(1000));
		// publish engage
		autoware_auto_vehicle_msgs::msg::Engage engage_msg;
		engage_msg.engage = true;
		engage_pub_->publish(engage_msg);

		if (going_to_point_a_){
			++lap_num_;
			std::cout << "Current lap: " << lap_num_ << std::endl;
		}
	} else { // not reached to the current goal yet
		return;
	}
}
