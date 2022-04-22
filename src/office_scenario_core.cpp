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

#include "office_100_laps/office_100_laps.hpp"

#include <cmath>
#include <memory>
#include <string>

OPfficeScenario::OPfficeScenario()
: Node("office_100_laps"),
  output_frame_(declare_parameter("base_link", "base_link")),
  message_timeout_sec_(declare_parameter("message_timeout_sec", 0.2))
{
	goal_tolerance_ = 1.0;
  goal_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", rclcpp::QoS{10});
  engage_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::Engage>("engage", rclcpp::QoS{10});
  vel_lim_pub_ = create_publisher<tier4_planning_msgs::msg::VelocityLimit>("velocity_limit", rclcpp::QoS{10});

	// publish velocity limit
}

OPfficeScenario::~OPfficeScenario() {}

void OPfficeScenario::callbackOdometry(
  const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_ptr)
{
  odom_msg_ptr_ = odom_msg_ptr;
}

void OPfficeScenario::timerCallback()
{
	double distance_to_goal;
  if (distance_to_goal < goal_tolerance_) { // close enough to current coal
		// change goal (a->b or b->a)
		// publish goal
		geometry_msgs::msg::Pose::ConstSharedPtr goal_pose_msg;
		publish
		// publish engage
	} else { // not reached to the current goal yet
		continue;
	}
}
