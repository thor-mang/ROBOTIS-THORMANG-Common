/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 * gazebo_grip_pub.cpp
 *
 *  Created on: Jul 11, 2015
 *      Author: SCH
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>

ros::Publisher g_left_arm_grip_joint_pub;
ros::Publisher g_right_arm_grip_joint_pub;

void leftArmGripJointCallback(const std_msgs::Float64::ConstPtr& msg)
{
  std_msgs::Float64 grip_joint_msg;

  grip_joint_msg.data = msg->data;

  g_left_arm_grip_joint_pub.publish(grip_joint_msg);
}

void rightArmGripJointCallback(const std_msgs::Float64::ConstPtr& msg)
{
  std_msgs::Float64 grip_joint_msg;

  grip_joint_msg.data = msg->data;

  g_right_arm_grip_joint_pub.publish(grip_joint_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_gripper_publisher");
  ros::NodeHandle nh("~");

  g_left_arm_grip_joint_pub = nh.advertise<std_msgs::Float64>("/thormang3/l_arm_grip_1_position/command", 0);
  g_right_arm_grip_joint_pub = nh.advertise<std_msgs::Float64>("/thormang3/r_arm_grip_1_position/command", 0);

  ros::Subscriber l_arm_grip_joint_sub = nh.subscribe("/thormang3/l_arm_grip_position/command", 5, leftArmGripJointCallback);
  ros::Subscriber r_arm_grip_joint_sub = nh.subscribe("/thormang3/r_arm_grip_position/command", 5, rightArmGripJointCallback);

  ros::spin();

  return 0;
}
