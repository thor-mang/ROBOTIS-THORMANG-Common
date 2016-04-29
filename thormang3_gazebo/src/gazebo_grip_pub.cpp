/*
 * calc_test.cpp
 *
 *  Created on: Jul 11, 2015
 *      Author: sch
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>

ros::Publisher l_arm_grip_joint_pub;
ros::Publisher r_arm_grip_joint_pub;

void l_arm_grip_joint_callback( const std_msgs::Float64::ConstPtr& msg )
{
    std_msgs::Float64 _grip_joint_msg;

    _grip_joint_msg.data = msg->data;

    l_arm_grip_joint_pub.publish( _grip_joint_msg );
}

void r_arm_grip_joint_callback( const std_msgs::Float64::ConstPtr& msg )
{
    std_msgs::Float64 _grip_joint_msg;

    _grip_joint_msg.data = msg->data;

    r_arm_grip_joint_pub.publish( _grip_joint_msg );
}

int main( int argc , char **argv )
{
    ros::init( argc , argv , "gazebo_gripper_publisher" );
    ros::NodeHandle nh("~");

    l_arm_grip_joint_pub  = nh.advertise<std_msgs::Float64>("/thormang3/l_arm_grip_1_pos/command", 0);
    r_arm_grip_joint_pub  = nh.advertise<std_msgs::Float64>("/thormang3/r_arm_grip_1_pos/command", 0);

    ros::Subscriber l_arm_grip_joint_sub = nh.subscribe("/thormang3/l_arm_grip_pos/command", 5, l_arm_grip_joint_callback);
    ros::Subscriber r_arm_grip_joint_sub = nh.subscribe("/thormang3/r_arm_grip_pos/command", 5, r_arm_grip_joint_callback);

	ros::spin();

    return 0;
}
