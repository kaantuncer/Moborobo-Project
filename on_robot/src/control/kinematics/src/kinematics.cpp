#include "ros/ros.h"
#include <iostream>

#include <geometry_msgs/Twist.h>
#include "robot_msgs/MotorSpeedCommand.h"

#define PI 3.14159265359

double base_width = 0.0;
double wheel_radius = 0.1;
double reduction_rate = 1.0;

using namespace std;

geometry_msgs::Twist cmd_vel_msg;
void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_vel_msg = *msg;
}

robot_msgs::MotorSpeedCommand calculateKinematics(geometry_msgs::Twist cmd_vel)
{
    robot_msgs::MotorSpeedCommand msc_msg;

    double vr = 1.0 * cmd_vel.linear.x + cmd_vel.angular.z * base_width / 2.0;
    double vl = 1.0 * cmd_vel.linear.x - cmd_vel.angular.z * base_width / 2.0;

    msc_msg.right = (60.0 / (2 * PI * wheel_radius)) * vr * reduction_rate;
    msc_msg.left = (60.0 / (2 * PI * wheel_radius)) * vl * reduction_rate;

    return msc_msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematics_node");
    ros::NodeHandle nh;
    ros::NodeHandle _nh("~");
    // params
    _nh.param<double>("base_width", base_width, 0.1);
    _nh.param<double>("wheel_radius", wheel_radius, 0.1);
    _nh.param<double>("reduction_rate", reduction_rate, 1);

    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmd_vel_cb);
    ros::Publisher msc_msg_pub = nh.advertise<robot_msgs::MotorSpeedCommand>("motor_commands", 1);

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();

        msc_msg_pub.publish(calculateKinematics(cmd_vel_msg));
    }
}
