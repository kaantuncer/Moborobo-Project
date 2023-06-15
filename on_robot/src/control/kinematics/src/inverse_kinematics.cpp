#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <iostream>

#include "nav_msgs/Odometry.h"
#include "robot_msgs/MotorSpeedCommand.h"

#define PI 3.14159265359

ros::Time last_time;
ros::Time current_time;

using namespace std;

bool rpm_msg_came;
robot_msgs::MotorSpeedCommand rpm_msg;
void motor_rpm_cb(const robot_msgs::MotorSpeedCommand::ConstPtr &msg)
{
    rpm_msg = *msg;
    rpm_msg_came = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inverse_kinematics_node");
    ros::NodeHandle _nh("~");
    ros::NodeHandle nh;
    double base_width = 0.0;
    double wheel_radius = 0.1;
    double reduction_rate = 1.0;

    // params
    _nh.param<double>("base_width", base_width, 0.1);
    _nh.param<double>("wheel_radius", wheel_radius, 0.1);
    _nh.param<double>("reduction_rate", reduction_rate, 1);
    ros::Subscriber rpm_sub = nh.subscribe<robot_msgs::MotorSpeedCommand>("/motor_rpms", 1, motor_rpm_cb);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    tf::TransformBroadcaster odom_broadcaster;
    last_time = ros::Time::now();

    double p_x = 0.0, p_y = 0.0, p_th = 0.0;
    double v_right, v_left, dt;
    double v_rx, v_ry, v_th;
    double v_wx, v_wy;

    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now(); // source: wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

        if (rpm_msg_came)
        {
            dt = (rpm_msg.header.stamp - last_time).toSec();
            last_time = rpm_msg.header.stamp;
            // cout << "this is the difference: " << current_time - last_time << endl;

            v_right = (2 * PI * wheel_radius) * (rpm_msg.right / reduction_rate) / 60.0; // 2.0; Right Wheel Velocity in m/sn (right side of the multiplication is the angular velocity of the wheel)
            v_left = (2 * PI * wheel_radius) * (rpm_msg.left / reduction_rate) / 60.0;   // 2.0;  Left  Wheel Velocity in m/sn (right side of the multiplication is the angular velocity of the wheel)

            v_rx = (v_right + v_left) / 2.0;
            v_ry = 0.0;

            v_th = (v_right - v_left) / base_width;
            p_th += v_th * dt;
    
            v_wx = v_rx * cos(p_th);
            v_wy = v_rx * sin(p_th);

            p_x += v_wx * dt;
            p_y += v_wy * dt;

            odom_msg.header.stamp = last_time; // ros::Time::now();
            odom_msg.pose.pose.position.x = p_x;
            odom_msg.pose.pose.position.y = p_y;
            odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(p_th);
            odom_pub.publish(odom_msg);

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(p_th);
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = last_time; // ros::Time::now();
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = p_x;
            odom_trans.transform.translation.y = p_y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            odom_broadcaster.sendTransform(odom_trans);
            rpm_msg_came = false;
        }
        rate.sleep();
    }
}
