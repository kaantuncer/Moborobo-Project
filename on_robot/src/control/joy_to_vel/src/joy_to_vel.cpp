#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

#include <math.h>
#include <iostream>
#include <string.h>

double max_vx = 1.0;
double max_rad_sn = -6.2831853072 /6;

using namespace std;

bool joy_msg_came = false;

bool joy_status = false; // triggered by halt button

sensor_msgs::Joy joy_msg;
void joy_cb(const sensor_msgs::Joy::ConstPtr &msg)
{
    joy_msg = *msg;
    joy_msg_came = true;
}

void joyStatusCb(const std_msgs::Bool::ConstPtr &msg)
{
    joy_status = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_to_vel_node");
    ros::NodeHandle n;

    ros::Subscriber joy_al = n.subscribe<sensor_msgs::Joy>("/joy", 1, joy_cb);
    ros::Subscriber joy_status_sub = n.subscribe<std_msgs::Bool>("/joy_status", 10, joyStatusCb);
    ros::Publisher cmd_vel_pb = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher uv_pub = n.advertise<std_msgs::Bool>("/uv_command", 1);

    bool uv_opened = false;
    bool uv_pressed = false;
    ros::Time last_time = ros::Time::now();

    ros::Rate rate(100);
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();

        // if halt button is active
        if (joy_msg_came && joy_msg.buttons[4] == 1 ) // && joy_status)
        {
            geometry_msgs::Twist twist_msg;
            if (abs(joy_msg.axes[4]) > 0.01)
                twist_msg.linear.x = joy_msg.axes[4] * max_vx;
            if (abs(joy_msg.axes[0]) > 0.01)
                twist_msg.angular.z = -1*joy_msg.axes[0] * max_rad_sn;
            cmd_vel_pb.publish(twist_msg);
        }

        if (joy_msg_came)
        {
            if (joy_msg.axes[5] < -0.75)
            {
                if (!uv_pressed)
                {
                    uv_pressed = true;
                    last_time = ros::Time::now();
                }
                else
                {
                    if ((ros::Time::now() - last_time).toSec() >= 3.0)
                    {
                        if (!uv_opened)
                        {
                            ROS_INFO("Aciliyor");
                            std_msgs::Bool uv_msg;
                            uv_msg.data = true;
                            uv_pub.publish(uv_msg);
                            uv_opened = true;
                        }
                        else
                        {
                            ROS_INFO("Kapatiliyor");
                            std_msgs::Bool uv_msg;
                            uv_msg.data = false;
                            uv_pub.publish(uv_msg);
                            uv_opened = false;
                        }
                        uv_pressed = false;
                    }
                }
            }
            else if (joy_msg.axes[5] > -0.75 && uv_pressed)
            {
                uv_pressed = false;
            }
        }
    }
}
