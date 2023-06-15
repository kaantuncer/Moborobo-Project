#include <ros/ros.h>
#include <iostream>
#include <time.h>

#include "std_msgs/Bool.h"
#include <roboteq_driver/driver.h>
#include "std_msgs/Int32.h"
#include<robot_msgs/MotorSpeedCommand.h>
using namespace std;

#define LEFT_MOTOR_ID 2
#define RIGHT_MOTOR_ID 1

#define GREEN_BUTTON_PIN_ID 7
#define BLUE_BUTTON_PIN_ID 6
#define RED_BUTTON_PIN_ID 4
#define SAFETY_BUTTON_PIN_ID 2

robot_msgs::MotorSpeedCommand cmd_msg;
void motor_command_cb(const robot_msgs::MotorSpeedCommand::ConstPtr &msg)
{
    cmd_msg = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roboteq_driver");
    ros::NodeHandle n;
    Driver driver;

    ros::Subscriber motor_command_sub = n.subscribe<robot_msgs::MotorSpeedCommand>("/motor_commands", 1, motor_command_cb);

    ros::Publisher rpm_pub = n.advertise<robot_msgs::MotorSpeedCommand>("/motor_rpms", 1);
    ros::Publisher joy_status_pub = n.advertise<std_msgs::Bool>("/joy_status", 10);
    int rpm_l, rpm_r;
    robot_msgs::MotorSpeedCommand motor_rpms;
    ros::Rate rate(100);

    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();

        // stop the robot if safety stop is active.

        if (driver.GetButtonStatus(SAFETY_BUTTON_PIN_ID))
        {
            driver.TurnWheelRPM(LEFT_MOTOR_ID, 0);
            driver.TurnWheelRPM(RIGHT_MOTOR_ID, 0);
        }
        else
        {
            driver.TurnWheelRPM(LEFT_MOTOR_ID, cmd_msg.left);
            driver.TurnWheelRPM(RIGHT_MOTOR_ID, cmd_msg.right);
        }

        if (!driver.GetMotorRPM(LEFT_MOTOR_ID, rpm_l) && !driver.GetMotorRPM(RIGHT_MOTOR_ID, rpm_r))
        {

            motor_rpms.header.stamp = ros::Time::now();
            motor_rpms.header.frame_id = "base_link";
            motor_rpms.left = rpm_l;
            motor_rpms.right = rpm_r;
            rpm_pub.publish(motor_rpms);
        }

        motor_rpms.left = 0;
        motor_rpms.right = 0;

        // green button
        bool green_button_status = driver.GetButtonStatus(GREEN_BUTTON_PIN_ID);
        std_msgs::Bool green_button_msg;
        if (green_button_status)
        {
            green_button_msg.data = true;
        }
        else
        {
            green_button_msg.data = false;
        }
        joy_status_pub.publish(green_button_msg);
    }
}
