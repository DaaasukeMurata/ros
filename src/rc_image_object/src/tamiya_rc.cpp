#include <iostream>
#include <iomanip>
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include "tamiya_rc.h"

TamiyaRc::TamiyaRc() : steer_scale_(80), speed_scale_(10), pnh("~")
{
    steer = 90;
    speed = 90;

    pnh.param("scale_speed", speed_scale_, speed_scale_);
    pnh.param("scale_steer", steer_scale_, steer_scale_);
    servo_pub = nh.advertise<std_msgs::UInt16MultiArray>("servo", 1);
}

void TamiyaRc::sendRc()
{
    std_msgs::UInt16MultiArray cmd_msg;

    cmd_msg.data.clear();
    cmd_msg.data.push_back((uint16_t)(steer));
    cmd_msg.data.push_back((uint16_t)(speed));
    servo_pub.publish(cmd_msg);
}

void TamiyaRc::go()
{
    speed = 90 - speed_scale_;
    sendRc();
}

void TamiyaRc::stop()
{
    speed = 90;
    sendRc();
}

/* -1.0〜1.0 */
void TamiyaRc::rawSpeedF(float speedF)
{
    speed = 90 + (90 * speedF);
    sendRc();
}

/* -1.0〜1.0 */
void TamiyaRc::rawSteerF(float steerF)
{
    steer = 90 + (90 * steerF);
    sendRc();
}

void TamiyaRc::dbgPrint()
{
    std::cout << "[TamiyaRC]"
              << "speed : " << speed << "steer : " << steer << std::endl;
}