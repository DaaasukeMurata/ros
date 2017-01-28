#ifndef __TAMIYA_RC_H_INCLUDED__
#define __TAMIYA_RC_H_INCLUDED__

#include <ros/ros.h>

class TamiyaRc
{
  public:
    TamiyaRc();
    void go();               // go forwarding
    void stop();             // stop
    void rawSpeedF(float f); // -1.0〜1.0 で進む
    void rawSteerF(float f); // -1.0〜1.0 で曲がる
    void dbgPrint();

  private:
    uint16_t speed;
    uint16_t steer;
    int speed_scale_, steer_scale_;
    ros::NodeHandle nh;
		ros::NodeHandle pnh;   // private
    ros::Publisher servo_pub;

    void sendRc();
};

#endif