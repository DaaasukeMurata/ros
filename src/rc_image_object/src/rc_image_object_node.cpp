#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/opencv.hpp>
#include "found_objects.h"
#include "tamiya_rc.h"

#define STEER_COEF 1.2 // 曲がりやすさ

class TrackingObjRc
{
  public:
    TrackingObjRc();

  private:
    ros::NodeHandle nh;
    ros::Subscriber objects_sub;
    TamiyaRc tamiyarc;
    void objCb(const std_msgs::Float32MultiArray &msg);
};

TrackingObjRc::TrackingObjRc()
{
    // find_2d_objectから配信される、/objectsの購読
    objects_sub = nh.subscribe("objects", 10, &TrackingObjRc::objCb, this);
}

// objectsのcallback
void TrackingObjRc::objCb(const std_msgs::Float32MultiArray &msg)
{

    ROS_INFO("TrackingOjbRc::objCb() start");

    ObjectList objList;
    std_msgs::Float32MultiArray wkArray = msg;

    if (wkArray.data.size())
    {
        // object登録
        objList.addObject(wkArray);
        objList.dbgPrint();

        // move
        FObjNormalizedPosition pos;
        pos = objList.getObjByIndex(0).getNormalizedPosition(); // とりあえず最初に見つかったobject

        pos.x = pos.x * STEER_COEF;

        if (pos.x < -1.0)
            pos.x = -1.0;
        else if (pos.x > 1.0)
            pos.x = 1.0;

        tamiyarc.go();
        tamiyarc.rawSteerF(pos.x);
        tamiyarc.dbgPrint();
    }
    else
    {
        ROS_INFO("No Object Found");
        tamiyarc.stop();
        tamiyarc.dbgPrint();
    }

    return;
}

int main(int argc, char **argv)
{
    ROS_INFO("start rc_image_object_node");

    // node名 = rc_image_node
    ros::init(argc, argv, "rc_image_object_node");

    TrackingObjRc trackingObjrc;

    ros::spin();
    return 0;
}