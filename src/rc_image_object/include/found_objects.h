#ifndef __FOUND_OBJECTS_H_INCLUDED__
#define __FOUND_OBJECTS_H_INCLUDED__

#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>
#include "squaref.h"

typedef struct
{
    float x;
    float y;
} FObjNormalizedPosition;

// find_2d_objectのObject
class FoundObject : public SquareF
{
  public:
    FoundObject(int idVal,
                float width, float height,
                float tlX, float tlY,
                float trX, float trY,
                float blX, float blY,
                float brX, float brY);
    int getId();
    FObjNormalizedPosition getNormalizedPosition(); /* -1.0 〜 1.0 を超えることもある */
    void dbgPrint();

  private:
    ros::NodeHandle pnh;  // private
    int id;
    float width;
    float height;
    int cameraWidth, cameraHeight;
};

// 検出したObject全部
class ObjectList
{
  public:
    int addObject(std_msgs::Float32MultiArray &array);
    std::size_t getObjNum();
    FoundObject getObjByIndex(int index);
    void dbgPrint();

  private:
    std::vector<FoundObject> objList;
};

#endif