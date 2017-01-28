#include <iostream>
#include <iomanip>
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "squaref.h"
#include "found_objects.h"

FoundObject::FoundObject(int idVal,
                         float width, float height,
                         float tlX, float tlY,
                         float trX, float trY,
                         float blX, float blY,
                         float brX, float brY)
    : pnh("~"), cameraWidth(640), cameraHeight(480)
{
    pnh.param("camera_width", cameraWidth, cameraWidth);
    pnh.param("camera_height", cameraHeight, cameraHeight);

    this->id = idVal;
    this->width = width;
    this->height = height;
    this->topLeft.x = tlX;
    this->topLeft.y = tlY;
    this->topRight.x = trX;
    this->topRight.y = trY;
    this->bottomLeft.x = blX;
    this->bottomLeft.y = blY;
    this->bottomRight.x = brX;
    this->bottomRight.y = brY;
}

int FoundObject::getId()
{
    return this->id;
}

/* 中心からのobject位置          */
/* -1.0 〜 1.0 を超えることもある */
FObjNormalizedPosition FoundObject::getNormalizedPosition()
{
    FObjNormalizedPosition rtnPos;
    rtnPos.x = (this->getCenter().x - (cameraWidth / 2)) / (cameraWidth / 2);
    rtnPos.y = (this->getCenter().y - (cameraHeight / 2)) / (cameraHeight / 2);

    return rtnPos;
}

void FoundObject::dbgPrint()
{
    std::cout << "[FoundObject] id     : " << id << std::endl;
    std::cout << "[FoundObject] width  : " << width << std::endl;
    std::cout << "[FoundObject] height : " << height << std::endl;
    std::cout << "[FoundObject] topLeft     : x = " << topLeft.x << "y = " << topLeft.y << std::endl;
    std::cout << "[FoundObject] topRight    : x = " << topRight.x << "y = " << topRight.y << std::endl;
    std::cout << "[FoundObject] bottomLeft  : x = " << bottomLeft.x << "y = " << bottomLeft.y << std::endl;
    std::cout << "[FoundObject] bottomRight : x = " << bottomRight.x << "y = " << bottomRight.y << std::endl;
}

/* ---for ObjectList class--- */

int ObjectList::addObject(std_msgs::Float32MultiArray &array)
{
    int num = 0;

    if (array.data.size())
    {
        // 12個で1つのobject
        for (unsigned int i = 0; i < array.data.size(); i += 12)
        {
            int id = (int)array.data[i];
            float objectWidth = array.data[i + 1];
            float objectHeight = array.data[i + 2];

            // Find corners OpenCV
            cv::Mat cvHomography(3, 3, CV_32F);
            cvHomography.at<float>(0, 0) = array.data[i + 3];
            cvHomography.at<float>(1, 0) = array.data[i + 4];
            cvHomography.at<float>(2, 0) = array.data[i + 5];
            cvHomography.at<float>(0, 1) = array.data[i + 6];
            cvHomography.at<float>(1, 1) = array.data[i + 7];
            cvHomography.at<float>(2, 1) = array.data[i + 8];
            cvHomography.at<float>(0, 2) = array.data[i + 9];
            cvHomography.at<float>(1, 2) = array.data[i + 10];
            cvHomography.at<float>(2, 2) = array.data[i + 11];
            std::vector<cv::Point2f> inPts, outPts;
            inPts.push_back(cv::Point2f(0, 0));
            inPts.push_back(cv::Point2f(objectWidth, 0));
            inPts.push_back(cv::Point2f(0, objectHeight));
            inPts.push_back(cv::Point2f(objectWidth, objectHeight));
            cv::perspectiveTransform(inPts, outPts, cvHomography);

            FoundObject obj(id,
                            objectWidth, objectHeight,
                            outPts.at(0).x, outPts.at(0).y,
                            outPts.at(1).x, outPts.at(1).y,
                            outPts.at(2).x, outPts.at(2).y,
                            outPts.at(3).x, outPts.at(3).y);

            this->objList.push_back(obj);
            num++;
        }
    }
    else
    {
        std::cout << "[ObjectList]"
                  << "No Object found." << std::endl;
    }

    return num;
}

std::size_t ObjectList::getObjNum()
{
    return this->objList.size();
}

FoundObject ObjectList::getObjByIndex(int index)
{
    return this->objList[index];
}

void ObjectList::dbgPrint()
{
    std::size_t num = getObjNum();

    std::cout << "[ObjectList]"
              << " Number of objects : " << num << std::endl;

    for (std::size_t i = 0; i < num; i++)
    {
        std::cout << "[ObjectList]"
                  << "id : " << std::setw(4) << std::left << objList[i].getId()
                  << "x : " << std::setw(8) << std::left << objList[i].getCenter().x
                  << "y : " << std::setw(8) << std::left << objList[i].getCenter().y << std::endl;
    }
}
