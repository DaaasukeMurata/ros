#include <iostream>
#include <iomanip>
#include "squaref.h"

SquareF::SquareF(float tlX, float tlY,
                 float trX, float trY,
                 float blX, float blY,
                 float brX, float brY)
{
    topLeft.x = tlX;
    topLeft.y = tlY;
    topRight.x = trX;
    topRight.y = trY;
    bottomLeft.x = blX;
    bottomLeft.y = blY;
    bottomRight.x = brX;
    bottomRight.y = brY;
}

// 中心座標
PointF SquareF::getCenter()
{
    PointF point;

    /* 左上と右下の真ん中の座標とする */
    point.x = (this->topLeft.x + this->bottomRight.x) / 2;
    point.y = (this->topLeft.y + this->bottomRight.y) / 2;

    return point;
}

void SquareF::dbgPrint()
{
    std::cout << "[SquareF] topLeft     : x = " << topLeft.x << "y = " << topLeft.y << std::endl;
    std::cout << "[SquareF] topRight    : x = " << topLeft.x << "y = " << topLeft.y << std::endl;
    std::cout << "[SquareF] bottomLeft  : x = " << topLeft.x << "y = " << topLeft.y << std::endl;
    std::cout << "[SquareF] bottomRight : x = " << topLeft.x << "y = " << topLeft.y << std::endl;
}
