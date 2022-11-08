#ifndef GEN_BOXES_H
#define GEN_BOXES_H

#include <opencv2/opencv.hpp>

class Box
{
private:
    cv::Point topLeft;
    cv::Point bottomRight;
    cv::Point barycentre;

public:
    Box();
    Box(cv::Point topLeft, cv::Point bottomRight);
    ~Box();

    void setCoordinates(cv::Point topLeft, cv::Point bottomRight);
    cv::Point getTopLeft();
    cv::Point getBottomRight();
    void calcBarycentre();
};

Box::Box() {}

class BoxManager
{
private:
    Box box;
    cv::Size imageSize;
    int randomSeed;

public:
    BoxManager();
    BoxManager(cv::Size imageSize, int randomSeed);
    ~BoxManager();
    void generateRandomTrajectory();
    Box generateRandomBox();
    Box getBox();
};

BoxManager::BoxManager() {}

#endif