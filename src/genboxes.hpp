#ifndef GEN_BOXES_H
#define GEN_BOXES_H

#include <opencv2/opencv.hpp>

enum Zone
{
    TOP_LEFT,
    TOP_RIGHT,
    BOTTOM_LEFT,
    BOTTOM_RIGHT
};

class Box
{
private:
    cv::Point barycenter;
    cv::Size boxSize;
    cv::Point2f velocity;
    cv::Point destination;
    cv::Point pos;
    Zone zone;

public:
    Box();
    Box(cv::Point barycenter, cv::Size boxSize);
    ~Box();

    void setCoordinates(cv::Point barycenter);
    cv::Point getTopLeft();
    cv::Point getBottomRight();
    void updatePosition();
    void setDestination(cv::Point dest);
    void setVelocity(cv::Point2f vel);
    void setZone(Zone zone);
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
    cv::Point genRandomDestination();
    Box generateRandomBox();
    Box getBox();
    void drawBox();
};

BoxManager::BoxManager() {}

#endif