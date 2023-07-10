#ifndef GEN_BOXES_H
#define GEN_BOXES_H

#include <opencv2/opencv.hpp>

enum Zone { TOP_LEFT, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT };

class Box {
  private:
    cv::Point barycenter;
    cv::Size boxSize;
    cv::Point2f velocity;
    cv::Point destination;
    Zone currentZone;
    Zone destinationZone;
    bool blocked;

  public:
    Box();
    Box(cv::Point barycenter, cv::Size boxSize);
    ~Box();

    void setCoordinates(cv::Point barycenter);
    cv::Point getCoordinates();
    cv::Size getBoxMeasures();
    void setDestination(cv::Point dest);
    cv::Point getTopLeft();
    cv::Point getBottomRight();
    void move();
    void setVelocity(cv::Point2f vel);
    cv::Point2f getVelocity();
    cv::Point getDestination();
    void setZone(Zone zone);
    Zone getZone();
    Zone getDestinationZone();
    void setDestinationZone(Zone destZone);
    bool boxInZone(Zone zone);
    bool isBlocked();
};

class BoxManager {
  private:
    std::vector<Box> boxes;
    cv::Size imageSize;
    cv::Mat canvas;
    int randomSeed;
    int refresh_rate;

  public:
    BoxManager();
    BoxManager(cv::Size imageSize, int randomSeed);
    ~BoxManager();
    cv::Point genRandomDestination();
    cv::Point genRandomDestination(const Zone zone);
    Box generateRandomBox();
    std::vector<Box> getBoxes();
    void addBox(Box box);
    void drawBox(Box &box, cv::Scalar color, bool draw, cv::Mat canvas);
    bool boxInCanvas(Box &box);
    void updateBoxPosition(Box &box);
    void setZone(Box &box);
    void show();
    void cleanCanvas();
    cv::Mat getCanvas();
    void setCanvas(cv::Mat canvas);
};

#endif