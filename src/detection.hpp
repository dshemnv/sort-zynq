#ifndef DETECTION_H
#define DETECTION_H
#include "genboxes.hpp"
#include <opencv2/opencv.hpp>

// TODO: Maybe use a simple array instead ?
struct detectionprops {
    int height;
    int width;
    cv::Point2f barycenter;
    const std::string label;
    float probability;
};

void printDetection(detectionprops det);
detectionprops detpropFromBox(Box &box, const char *label, float probability);

Box boxFromDetprop(detectionprops &detprop);

#endif