#ifndef DETECTION_H
#define DETECTION_H
#include "genboxes.hpp"
#include <opencv2/opencv.hpp>

typedef struct {
    int height;
    int width;
    cv::Point2f barycenter;
    const char *label;
    float probability;
} detectionprops;

void printDetection(detectionprops det);
detectionprops detpropFromBox(Box &box, const char *label, float probability);

Box boxFromDetprop(detectionprops &detprop);

#endif