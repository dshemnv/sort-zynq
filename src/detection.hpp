#ifndef DETECTION_H
#define DETECTION_H
#include <opencv2/opencv.hpp>

typedef struct {
    float height;
    float width;
    cv::Point2f barycenter;
    const char *label;
    float probability;
} detectionprops;

void printDetection(detectionprops det);

#endif