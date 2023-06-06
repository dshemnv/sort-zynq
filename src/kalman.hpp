#ifndef KALMAN_HPP
#define KALMAN_HPP
#include "detection.hpp"
#include <iostream>
#include <opencv2/video/tracking.hpp>

using namespace cv;

typedef struct {
    Mat F; // State transition matrix
    Mat H; // Observation matrix
    Mat Q; // Process noise uncertainty
    Mat P; // Estimate uncertainty
    Mat R; // Measurment uncertainty
} kalmanConfig;

class KalmanWrapper {
  private:
    KalmanFilter kf;

  public:
    KalmanWrapper(int dynamParams, int measureParams, int controlParams);
    KalmanWrapper();
    const Mat &predict();
    void update(detectionprops detection);
    ~KalmanWrapper();
    void load(kalmanConfig config);
    void init(Mat initialEstimateUncertainty);
    kalmanConfig dump();
};

class MatManager {
  private:
    /* data */
  public:
    MatManager(/* args */);
    ~MatManager();
};

#endif