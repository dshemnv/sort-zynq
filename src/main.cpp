#include <stdio.h>
#include <iostream>
#include "kalman.hpp"
#include "objecthistory.hpp"
using namespace cv;

KalmanWrapper kalmanInit(float dt, int dynamParams, int measuredParams)
{
    KalmanWrapper predictor(dynamParams, measuredParams, 0);
    kalmanConfig config;

    config.F = Mat::eye(Size(8, 8), CV_32F); // F
    config.F.at<float>(1) = dt;
    config.F.at<float>(2) = 0.5f * dt * dt;
    config.F.at<float>(10) = dt;
    config.F.at<float>(28) = dt;
    config.F.at<float>(29) = 0.5f * dt * dt;
    config.F.at<float>(37) = dt;

    config.R = 0.09f * Mat::eye(Size(8, 8), CV_32F); // Q
    config.R = 0.1f * Mat::eye(Size(8, 8), CV_32F);  // R

    config.H = (Mat_<float>(4, 8) << 1, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 0, 0, 1); // H

    Mat P = Mat::eye(Size(8, 8), CV_32F) * 80;

    predictor.init(P);
    predictor.load(config);
    return predictor;
}

int main(int argc, char const *argv[])
{
    KalmanWrapper predictor = kalmanInit(0.01, 8, 4);

    KalmanWrapper *pred_ptr;
    ObjectHistory *oh;

    pred_ptr = new KalmanWrapper(8, 4, 0);
    oh = new ObjectHistory(5);

    oh->predict(pred_ptr);

    return 0;
}
