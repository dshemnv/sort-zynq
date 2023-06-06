#include "detection.hpp"
#include "kalman.hpp"
#include "objecthistory.hpp"
#include <cstdio>
#include <iostream>
using namespace cv;

void kalmanInit(float dt, KalmanWrapper *predictor) {
    kalmanConfig config;

    config.F               = Mat::eye(Size(8, 8), CV_32F); // F
    config.F.at<float>(1)  = dt;
    config.F.at<float>(2)  = 0.5f * dt * dt;
    config.F.at<float>(10) = dt;
    config.F.at<float>(28) = dt;
    config.F.at<float>(29) = 0.5f * dt * dt;
    config.F.at<float>(37) = dt;

    config.Q = 0.09f * Mat::eye(Size(8, 8), CV_32F); // Q
    config.R = 0.1f * Mat::eye(Size(4, 4), CV_32F);  // R
    // clang-format off
    config.H = (Mat_<float>(4, 8) << 1, 0, 0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 1, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0, 1, 0,
                                    0, 0, 0, 0, 0, 0, 0, 1); // H
    // clang-format on

    Mat P = Mat::eye(Size(8, 8), CV_32F) * 80;

    predictor->init(P);
    predictor->load(config);
}

int main(int argc, char const *argv[]) {
    KalmanWrapper *pred_ptr;
    ObjectHistory *oh;
    detectionprops det1 = {
        2, 4, Point_<float>(2.1f, 2.3f), "label1", 0.87,
    };
    detectionprops det2 = {
        5, 10, Point_<float>(5.9f, 2.3f), "label2", 0.63,
    };
    pred_ptr = new KalmanWrapper(8, 4, 0);
    oh       = new ObjectHistory(5);
    kalmanInit(0.01, pred_ptr);
    oh->add(det1);
    oh->add(det2);
    oh->showHistory();

    oh->update(pred_ptr);
    detectionprops out = oh->predict(pred_ptr);
    printDetection(out);

    delete oh;
    delete pred_ptr;
    return 0;
}
