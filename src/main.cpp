#include "detection.hpp"
#include "genboxes.hpp"
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

    Mat P = Mat::eye(Size(8, 8), CV_32F) * 80; // P

    predictor->init(P);
    predictor->load(config);
}

int main(int argc, char const *argv[]) {
    KalmanWrapper *pred_ptr;

    pred_ptr = new KalmanWrapper(8, 4, 0);
    ObjectHistory oh(5);
    kalmanInit(0.01, pred_ptr);

    srand(time(NULL));
    Size canvasSize = Size(640, 480);
    BoxManager boxm = BoxManager(canvasSize, 4);

    Box box1 = boxm.generateRandomBox();
    box1.setVelocity(Point2f(13, 13));

    while (true) {
        oh.add(detpropFromBox(box1, "label", 0.9));
        oh.update(pred_ptr);
        detectionprops det1_pred = oh.predict(pred_ptr);
        Box box1_pred            = boxFromDetprop(det1_pred);

        boxm.cleanCanvas();
        boxm.drawBox(box1, Scalar(0, 0, 255));
        boxm.drawBox(box1_pred, Scalar(0, 255, 0));
        boxm.show();
        char code = (char)waitKey(50);
        if (code == 'q' || code == 'Q' || code == 27) {
            break;
        }
    }

    delete pred_ptr;
    return 0;
}
