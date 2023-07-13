#include "detection.hpp"
#include "genboxes.hpp"
#include "kalman.hpp"
#include "objecthistory.hpp"
#include <cstdio>
#include <iostream>

void kalmanInit(float dt, KalmanOCV *predictor) {
    kalmanConfig config;

    config.F               = cv::Mat::eye(8, 8, CV_32F); // F
    config.F.at<float>(1)  = dt;
    config.F.at<float>(2)  = 0.5f * dt * dt;
    config.F.at<float>(10) = dt;
    config.F.at<float>(28) = dt;
    config.F.at<float>(29) = 0.5f * dt * dt;
    config.F.at<float>(37) = dt;

    config.Q = 0.09f * cv::Mat::eye(8, 8, CV_32F); // Q
    config.R = 0.1f * cv::Mat::eye(4, 4, CV_32F);  // R
    // clang-forcv::Mat off
    config.H = (cv::Mat_<float>(4, 8) << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1); // H
    // clang-forcv::Mat on

    cv::Mat P = cv::Mat::eye(8, 8, CV_32F) * 80; // P

    predictor->init(P);
    predictor->load(config);
}

int main(int argc, char const *argv[]) {
    KalmanOCV *pred_ptr;

    pred_ptr = new KalmanOCV(8, 4, 0);
    ObjectHistory oh(5);
    kalmanInit(0.01, pred_ptr);

    cv::Size canvasSize(900, 900);
    BoxManager boxm = BoxManager(canvasSize, 4);

    Box box1 = boxm.generateRandomBox();
    box1.setVelocity(cv::Point2f(1, 1));

    bool disapear = false;
    int duration  = 0;

    while (true) {
        if (std::rand() % 100 == 5) {
            disapear = true;
        }
        oh.add(detpropFromBox(box1, "label", 0.9));
        if (!disapear) {
            oh.update(pred_ptr);
        }
        detectionprops det1_pred = oh.predict(pred_ptr);
        Box box1_pred            = boxFromDetprop(det1_pred);

        boxm.cleanCanvas();
        cv::Mat normal_canvas = boxm.getCanvas().clone();
        cv::Size normal_size  = normal_canvas.size();
        cv::Mat result        = cv::Mat::zeros(
            cv::Size(normal_size.width * 2, normal_size.height), CV_8UC3);
        std::string message;
        if (disapear) {
            message = "Predicting";
            duration++;
        } else {
            message = "Updating";
        }
        cv::Size txt_size =
            cv::getTextSize(message, cv::FONT_HERSHEY_SIMPLEX, 1, 1, 0);
        putText(boxm.getCanvas(), message, cv::Point(0, 0 + txt_size.height),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);
        boxm.drawBox(box1, cv::Scalar(0, 0, 255), !disapear, boxm.getCanvas());
        boxm.drawBox(box1, cv::Scalar(0, 0, 255), true, normal_canvas);
        boxm.drawBox(box1_pred, cv::Scalar(0, 255, 0), disapear,
                     boxm.getCanvas());

        hconcat(boxm.getCanvas(), normal_canvas, result);

        boxm.setCanvas(result);

        boxm.show();
        char code = (char)cv::waitKey(10);
        if (code == 'q' || code == 'Q' || code == 27) {
            break;
        }
        if (duration >= std::rand() % 1000 + 1) {
            duration = 0;
            disapear = false;
        }
    }

    delete pred_ptr;
    return 0;
}
