#include "kalman.hpp"

KalmanWrapper::KalmanWrapper(int dynamParams, int measureParams,
                             int controlParams) {
    kf = KalmanFilter();
    kf.init(dynamParams, measureParams, controlParams);
}

KalmanWrapper::KalmanWrapper() {}

KalmanWrapper::~KalmanWrapper() {}

void KalmanWrapper::init(Mat initialEstimateUncertainty) {
    kf.errorCovPre = initialEstimateUncertainty;
}

void KalmanWrapper::load(kalmanConfig config) {
    config.F.copyTo(kf.transitionMatrix);
    config.Q.copyTo(kf.processNoiseCov);
    config.R.copyTo(kf.measurementNoiseCov);
    config.H.copyTo(kf.measurementMatrix);
}

kalmanConfig KalmanWrapper::dump() {
    kalmanConfig config;
    kf.transitionMatrix.copyTo(config.F);
    kf.processNoiseCov.copyTo(config.Q);
    kf.measurementNoiseCov.copyTo(config.R);
    kf.measurementMatrix.copyTo(config.H);
    return config;
}

const Mat &KalmanWrapper::predict() {
    std::cout << "Made a prediction" << std::endl;
    return kf.predict();
}

void KalmanWrapper::update(detectionprops det) {
    Mat measurement = (Mat_<float>(4, 1) << det.barycenter.x, det.barycenter.y,
                       det.height, det.width);
    kf.correct(measurement);
}

MatManager::MatManager(/* args */) {}

MatManager::~MatManager() {}