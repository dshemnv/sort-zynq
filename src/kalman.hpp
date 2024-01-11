#ifndef KALMAN_HPP
#define KALMAN_HPP
#include "utils.hpp"
#ifndef KF_N
#define KF_N 7
#endif
#ifndef KF_M
#define KF_M 4
#endif
#if __INTELLISENSE__ // This is to fix VSCode intellisense
#undef __ARM_NEON
#undef __ARM_NEON__
#endif
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/video/tracking.hpp>

struct kalmanConfig {
    cv::Mat F; // State transition matrix
    cv::Mat H; // Observation matrix
    cv::Mat Q; // Process noise uncertainty
    cv::Mat P; // Estimate uncertainty
    cv::Mat R; // Measurment uncertainty

    void printConfig() {
        std::cout << "F=" << std::endl;
        std::cout << F << std::endl;
        std::cout << "H=" << std::endl;
        std::cout << H << std::endl;
        std::cout << "Q=" << std::endl;
        std::cout << Q << std::endl;
        std::cout << "P=" << std::endl;
        std::cout << P << std::endl;
        std::cout << "R=" << std::endl;
        std::cout << R << std::endl;
    }
};

class KalmanBase {
  public:
    virtual const cv::Mat &predict()                      = 0;
    virtual void update(const cv::Mat &meas)              = 0;
    virtual void load(kalmanConfig config)                = 0;
    virtual void init(cv::Mat initialEstimateUncertainty) = 0;
    virtual kalmanConfig dump()                           = 0;
    virtual void setState(const cv::Mat &newState)        = 0;
    virtual const cv::Mat &getState()                     = 0;
};

template <size_t N_STATES, size_t N_MEAS> class KalmanOCV : public KalmanBase {
  private:
    cv::KalmanFilter kf;

  public:
    KalmanOCV();
    ~KalmanOCV();
    const cv::Mat &predict();
    void update(const cv::Mat &meas);
    void load(kalmanConfig config);
    void init(cv::Mat initialEstimateUncertainty);
    const cv::Mat &getState();
    void setState(const cv::Mat &newState);
    kalmanConfig dump();
};
template <size_t N_STATES, size_t N_MEAS>
KalmanOCV<N_STATES, N_MEAS>::KalmanOCV() {
    kf = cv::KalmanFilter();
    kf.init(N_STATES, N_MEAS, 0);
}

template <size_t N_STATES, size_t N_MEAS>
KalmanOCV<N_STATES, N_MEAS>::~KalmanOCV() {}

template <size_t N_STATES, size_t N_MEAS>
void KalmanOCV<N_STATES, N_MEAS>::init(cv::Mat initialEstimateUncertainty) {
    kf.errorCovPre = initialEstimateUncertainty;
}

template <size_t N_STATES, size_t N_MEAS>
const cv::Mat &KalmanOCV<N_STATES, N_MEAS>::getState() {
    return kf.statePost;
}

template <size_t N_STATES, size_t N_MEAS>
void KalmanOCV<N_STATES, N_MEAS>::setState(const cv::Mat &new_state) {
    kf.statePost = new_state;
}

template <size_t N_STATES, size_t N_MEAS>
void KalmanOCV<N_STATES, N_MEAS>::load(kalmanConfig config) {
    config.F.copyTo(kf.transitionMatrix);
    config.Q.copyTo(kf.processNoiseCov);
    config.R.copyTo(kf.measurementNoiseCov);
    config.H.copyTo(kf.measurementMatrix);
    config.P.copyTo(kf.errorCovPost);
}

template <size_t N_STATES, size_t N_MEAS>
kalmanConfig KalmanOCV<N_STATES, N_MEAS>::dump() {
    kalmanConfig config;
    kf.transitionMatrix.copyTo(config.F);
    kf.processNoiseCov.copyTo(config.Q);
    kf.measurementNoiseCov.copyTo(config.R);
    kf.measurementMatrix.copyTo(config.H);
    return config;
}

template <size_t N_STATES, size_t N_MEAS>
const cv::Mat &KalmanOCV<N_STATES, N_MEAS>::predict() {
    // std::cout << "Made a prediction" << std::endl;
    return kf.predict();
}

template <size_t N_STATES, size_t N_MEAS>
void KalmanOCV<N_STATES, N_MEAS>::update(const cv::Mat &meas) {
    kf.correct(meas);
}

template <size_t N_STATES, size_t N_MEAS>
class KalmanEigen : public KalmanBase {
  private:
    Eigen::Matrix<double, N_STATES, N_STATES> F;
    Eigen::Matrix<double, N_MEAS, N_STATES> H;
    Eigen::Matrix<double, N_STATES, N_STATES> Q;
    Eigen::Matrix<double, N_MEAS, N_MEAS> R;
    Eigen::Matrix<double, N_STATES, N_STATES> P;
    Eigen::Vector<double, N_STATES> X;
    cv::Mat output;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KalmanEigen();
    ~KalmanEigen();
    Eigen::Matrix<double, N_STATES, N_MEAS> K();
    const cv::Mat &predict();
    void update(const cv::Mat &meas);
    void load(kalmanConfig config);
    void init(cv::Mat initialEstimateUncertainty);
    const cv::Mat &getState();
    void setState(const cv::Mat &newState);
    kalmanConfig dump();
};

template <size_t N_STATES, size_t N_MEAS>
KalmanEigen<N_STATES, N_MEAS>::KalmanEigen() {
    X = Eigen::Vector<double, N_STATES>::Zero();
    F = Eigen::Matrix<double, N_STATES, N_STATES>::Identity();
    H = Eigen::Matrix<double, N_MEAS, N_STATES>::Zero();
    Q = Eigen::Matrix<double, N_STATES, N_STATES>::Identity();
    R = Eigen::Matrix<double, N_MEAS, N_MEAS>::Identity();
    P = Eigen::Matrix<double, N_STATES, N_STATES>::Zero();
    // std::cout << "Initialized Kalman Tracker" << std::endl;
}

template <size_t N_STATES, size_t N_MEAS>
KalmanEigen<N_STATES, N_MEAS>::~KalmanEigen() {}

template <size_t N_STATES, size_t N_MEAS>
Eigen::Matrix<double, N_STATES, N_MEAS> KalmanEigen<N_STATES, N_MEAS>::K() {
    Eigen::MatrixXd tmp1 = H * P * H.transpose() + R;
    Eigen::MatrixXd gain = P * H.transpose() * tmp1.inverse();
    return gain;
}

template <size_t N_STATES, size_t N_MEAS>
const cv::Mat &KalmanEigen<N_STATES, N_MEAS>::getState() {
    cv::eigen2cv<double>(X, output);
    return output;
}

template <size_t N_STATES, size_t N_MEAS>
void KalmanEigen<N_STATES, N_MEAS>::setState(const cv::Mat &newState) {
    assert((newState.cols == 1) && (newState.rows == KF_N));
    cv::cv2eigen(newState, X);
}

template <size_t N_STATES, size_t N_MEAS>
const cv::Mat &KalmanEigen<N_STATES, N_MEAS>::predict() {

    X = F * X;
    P = F * P * F.transpose() + Q;

    cv::eigen2cv<double>(X, output);
    return output;
}

template <size_t N_STATES, size_t N_MEAS>
void KalmanEigen<N_STATES, N_MEAS>::update(const cv::Mat &meas) {
    assert((meas.cols == 1) && (meas.rows == 4));
    Eigen::MatrixXd measure;
    cv::cv2eigen<double>(meas, measure);

    Eigen::MatrixXd gain = K();
    Eigen::MatrixXd diag = Eigen::MatrixXd::Identity(N_STATES, N_STATES);

    Eigen::Matrix<double, N_STATES, N_STATES> U = diag - gain * H;

    X = X + gain * (measure - H * X);
    P = U * P * U.transpose() + gain * R * gain.transpose();
}

template <size_t N_STATES, size_t N_MEAS>
void KalmanEigen<N_STATES, N_MEAS>::load(kalmanConfig config) {
    cv::cv2eigen<double>(config.F, F);
    cv::cv2eigen<double>(config.H, H);
    cv::cv2eigen<double>(config.Q, Q);
    cv::cv2eigen<double>(config.R, R);
    cv::cv2eigen<double>(config.P, P);
}

template <size_t N_STATES, size_t N_MEAS>
void KalmanEigen<N_STATES, N_MEAS>::init(cv::Mat initialEstimateUncertainty) {
    X = Eigen::VectorXd::Zero(N_STATES);
    cv::cv2eigen<double>(initialEstimateUncertainty, P);
}

template <size_t N_STATES, size_t N_MEAS>
kalmanConfig KalmanEigen<N_STATES, N_MEAS>::dump() {
    kalmanConfig config;
    cv::eigen2cv<double>(F, config.F);
    cv::eigen2cv<double>(H, config.H);
    cv::eigen2cv<double>(Q, config.Q);
    cv::eigen2cv<double>(R, config.R);
    cv::eigen2cv<double>(P, config.P);
    return config;
}

class KalmanCreator {
  public:
    virtual KalmanBase *create() = 0;
    virtual ~KalmanCreator(){};
};

template <typename KALMAN_TYPE>
class KalmanCreatorFactory : public KalmanCreator {
  public:
    KalmanBase *create() override { return new KALMAN_TYPE(); }
};

#endif