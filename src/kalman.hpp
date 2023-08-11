#ifndef KALMAN_HPP
#define KALMAN_HPP
#include "detection.hpp"
#include "utils.hpp"
#ifdef KALMAN_ACCEL
#include "kalman_hls_accel.hpp"
#include "xcl2.hpp"
#include "xf_kalmanfilter.hpp"
#endif
#ifndef KF_N
#define KF_N 7
#endif
#ifndef KF_M
#define KF_M 4
#endif
#if __INTELLISENSE__
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

struct kalmanParams {
    int dynamParams;
    int measureParams;
    int controlParams;
};

#ifdef KALMAN_ACCEL
struct kalmanBuf {
    enum destination { CV_MAT, DATA_PTR };
    cv::Mat cv_mat;
    size_t size;
    cl::Buffer ocl_buffer;
    double *data_ptr;
    void extactData(destination dest) {
        if (dest == DATA_PTR) {
            mat2FloatPtr(&cv_mat, data_ptr);
        } else if (dest == CV_MAT) {
            floatPtr2Mat(&cv_mat, data_ptr);
        };
    };
};
#endif

class KalmanBase {
  private:
    kalmanParams params;

  public:
    virtual const cv::Mat &predict()                      = 0;
    virtual void update(const cv::Mat &meas)              = 0;
    virtual void load(kalmanConfig config)                = 0;
    virtual void init(cv::Mat initialEstimateUncertainty) = 0;
    virtual kalmanConfig dump()                           = 0;
    virtual void setState(const cv::Mat &newState)        = 0;
    virtual const cv::Mat &getState()                     = 0;
    kalmanParams getParams();
    void setParams(kalmanParams params);
};

class KalmanOCV : public KalmanBase {
  private:
    cv::KalmanFilter kf;

  public:
    KalmanOCV(kalmanParams params);
    KalmanOCV(int dynamParams, int measureParams, int controlParams);
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
    KalmanEigen(kalmanParams params){};
    KalmanEigen(){};
    ~KalmanEigen(){};
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
Eigen::Matrix<double, N_STATES, N_MEAS> KalmanEigen<N_STATES, N_MEAS>::K() {
    Eigen::MatrixXd tmp1 = H * P * H.transpose() + R;
    Eigen::MatrixXd gain = P * H.transpose() * tmp1.inverse();
    return gain;
}

template <size_t N_STATES, size_t N_MEAS>
const cv::Mat &KalmanEigen<N_STATES, N_MEAS>::getState() {
    cv::Mat state;
    cv::eigen2cv<double>(X, state);
    return state;
}

template <size_t N_STATES, size_t N_MEAS>
void KalmanEigen<N_STATES, N_MEAS>::setState(const cv::Mat &newState) {
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
    Eigen::MatrixXd measure;
    cv::cv2eigen<double>(meas, measure);

    Eigen::MatrixXd gain = K();
    Eigen::MatrixXd diag = Eigen::MatrixXd::Identity(N_STATES, N_STATES);

    Eigen::Matrix<double, N_STATES, N_STATES> U = diag - gain * H;

    X = X + gain * (meas - H * X);
    P = U * P * U.transpose() + gain * R * gain.transpose();
}

template <size_t N_STATES, size_t N_MEAS>
void KalmanEigen<N_STATES, N_MEAS>::load(kalmanConfig config) {
    cv::cv2eigen<double>(config.F, F);
    cv::cv2eigen<double>(config.H, H);
    cv::cv2eigen<double>(config.Q, Q);
    cv::cv2eigen<double>(config.R, R);
}

template <size_t N_STATES, size_t N_MEAS>
void KalmanEigen<N_STATES, N_MEAS>::init(cv::Mat initialEstimateUncertainty) {
    X = Eigen::VectorXf::Zero(N_STATES);
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
    virtual ~KalmanCreator(){};
    virtual KalmanBase *create(int dynamParams, int measureParams,
                               int controlParams = 0) const = 0;
};

class KalmanOCVCreator : public KalmanCreator {
  public:
    KalmanBase *create(int dynamParams, int measureParams,
                       int controlParams = 0) const override {
        return new KalmanOCV(dynamParams, measureParams, controlParams);
    };
    ~KalmanOCVCreator(){};
};

template <size_t N_STATES, size_t N_MEAS>
class KalmanEigenCreator : public KalmanCreator {
  public:
    KalmanBase *create(int dynamStates, int measureStates) const override;
    ~KalmanEigenCreator(){};
};

template <size_t N_STATES, size_t N_MEAS>
inline KalmanBase *
KalmanEigenCreator<N_STATES, N_MEAS>::create(int dynamStates,
                                             int measureStates) const {
    return new KalmanEigen<N_STATES, N_MEAS>();
}

#ifdef KALMAN_ACCEL
class KalmanHLS : public KalmanBase {
  private:
    kalmanBuf A;  // Transition matrix
    kalmanBuf Uq; // Process Noice Covariance Matrix, U part
    kalmanBuf Dq; // Process Noise Covariance Matrix, D part
    kalmanBuf U0; // Process Noise Covariance MAtrix, U part, initial value
    kalmanBuf D0; // Process Noise Covariance Matrix, D part, initial value
    kalmanBuf X0; // State Matrix, initial value
    kalmanBuf H;  // Measurement Matrix
    kalmanBuf R;  // Measurement Noise Covariance Matrix
    kalmanBuf y;  // Measurement vector

    kalmanBuf outX; // Output State Matrix
    kalmanBuf outU; // Output Error Estimate Covariance matrix, U part
    kalmanBuf outD; // Output Error Estimate Covariance matrix, D part

    cl::Kernel kernel;
    cl::CommandQueue *queuePtr;
    cl::Event *eventPtr;

  public:
    cl_int err;
    KalmanHLS();
    KalmanHLS(kalmanParams params);
    ~KalmanHLS();
    void init_accelerator(std::vector<cl::Device> &devices,
                          cl::Context &context, cl::CommandQueue &queue,
                          cl::Event &event);
    void allocateBuffers(cl::Context &context);
    void setKernelArgs();
    void copyDataToDevice(cl::CommandQueue &queue, cl::Event &event);
    void copyDataToHost(cl::CommandQueue &queue, cl::Event &event);
    const cv::Mat &predict();
    void update(detectionprops detection);
    void load(kalmanConfig config);
    void init(cv::Mat initialEstimateUncertainty);
    void executeKernel(cl::CommandQueue &queue, const int &flag);
    void printOutput();
    void finish();
    kalmanConfig dump();
};
#endif

class MatManager {
  private:
    /* data */
  public:
    MatManager(/* args */);
    ~MatManager();
};

#endif
