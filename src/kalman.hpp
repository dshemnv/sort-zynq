#ifndef KALMAN_HPP
#define KALMAN_HPP
#include "detection.hpp"
#include "utils.hpp"
#ifdef KALMAN_ACCEL
#include "kalman_hls_accel.hpp"
#include "xcl2.hpp"
#include "xf_kalmanfilter.hpp"
#endif
#include <iostream>
#include <opencv2/video/tracking.hpp>

struct kalmanConfig {
    cv::Mat F; // State transition matrix
    cv::Mat H; // Observation matrix
    cv::Mat Q; // Process noise uncertainty
    cv::Mat P; // Estimate uncertainty
    cv::Mat R; // Measurment uncertainty
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
    float *data_ptr;
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
    virtual void update(detectionprops detection)         = 0;
    virtual void load(kalmanConfig config)                = 0;
    virtual void init(cv::Mat initialEstimateUncertainty) = 0;
    virtual kalmanConfig dump()                           = 0;
    kalmanParams getParams();
    void setParams(kalmanParams params);
};

class KalmanOCV : public KalmanBase {
  private:
    cv::KalmanFilter kf;

  public:
    KalmanOCV(kalmanParams params);
    KalmanOCV();
    ~KalmanOCV();
    const cv::Mat &predict();
    void update(detectionprops detection);
    void load(kalmanConfig config);
    void init(cv::Mat initialEstimateUncertainty);
    kalmanConfig dump();
};

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