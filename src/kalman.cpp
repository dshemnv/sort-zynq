#include "kalman.hpp"
#ifdef KALMAN_ACCEL
#include "common/xf_headers.hpp"
#include "hls/kalman_hls_accel.hpp"
#include "xcl2.hpp"
#endif
#include "utils.hpp"

KalmanOCV::KalmanOCV(int dynamParams, int measureParams, int controlParams) {
    kf = cv::KalmanFilter();
    kf.init(dynamParams, measureParams, controlParams);
}

KalmanOCV::KalmanOCV() {}

KalmanOCV::~KalmanOCV() {}

void KalmanOCV::init(cv::Mat initialEstimateUncertainty) {
    kf.errorCovPre = initialEstimateUncertainty;
}

void KalmanOCV::load(kalmanConfig config) {
    config.F.copyTo(kf.transitionMatrix);
    config.Q.copyTo(kf.processNoiseCov);
    config.R.copyTo(kf.measurementNoiseCov);
    config.H.copyTo(kf.measurementMatrix);
}

kalmanConfig KalmanOCV::dump() {
    kalmanConfig config;
    kf.transitionMatrix.copyTo(config.F);
    kf.processNoiseCov.copyTo(config.Q);
    kf.measurementNoiseCov.copyTo(config.R);
    kf.measurementMatrix.copyTo(config.H);
    return config;
}

const cv::Mat &KalmanOCV::predict() {
    // std::cout << "Made a prediction" << std::endl;
    return kf.predict();
}

void KalmanOCV::update(detectionprops det) {
    cv::Mat measurement = (cv::Mat_<float>(4, 1) << det.barycenter.x,
                           det.barycenter.y, det.height, det.width);
    kf.correct(measurement);
}

#ifdef KALMAN_ACCEL
KalmanHLS::KalmanHLS() {}
KalmanHLS::KalmanHLS(int dynamParams, int measureParams, int controlParams) {}
KalmanHLS::~KalmanHLS() {}

void KalmanHLS::init(cv::Mat initialEstimateUncertainty) {

    cv::Mat qu(initialEstimateUncertainty.size(), CV_32F);
    cv::Mat qd(initialEstimateUncertainty.size(), CV_32F);
    decomposeInUDU(initialEstimateUncertainty, &qu, &qd);

    U0.cv_mat   = qu;
    U0.size     = qu.cols * qu.rows * sizeof(float);
    U0.data_ptr = (float *)malloc(U0.size);

    D0.cv_mat   = qd.diag(0);
    D0.size     = qd.cols * sizeof(float);
    D0.data_ptr = (float *)malloc(D0.size);

    X0.cv_mat   = cv::Mat::zeros(KF_N, 1, CV_32F);
    X0.size     = X0.cv_mat.rows * sizeof(float);
    X0.data_ptr = (float *)malloc(X0.size);
}

void KalmanHLS::init_accelerator(std::vector<cl::Device> &devices,
                                 cl::Context &context, cl::CommandQueue &queue,
                                 cl::Event &event) {

    LOG_INFO("Accelerator initialization");
    cl::Device device = devices[0];

    OCL_CHECK(err,
              std::string deviceName = device.getInfo<CL_DEVICE_NAME>(&err));

    LOG_INFO("Device name: " << deviceName);

    std::string binaryFile =
        xcl::find_binary_file(deviceName, "krnl_kalmanfilter");
    cl::Program::Binaries bins = xcl::import_binary_file(binaryFile);

    devices.resize(1);
    OCL_CHECK(err, cl::Program program(context, devices, bins, NULL, &err));

    OCL_CHECK(err, kernel = cl::Kernel(program, "kalmanfilter_accel", &err));

    LOG_INFO("Allocating buffers");
    allocateBuffers(context);

    LOG_INFO("Setting Accelerator arguments");
    setKernelArgs();

    LOG_INFO("Setup initial data to device transfer");
    copyDataToDevice(queue, event);

    LOG_INFO("Execute accelerator initialization");
    executeKernel(queue, INIT_EN);
}

void KalmanHLS::load(kalmanConfig config) {
    config.F.copyTo(A.cv_mat);
    config.H.copyTo(H.cv_mat);
    config.R.copyTo(R.cv_mat);

    cv::Mat uq(config.Q.size(), CV_32F);
    cv::Mat dq(config.Q.size(), CV_32F);
    decomposeInUDU(config.Q, &uq, &dq);

    Uq.cv_mat   = uq;
    Uq.size     = uq.cols * uq.rows * sizeof(float);
    Uq.data_ptr = (float *)malloc(Uq.size);

    Dq.cv_mat   = dq;
    Dq.size     = dq.cols * sizeof(float);
    Dq.data_ptr = (float *)malloc(Dq.size);

    y.cv_mat   = cv::Mat::zeros(KF_M, 1, CV_32F);
    y.size     = KF_M * sizeof(float);
    y.data_ptr = (float *)malloc(y.size);
}

void KalmanHLS::allocateBuffers(cl::Context &context) {
    OCL_CHECK(err, A.ocl_buffer = cl::Buffer(context, CL_MEM_READ_ONLY, A.size,
                                             NULL, &err));
    OCL_CHECK(err, Uq.ocl_buffer = cl::Buffer(context, CL_MEM_READ_ONLY,
                                              Uq.size, NULL, &err));
    OCL_CHECK(err, Dq.ocl_buffer = cl::Buffer(context, CL_MEM_READ_ONLY,
                                              Dq.size, NULL, &err));
    OCL_CHECK(err, U0.ocl_buffer = cl::Buffer(context, CL_MEM_READ_ONLY,
                                              U0.size, NULL, &err));
    OCL_CHECK(err, D0.ocl_buffer = cl::Buffer(context, CL_MEM_READ_ONLY,
                                              D0.size, NULL, &err));
    OCL_CHECK(err, X0.ocl_buffer = cl::Buffer(context, CL_MEM_READ_ONLY,
                                              X0.size, NULL, &err));
    OCL_CHECK(err, H.ocl_buffer = cl::Buffer(context, CL_MEM_READ_ONLY, H.size,
                                             NULL, &err));
    OCL_CHECK(err, R.ocl_buffer = cl::Buffer(context, CL_MEM_READ_ONLY, R.size,
                                             NULL, &err));
    OCL_CHECK(err, y.ocl_buffer = cl::Buffer(context, CL_MEM_READ_ONLY, y.size,
                                             NULL, &err));

    OCL_CHECK(err, outX.ocl_buffer = cl::Buffer(context, CL_MEM_WRITE_ONLY,
                                                outX.size, NULL, &err));
    OCL_CHECK(err, outU.ocl_buffer = cl::Buffer(context, CL_MEM_WRITE_ONLY,
                                                outU.size, NULL, &err));
    OCL_CHECK(err, outD.ocl_buffer = cl::Buffer(context, CL_MEM_WRITE_ONLY,
                                                outD.size, NULL, &err));
}

void KalmanHLS::setKernelArgs() {
    OCL_CHECK(err, err = kernel.setArg(0, A.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(1, Uq.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(2, Dq.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(3, U0.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(4, D0.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(5, X0.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(6, H.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(7, R.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(8, y.ocl_buffer));

    OCL_CHECK(err, err = kernel.setArg(10, outX.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(11, outD.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(12, outU.ocl_buffer));
}

void KalmanHLS::copyDataToDevice(cl::CommandQueue &queue, cl::Event &event) {
    A.extactData(kalmanBuf::DATA_PTR);
    Uq.extactData(kalmanBuf::DATA_PTR);
    Dq.extactData(kalmanBuf::DATA_PTR);
    U0.extactData(kalmanBuf::DATA_PTR);
    D0.extactData(kalmanBuf::DATA_PTR);
    X0.extactData(kalmanBuf::DATA_PTR);
    H.extactData(kalmanBuf::DATA_PTR);
    R.extactData(kalmanBuf::DATA_PTR);
    y.extactData(kalmanBuf::DATA_PTR);

    OCL_CHECK(err, queue.enqueueWriteBuffer(A.ocl_buffer, CL_TRUE, 0, A.size,
                                            A.data_ptr, nullptr, &event));
    OCL_CHECK(err, queue.enqueueWriteBuffer(Uq.ocl_buffer, CL_TRUE, 0, Uq.size,
                                            Uq.data_ptr, nullptr, &event));
    OCL_CHECK(err, queue.enqueueWriteBuffer(Dq.ocl_buffer, CL_TRUE, 0, Dq.size,
                                            Dq.data_ptr, nullptr, &event));
    OCL_CHECK(err, queue.enqueueWriteBuffer(U0.ocl_buffer, CL_TRUE, 0, U0.size,
                                            U0.data_ptr, nullptr, &event));
    OCL_CHECK(err, queue.enqueueWriteBuffer(D0.ocl_buffer, CL_TRUE, 0, D0.size,
                                            D0.data_ptr, nullptr, &event));
    OCL_CHECK(err, queue.enqueueWriteBuffer(X0.ocl_buffer, CL_TRUE, 0, X0.size,
                                            X0.data_ptr, nullptr, &event));
    OCL_CHECK(err, queue.enqueueWriteBuffer(H.ocl_buffer, CL_TRUE, 0, H.size,
                                            H.data_ptr, nullptr, &event));
    OCL_CHECK(err, queue.enqueueWriteBuffer(R.ocl_buffer, CL_TRUE, 0, R.size,
                                            R.data_ptr, nullptr, &event));
    OCL_CHECK(err, queue.enqueueWriteBuffer(y.ocl_buffer, CL_TRUE, 0, y.size,
                                            y.data_ptr, nullptr, &event));
}

void KalmanHLS::copyDataToHost(cl::CommandQueue &queue, cl::Event &event) {
    OCL_CHECK(err,
              queue.enqueueReadBuffer(outX.ocl_buffer, CL_TRUE, 0, outX.size,
                                      outX.data_ptr, nullptr, &event));
    OCL_CHECK(err,
              queue.enqueueReadBuffer(outU.ocl_buffer, CL_TRUE, 0, outU.size,
                                      outU.data_ptr, nullptr, &event));

    OCL_CHECK(err,
              queue.enqueueReadBuffer(outD.ocl_buffer, CL_TRUE, 0, outD.size,
                                      outD.data_ptr, nullptr, &event));

    outX.extactData(kalmanBuf::CV_MAT);
    outU.extactData(kalmanBuf::CV_MAT);
    outD.extactData(kalmanBuf::CV_MAT);
}

void KalmanHLS::executeKernel(cl::CommandQueue &queue,
                              const controlFlag &flag) {
    OCL_CHECK(err, kernel.setArg(9, (unsigned char)flag));
    OCL_CHECK(err, err = queue.enqueueTask(kernel));
}

const cv::Mat &KalmanHLS::predict() {}

void KalmanHLS::update(detectionprops detection) {}

kalmanConfig KalmanHLS::dump() {}
#endif
MatManager::MatManager(/* args */) {}
MatManager::~MatManager() {}