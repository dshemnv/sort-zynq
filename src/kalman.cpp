#include "kalman.hpp"
#ifdef KALMAN_ACCEL
#include "common/xf_headers.hpp"
#include "kalman_hls_accel.hpp"
#include "xcl2.hpp"
#endif
#include "utils.hpp"

kalmanParams KalmanBase::getParams() { return params; }
void KalmanBase::setParams(kalmanParams params) { this->params = params; }

KalmanOCV::KalmanOCV(kalmanParams params) {
    kf = cv::KalmanFilter();
    kf.init(params.dynamParams, params.measureParams, params.controlParams);
    setParams(params);
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
KalmanHLS::KalmanHLS(kalmanParams params) { setParams(params); }
KalmanHLS::~KalmanHLS() {
    free(A.data_ptr);
    LOG_INFO("A freed");
    free(Uq.data_ptr);
    LOG_INFO("Uq freed");
    free(Dq.data_ptr);
    LOG_INFO("Dq freed");
    free(U0.data_ptr);
    LOG_INFO("U0 freed");
    free(D0.data_ptr);
    LOG_INFO("D0 freed");
    free(X0.data_ptr);
    LOG_INFO("X0 freed");
    free(H.data_ptr);
    LOG_INFO("H freed");
    free(R.data_ptr);
    LOG_INFO("R freed");
    free(y.data_ptr);
    LOG_INFO("y freed");
    free(outX.data_ptr);
    LOG_INFO("outX freed");
    free(outD.data_ptr);
    LOG_INFO("outD freed");
    free(outU.data_ptr);
    LOG_INFO("outU freed");
}

void KalmanHLS::init(cv::Mat initialEstimateUncertainty) {

    cv::Mat qu(initialEstimateUncertainty.size(), CV_32F);
    cv::Mat qd(initialEstimateUncertainty.size(), CV_32F);
    decomposeInUDU(initialEstimateUncertainty, &qu, &qd);

    U0.cv_mat   = qu;
    U0.size     = qu.cols * qu.rows * sizeof(float);
    U0.data_ptr = (float *)malloc(U0.size);

    D0.cv_mat   = qd.diag(0);
    D0.size     = qd.rows * sizeof(float);
    D0.data_ptr = (float *)malloc(D0.size);
    LOG_INFO("Size of D is " << D0.size);
    LOG_INFO("With " << D0.cv_mat.rows << " rows and " << D0.cv_mat.cols
                     << " cols.");
    X0.cv_mat   = cv::Mat::zeros(KF_N, 1, CV_32F);
    X0.size     = X0.cv_mat.rows * sizeof(float);
    X0.data_ptr = (float *)malloc(X0.size);

    outU.size     = U0.size;
    outU.data_ptr = (float *)malloc(outU.size);

    outX.size     = X0.size;
    outX.data_ptr = (float *)malloc(outX.size);

    outD.size     = D0.size;
    outD.data_ptr = (float *)malloc(outD.size);
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
        xcl::find_binary_file(deviceName, "krnl_kalmanfilter_pkg");
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
    executeKernel(queue, 103);

    LOG_INFO("Init output vectors");
    outX.cv_mat = cv::Mat::zeros(KF_N, 1, CV_32F);
    outD.cv_mat = cv::Mat::zeros(KF_N, 1, CV_32F);
    outU.cv_mat = cv::Mat::zeros(KF_N, KF_M, CV_32F);

    copyDataToHost(queue, event);

    LOG_INFO("Successfully initialized accelerator");
}

void KalmanHLS::load(kalmanConfig config) {
    config.F.copyTo(A.cv_mat);
    A.size     = config.F.cols * config.F.rows * sizeof(float);
    A.data_ptr = (float *)malloc(A.size);

    config.H.copyTo(H.cv_mat);
    H.size     = config.H.cols * config.H.rows * sizeof(float);
    H.data_ptr = (float *)malloc(H.size);

    config.R.copyTo(R.cv_mat);
    R.size     = config.R.cols * config.R.rows * sizeof(float);
    R.data_ptr = (float *)malloc(R.size);

    cv::Mat uq(config.Q.size(), CV_32F);
    cv::Mat dq(config.Q.size(), CV_32F);
    decomposeInUDU(config.Q, &uq, &dq);

    Uq.cv_mat   = uq;
    Uq.size     = uq.cols * uq.rows * sizeof(float);
    Uq.data_ptr = (float *)malloc(Uq.size);

    Dq.cv_mat   = dq.diag(0);
    Dq.size     = dq.rows * sizeof(float);
    Dq.data_ptr = (float *)malloc(Dq.size);

    y.cv_mat   = cv::Mat::zeros(KF_M, 1, CV_32F);
    y.size     = KF_M * sizeof(float);
    y.data_ptr = (float *)malloc(y.size);
}

void KalmanHLS::allocateBuffers(cl::Context &context) {
    OCL_CHECK(
        err, cl::Buffer clBuffA(context, CL_MEM_READ_ONLY, A.size, NULL, &err));
    A.ocl_buffer = clBuffA;

    OCL_CHECK(err, cl::Buffer clBuffUq(context, CL_MEM_READ_ONLY, Uq.size, NULL,
                                       &err));
    Uq.ocl_buffer = clBuffUq;

    OCL_CHECK(err, cl::Buffer clBuffDq(context, CL_MEM_READ_ONLY, Dq.size, NULL,
                                       &err));
    Dq.ocl_buffer = clBuffDq;

    OCL_CHECK(err, cl::Buffer clBuffU0(context, CL_MEM_READ_ONLY, U0.size, NULL,
                                       &err));
    U0.ocl_buffer = clBuffU0;

    OCL_CHECK(err, cl::Buffer clBuffD0(context, CL_MEM_READ_ONLY, D0.size, NULL,
                                       &err));
    D0.ocl_buffer = clBuffD0;

    OCL_CHECK(err, cl::Buffer clBuffX0(context, CL_MEM_READ_ONLY, X0.size, NULL,
                                       &err));
    X0.ocl_buffer = clBuffX0;

    OCL_CHECK(
        err, cl::Buffer clBuffH(context, CL_MEM_READ_ONLY, H.size, NULL, &err));
    H.ocl_buffer = clBuffH;

    OCL_CHECK(
        err, cl::Buffer clBuffR(context, CL_MEM_READ_ONLY, R.size, NULL, &err));
    R.ocl_buffer = clBuffR;

    OCL_CHECK(
        err, cl::Buffer clBuffy(context, CL_MEM_READ_ONLY, y.size, NULL, &err));
    y.ocl_buffer = clBuffy;

    OCL_CHECK(err, cl::Buffer clBuffoutX(context, CL_MEM_WRITE_ONLY, outX.size,
                                         NULL, &err));
    outX.ocl_buffer = clBuffoutX;

    OCL_CHECK(err, cl::Buffer clBuffoutU(context, CL_MEM_WRITE_ONLY, outU.size,
                                         NULL, &err));
    outU.ocl_buffer = clBuffoutU;
    OCL_CHECK(err, cl::Buffer clBuffoutD(context, CL_MEM_WRITE_ONLY, outD.size,
                                         NULL, &err));
    outD.ocl_buffer = clBuffoutD;
}

void KalmanHLS::setKernelArgs() {
    OCL_CHECK(err, err = kernel.setArg(0, A.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(1, Uq.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(2, Dq.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(3, H.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(4, X0.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(5, U0.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(6, D0.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(7, R.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(8, y.ocl_buffer));

    OCL_CHECK(err, err = kernel.setArg(10, outX.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(11, outU.ocl_buffer));
    OCL_CHECK(err, err = kernel.setArg(12, outD.ocl_buffer));
}

void KalmanHLS::copyDataToDevice(cl::CommandQueue &queue, cl::Event &event) {
    A.extactData(kalmanBuf::DATA_PTR);
    LOG_INFO("A data converted");
    Uq.extactData(kalmanBuf::DATA_PTR);
    LOG_INFO("Uq data converted");
    Dq.extactData(kalmanBuf::DATA_PTR);
    LOG_INFO("Dq data converted");
    U0.extactData(kalmanBuf::DATA_PTR);
    LOG_INFO("U0 data converted");
    D0.extactData(kalmanBuf::DATA_PTR);
    LOG_INFO("D0 data converted");
    X0.extactData(kalmanBuf::DATA_PTR);
    LOG_INFO("X0 data converted");
    H.extactData(kalmanBuf::DATA_PTR);
    LOG_INFO("H data converted");
    R.extactData(kalmanBuf::DATA_PTR);
    LOG_INFO("R data converted");
    y.extactData(kalmanBuf::DATA_PTR);
    LOG_INFO("y data converted");

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
    LOG_INFO("Received outX");
    OCL_CHECK(err,
              queue.enqueueReadBuffer(outU.ocl_buffer, CL_TRUE, 0, outU.size,
                                      outU.data_ptr, nullptr, &event));
    LOG_INFO("Received outU");

    OCL_CHECK(err,
              queue.enqueueReadBuffer(outD.ocl_buffer, CL_TRUE, 0, outD.size,
                                      outD.data_ptr, nullptr, &event));
    LOG_INFO("Received outD");

    outX.extactData(kalmanBuf::CV_MAT);
    LOG_INFO("Extracted outX");
    outU.extactData(kalmanBuf::CV_MAT);
    LOG_INFO("Extracted outU");
    outD.extactData(kalmanBuf::CV_MAT, true);
    LOG_INFO("Extracted outD");
}

void KalmanHLS::executeKernel(cl::CommandQueue &queue, const int &flag) {
    OCL_CHECK(err, kernel.setArg(9, (unsigned char)flag));
    OCL_CHECK(err, err = queue.enqueueTask(kernel));
}

const cv::Mat &KalmanHLS::predict() {}

void KalmanHLS::update(detectionprops detection) {}

kalmanConfig KalmanHLS::dump() {}
#endif
MatManager::MatManager(/* args */) {}
MatManager::~MatManager() {}