#include "kalman.hpp"
#ifdef KALMAN_ACCEL
#include "common/xf_headers.hpp"
#include "xcl2.hpp"
#endif
#include "utils.hpp"
#include <cassert>

kalmanParams KalmanBase::getParams() { return params; }
void KalmanBase::setParams(kalmanParams params) { this->params = params; }
// TODO: Change this class to be templated instead of using kalmanParams
KalmanOCV::KalmanOCV(kalmanParams params) {
    kf = cv::KalmanFilter();
    kf.init(params.dynamParams, params.measureParams, params.controlParams);
    setParams(params);
}

KalmanOCV::KalmanOCV(int dynamParams, int measureParams, int controlParams) {
    kf = cv::KalmanFilter(dynamParams, measureParams, controlParams);
}

KalmanOCV::KalmanOCV() {}

KalmanOCV::~KalmanOCV() {}

void KalmanOCV::init(cv::Mat initialEstimateUncertainty) {
    kf.errorCovPre = initialEstimateUncertainty;
}

const cv::Mat &KalmanOCV::getState() { return kf.statePost; }

void KalmanOCV::setState(const cv::Mat &new_state) { kf.statePost = new_state; }

void KalmanOCV::load(kalmanConfig config) {
    config.F.copyTo(kf.transitionMatrix);
    config.Q.copyTo(kf.processNoiseCov);
    config.R.copyTo(kf.measurementNoiseCov);
    config.H.copyTo(kf.measurementMatrix);
    config.P.copyTo(kf.errorCovPost);
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

void KalmanOCV::update(const cv::Mat &meas) { kf.correct(meas); }

#ifdef KALMAN_ACCEL
KalmanHLS::KalmanHLS() {}
KalmanHLS::KalmanHLS(kalmanParams params) { setParams(params); }
KalmanHLS::~KalmanHLS() {
    // free(A.data_ptr);
    // LOG_INFO("A freed");
    // free(Uq.data_ptr);
    // LOG_INFO("Uq freed");
    // free(Dq.data_ptr);
    // LOG_INFO("Dq freed");
    // free(U0.data_ptr);
    // LOG_INFO("U0 freed");
    // free(D0.data_ptr);
    // LOG_INFO("D0 freed");
    // free(X0.data_ptr);
    // LOG_INFO("X0 freed");
    // free(H.data_ptr);
    // LOG_INFO("H freed");
    // free(R.data_ptr);
    // LOG_INFO("R freed");
    // free(y.data_ptr);
    // LOG_INFO("y freed");
    // free(outX.data_ptr);
    // LOG_INFO("outX freed");
    // free(outD.data_ptr);
    // LOG_INFO("outD freed");
    // free(outU.data_ptr);
    // LOG_INFO("outU freed");
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

    X0.cv_mat   = cv::Mat::zeros(KF_N, 1, CV_32F);
    X0.size     = KF_N * sizeof(float);
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

    // Store references
    queuePtr = &queue;
    eventPtr = &event;

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

    std::cout << "wait for enter" << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    LOG_INFO("Allocating buffers");
    allocateBuffers(context);

    LOG_INFO("Setting Accelerator arguments");
    setKernelArgs();

    LOG_INFO("Setup initial data to device transfer");
    copyDataToDevice(queue, event);

    LOG_INFO("Execute accelerator initialization");
    // executeKernel(queue, INIT_EN + TIMEUPDATE_EN + XOUT_EN_TU + UDOUT_EN_TU);
    executeKernel(queue, INIT_EN + TIMEUPDATE_EN + MEASUPDATE_EN + XOUT_EN_MU +
                             UDOUT_EN_MU);

    LOG_INFO("Init output vectors");
    outX.cv_mat = cv::Mat::zeros(KF_N, 1, CV_32F);
    outD.cv_mat = cv::Mat::zeros(KF_N, 1, CV_32F);
    outU.cv_mat = cv::Mat::zeros(KF_N, KF_N, CV_32F);

    LOG_INFO("Transfering data back");
    copyDataToHost(queue, event);

    LOG_INFO("Successfully initialized accelerator");
}

void KalmanHLS::load(kalmanConfig config) {
    A.cv_mat   = config.F.clone();
    A.size     = config.F.cols * config.F.rows * sizeof(float);
    A.data_ptr = (float *)malloc(A.size);

    H.cv_mat   = config.H.clone();
    H.size     = config.H.cols * config.H.rows * sizeof(float);
    H.data_ptr = (float *)malloc(H.size);

    R.cv_mat   = config.R.diag(0);
    R.size     = KF_M * sizeof(float);
    R.data_ptr = (float *)malloc(R.size);

    cv::Mat uq(config.Q.size(), CV_32F);
    cv::Mat dq(config.Q.size(), CV_32F);
    decomposeInUDU(config.Q.clone(), &uq, &dq);

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

    assert(A.size == KF_N * KF_N * sizeof(float));
    assert(Uq.size == KF_N * KF_N * sizeof(float));
    assert(Dq.size == KF_N * sizeof(float));
    assert(U0.size == KF_N * KF_N * sizeof(float));
    assert(D0.size == KF_N * sizeof(float));
    assert(X0.size == KF_N * sizeof(float));
    assert(H.size == KF_M * KF_N * sizeof(float));
    assert(R.size == KF_M * sizeof(float));
    assert(y.size == KF_M * sizeof(float));

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

    assert(outX.size == KF_N * sizeof(float));
    assert(outU.size == KF_N * KF_N * sizeof(float));
    assert(outD.size == KF_N * sizeof(float));

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
    outD.extactData(kalmanBuf::CV_MAT);
    LOG_INFO("Extracted outD");

    // std::cout << outX.cv_mat << std::endl;
    // std::cout << outU.cv_mat << std::endl;
    // std::cout << outD.cv_mat << std::endl;
}

void KalmanHLS::executeKernel(cl::CommandQueue &queue, const int &flag) {
    OCL_CHECK(err, kernel.setArg(9, (unsigned char)flag));
    OCL_CHECK(err, err = queue.enqueueTask(kernel, NULL, eventPtr));
    clWaitForEvents(1, (const cl_event *)eventPtr);
}

const cv::Mat &KalmanHLS::predict() {
    copyDataToDevice(*queuePtr, *eventPtr);

    LOG_INFO("Execute accelerator prediction");
    executeKernel(*queuePtr,
                  INIT_EN + TIMEUPDATE_EN + XOUT_EN_TU + UDOUT_EN_TU);

    copyDataToHost(*queuePtr, *eventPtr);

    LOG_INFO("Successfully made a prediction");
}

void KalmanHLS::update(detectionprops detection) {
    float xd = detection.barycenter.x;
    float yd = detection.barycenter.y;
    float wd = (float)detection.width;
    float hd = (float)detection.height;

    y.cv_mat = (cv::Mat_<float>(y.cv_mat.size()) << xd, yd, wd, hd);

    copyDataToDevice(*queuePtr, *eventPtr);

    LOG_INFO("Execute accelerator measure update");
    // FIXME: This is not working properly. The Accelerator hangs in a START
    // state on second call
    executeKernel(*queuePtr,
                  INIT_EN + MEASUPDATE_EN + XOUT_EN_MU + UDOUT_EN_MU);

    copyDataToHost(*queuePtr, *eventPtr);

    LOG_INFO("Successfully made a measure update");
}

void KalmanHLS::finish() {
    queuePtr = nullptr;
    eventPtr = nullptr;
}

void KalmanHLS::printOutput() {
    LOG_INFO("X output");
    LOG_INFO(outX.cv_mat);

    LOG_INFO("P output");
    LOG_INFO(outU.cv_mat);
    LOG_INFO(outD.cv_mat);

    // cv::Mat P = outU.cv_mat * outD.cv_mat * outU.cv_mat.t();
    // LOG_INFO(P);
}

kalmanConfig KalmanHLS::dump() {
    cv::FileStorage fs("config.json", cv::FileStorage::WRITE);

    fs << "A" << A.cv_mat;
    fs << "Uq" << Uq.cv_mat;
    fs << "Dq" << Dq.cv_mat;
    fs << "U0" << D0.cv_mat;
    fs << "D0" << D0.cv_mat;
    fs << "X0" << X0.cv_mat;
    fs << "H" << H.cv_mat;
    fs << "R" << R.cv_mat;
    fs << "y" << y.cv_mat;

    fs.release();
}
#endif
MatManager::MatManager(/* args */) {}
MatManager::~MatManager() {}