#ifndef UTILS_HPP
#define UTILS_HPP
#define LOG_INFO(msg) std::cout << "[INFO] " << msg << std::endl;
#define LOG_ERR(msg) std::cout << "[ERROR] " << msg << std::endl;
#include <chrono>
#include <opencv2/opencv.hpp>
#include <typeinfo>
#define TIMEXEC(name, N, call)                                                 \
    double times[N];                                                           \
    auto start = std::chrono::high_resolution_clock::now();                    \
    call;                                                                      \
    auto end = std::chrono::high_resolution_clock::now();                      \
    std::chrono::duration<double, std::micro> diff = (end - start);            \
    times[0]                                       = diff.count();             \
                                                                               \
    for (int i = 0; i < N - 1; i++) {                                          \
        auto start = std::chrono::high_resolution_clock::now();                \
        call;                                                                  \
        auto end = std::chrono::high_resolution_clock::now();                  \
        std::chrono::duration<double, std::micro> diff = (end - start);        \
        times[i]                                       = diff.count();         \
    }                                                                          \
    double sum = 0;                                                            \
    for (int i = 0; i < N; i++) {                                              \
        sum += times[i];                                                       \
    }                                                                          \
    sum = sum / N;                                                             \
    std::cout << name << " took " << sum << " μs"                             \
              << " for " << N << " executions." << std::endl;                  \
    // exit(EXIT_SUCCESS);

void decomposeInUDU(const cv::Mat &M, cv::Mat *U, cv::Mat *D);
void mat2FloatPtr(cv::Mat *mat, float *data_ptr);
void floatPtr2Mat(cv::Mat *mat, float *data_ptr);
bool hasNan(const cv::Mat &mat);

template <class T> void removeRow(cv::Mat &matIn, int rowIdx) {
    cv::Size inSize = matIn.size();
    cv::Mat_<T> matOut(inSize.height - 1, inSize.width);

    if (rowIdx > 0) {
        cv::Rect selection(0, 0, inSize.width, rowIdx);
        matIn(selection).copyTo(matOut(selection));
    }

    if (rowIdx < inSize.height - 1) {
        cv::Rect selection1(0, rowIdx + 1, inSize.width,
                            inSize.height - rowIdx - 1);
        cv::Rect selection2(0, rowIdx, inSize.width,
                            inSize.height - rowIdx - 1);

        matIn(selection1).copyTo(matOut(selection2));
    }
    matIn = matOut;
}

template <class T> bool findValueInMat(const cv::Mat &mat, T value) {
    for (int i = 0; i < mat.rows; i++) {
        const T *row = mat.ptr<T>(i);
        if (std::find(row, row + mat.cols, value) != row + mat.cols) {
            return true;
        }
    }
    return false;
}

void printProgress(int currentStep, int totalSteps);

#endif
