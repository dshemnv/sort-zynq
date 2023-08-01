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
#endif
