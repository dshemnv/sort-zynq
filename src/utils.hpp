#ifndef UTILS_HPP
#define UTILS_HPP
#define LOG_INFO(msg) std::cout << "[INFO] " << msg << std::endl;
#define LOG_ERR(msg) std::cout << "[ERROR] " << msg << std::endl;

#include <opencv2/opencv.hpp>

void decomposeInUDU(const cv::Mat &M, cv::Mat *U, cv::Mat *D);
void mat2FloatPtr(cv::Mat *mat, float *data_ptr);
void floatPtr2Mat(cv::Mat *mat, float *data_ptr);
#endif
