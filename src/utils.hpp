#ifndef UTILS_HPP
#define UTILS_HPP

#include <opencv2/opencv.hpp>

void decomposeInUDU(const cv::Mat &M, cv::Mat *U, cv::Mat *D) {
    int m = M.cols;
    for (int j = m - 1; j > -1; j--) {
        for (int i = j; i > -1; i--) {
            float sigma = M.at<float>(i, j);
            for (int k = j + 1; k < m; k++) {
                sigma = sigma - (U->at<float>(i, k) * D->at<float>(k, k) *
                                 U->at<float>(j, k));
            }
            if (i == j) {
                D->at<float>(j, j) = sigma;
                U->at<float>(j, j) = 1.0;
            } else {
                U->at<float>(i, j) = sigma / D->at<float>(j, j);
            }
        }
    }
}

void mat2FloatPtr(cv::Mat &mat, float *data_ptr) {
    cv::Mat tmp1d = mat.reshape(0, 1);
    for (int i = 0; i < tmp1d.cols; i++) {
        data_ptr[i] = tmp1d.at<float>(i);
    }
}

void floatPtr2Mat(cv::Mat *mat, float *data_ptr) {
    cv::Size origSize = mat->size();
    mat->reshape(0, 1);
    for (int i = 0; i < origSize.height * origSize.width; i++) {
        mat->at<float>(i) = data_ptr[i];
    }
    mat->reshape(0, origSize.height);
}

#endif
