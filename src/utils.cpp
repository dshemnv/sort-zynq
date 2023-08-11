#include "utils.hpp"

// Adapted from arXiv:2203.06105
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

void mat2FloatPtr(cv::Mat *mat, float *data_ptr) {
    cv::Mat tmp1d;
    if (!mat->isContinuous()) {
        tmp1d = mat->clone();
        tmp1d = tmp1d.reshape(0, 1);
    } else {
        tmp1d = mat->reshape(0, 1);
    }
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

bool hasNan(const cv::Mat &mat) {
    cv::Mat tmp = mat.clone();
    tmp.convertTo(tmp, CV_32F);
    cv::patchNaNs(tmp, -1.0);
    for (int i = 0; i < tmp.rows; i++) {
        for (int j = 0; j < tmp.cols; j++) {
            if (tmp.at<float>(i, j) == -1.0) {
                return true;
            }
        }
    }
    return false;
}