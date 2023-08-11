#include "detsys.hpp"
#include "gui.hpp"
#include "sort.hpp"
#include <chrono>
#include <opencv2/opencv.hpp>

int main(int argc, char const *argv[]) {
    std::string mot_dataset;

    if (argc < 2) {
        LOG_ERR("Please provide path to MOT dataset.");
        exit(EXIT_FAILURE);
    }

    mot_dataset = argv[1];

    Sort sort(1, 3, 0.3);
    // sort.saveResults();
    KalmanOCVCreator ocv_kalman;
    AuctionNaive auction(0.001);
    sort.setTracker(&ocv_kalman);
    sort.setIOUSolver(&auction);
    MOTData data(mot_dataset);

    GUI gui(data, data.getAqsys(), "test");

    // gui.toggleBb();
    bool pause        = false;
    auto realFPSstart = std::chrono::high_resolution_clock::now();
    while (true) {
        if (!pause) {
            cv::Mat frame;
            int result = gui.nextFrame(&frame);
            if (result == 1) {
                break;
            }
            auto start = std::chrono::high_resolution_clock::now();

            sort.update(data.getDetections(sort.getFrameCnt()));

            auto end = std::chrono::high_resolution_clock::now();

            std::chrono::duration<double, std::micro> diff = (end - start);
            auto time                                      = diff.count();

            std::chrono::duration<double, std::micro> realFPSdiff =
                (std::chrono::high_resolution_clock::now() - realFPSstart);
            double realFPStime = realFPSdiff.count();
            int realFPS        = static_cast<int>(1 / (realFPStime * 1e-6));

            int fps = static_cast<int>(1 / (time * 1e-6));
            gui.addFPS(fps, "Processing FPS: ", cv::Point(0, 50));
            gui.addFPS(realFPS, "Real output FPS: ", cv::Point(0, 100));

            realFPSstart = std::chrono::high_resolution_clock::now();
            std::vector<Metadata> correctedDetections =
                sort.getCorrectedDetections();
            gui.drawFromDetections(correctedDetections);
        }
        gui.show();
        char c = (char)cv::waitKey(25);
        if (c == 27) {
            break;
        } else if (c == 32) {
            pause = !pause;
        }
    }

    cv::destroyAllWindows();
    // sort.writeTrackingResults("results.txt");

    // KalmanOCVCreator ocv_kalman;

    // KalmanBase *kalman_ptr = new KalmanOCV;
    // kalman_ptr             = ocv_kalman.create(7, 4);

    // kalmanConfig config;

    // config.F                = cv::Mat1d::eye(KF_N, KF_N); // F
    // config.F.at<double>(4)  = 1.0;
    // config.F.at<double>(12) = 1.0;
    // config.F.at<double>(20) = 1.0;

    // config.Q     = cv::Mat1d::eye(KF_N, KF_N); // Q
    // double *last = config.Q.ptr<double>(KF_N - 1) + KF_N - 1;
    // cv::Rect QsubMatIdx(4, 4, 3, 3);
    // *last *= 0.01;
    // config.Q(QsubMatIdx) *= 0.01;

    // config.R = cv::Mat1d::eye(KF_M, KF_M); // R
    // cv::Rect RsubMatIdx(2, 2, 2, 2);
    // config.R(RsubMatIdx) *= 10.0;

    // // clang-format off
    // config.H = (cv::Mat_<double>(KF_M, KF_N) <<  1, 0, 0, 0, 0, 0, 0,
    //                                              0, 1, 0, 0, 0, 0, 0,
    //                                              0, 0, 1, 0, 0, 0, 0,
    //                                              0, 0, 0, 1, 0, 0, 0); //
    //                                              H
    // // clang-format on

    // config.P = cv::Mat1d::eye(KF_N, KF_N); // P
    // config.P(QsubMatIdx) *= 1000.0;
    // config.P *= 10.0;

    // kalman_ptr->load(config);
    // kalman_ptr->setState(cv::Mat1d::zeros(7, 1));

    // cv::Mat prediction = kalman_ptr->predict();

    // delete kalman_ptr;
    return 0;
}