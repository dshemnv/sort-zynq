#include "detsys.hpp"
#include "gui.hpp"
#include "sort.hpp"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <regex>

int main(int argc, char const *argv[]) {

    if (argc < 3) {
        LOG_ERR("Please provide path to MOT dataset/webcam device id and yolo "
                "model name");
        exit(EXIT_FAILURE);
    }
    std::string motDataset = argv[1];
    std::string yoloModel  = argv[2];

    // motDataset = argv[1];
    Sort sort(1, 3, 0.3);
    // KalmanOCVCreator<KF_N, KF_M> ocv_kalman;
    // KalmanEigenCreator<KF_N, KF_M> kalman;
    KalmanCreatorFactory<KalmanEigen<KF_N, KF_M>> kalmanFactory;
    // KalmanCreatorFactory<KalmanOCV<KF_N, KF_M>> kalmanFactory;
    AuctionNaive auction(0.001);
    sort.setTracker(&kalmanFactory);
    sort.setIOUSolver(&auction);
    YOLODPU yolo(yoloModel, true);
    AqSys *aq = NULL;
    /* ------------------------------- Load images -------------------------- */
    if (motDataset.length() > 2) {
        AqSysFiles *mot = new AqSysFiles(motDataset);
        // AqSysJPEGFiles files(motDataset);
        glob_t globResult;
        std::string imgPathPattern = motDataset + "/img1/*.jpg";

        // Find all dataset images
        int returnVal =
            glob(imgPathPattern.c_str(), GLOB_ERR, NULL, &globResult);
        if (returnVal != 0) {
            globfree(&globResult);
            LOG_ERR("No files found.");
            exit(EXIT_FAILURE);
        }

        // Load dataset images in acqsys
        for (int i = 0; i < globResult.gl_pathc; i++) {
            // for (int i = 0; i < 2; i++) {
            std::cout << "\r"
                      << "[INFO] Loading image " << i << " / "
                      << globResult.gl_pathc;
            mot->addImgFile(std::string(globResult.gl_pathv[i]));
        }
        std::cout << std::endl;
        globfree(&globResult);
        aq = mot;
    } else {
        int devId     = atoi(motDataset.c_str());
        AqSysCam *cam = new AqSysCam(devId);
        aq            = cam;
    }
    yolo.setAqsys(aq);
    GUI gui(yolo, *aq, "test");
    /* ---------------------------------------------------------------------- */

    if (std::regex_search(yoloModel, std::regex("adas"))) {
        yolo.setLabels(adasLabels);
    } else if (std::regex_search(yoloModel, std::regex("voc"))) {
        yolo.setLabels(vocLabels);
    } else if (std::regex_search(yoloModel, std::regex("yolov4"))) {
        yolo.setLabels(cocoLabels);
    } else {
        LOG_ERR("Wrong model");
        exit(EXIT_FAILURE);
    }

    auto realFPSstart = std::chrono::high_resolution_clock::now();
    auto realFPSstop  = std::chrono::high_resolution_clock::now();
    bool pause        = false;
    while (true) {
        if (!pause) {
            cv::Mat frame;

            std::chrono::duration<double, std::micro> realFPSdiff =
                (realFPSstop - realFPSstart);
            double realFPStime = realFPSdiff.count();
            int realFPS        = static_cast<int>(1 / (realFPStime * 1e-6));
            realFPSstart       = std::chrono::high_resolution_clock::now();

            int result = gui.nextFrame(&frame);
            if (result == 1) {
                std::cout << "No more frames to show" << std::endl;
                break;
            }

            yolo.detect();

            std::chrono::duration<double, std::micro> yoloFPSdiff =
                (std::chrono::high_resolution_clock::now() - realFPSstart);
            double yoloFPStime = yoloFPSdiff.count();
            int yoloFPS        = static_cast<int>(1 / (yoloFPStime * 1e-6));

            std::vector<Metadata> detections =
                yolo.getDetections("person", 0.5);
            auto sortFPSstart = std::chrono::high_resolution_clock::now();

            if (detections.size() >= 1) {
                sort.update(detections);
                std::chrono::duration<double, std::micro> sortFPSdiff =
                    (std::chrono::high_resolution_clock::now() - sortFPSstart);
                double sortFPStime = sortFPSdiff.count();
                int sortFPS        = static_cast<int>(1 / (sortFPStime * 1e-6));
                gui.addFPS(sortFPS, "SORT FPS: ", cv::Point(0, 150));
            }

            gui.addFPS(realFPS, "Overall FPS: ", cv::Point(0, 50));
            gui.addFPS(yoloFPS, "YOLO Inference FPS: ", cv::Point(0, 100));

            std::vector<Metadata> correctedDetections;
            if (detections.size() >= 1) {
                correctedDetections = sort.getCorrectedDetections();
            } else {
                correctedDetections = detections;
            }
            gui.drawFromDetections(correctedDetections);
            // gui.addFPS(fps);
            realFPSstop = std::chrono::high_resolution_clock::now();
        }
        gui.show();
        char c = (char)cv::waitKey(1);
        if (c == 27) {
            break;
        } else if (c == 32) {
            pause = !pause;
        }
    }

    cv::destroyAllWindows();
    delete aq;
    return 0;
}