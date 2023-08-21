#include "detsys.hpp"
#include "gui.hpp"
#include "sort.hpp"
#include <chrono>
#include <opencv2/opencv.hpp>

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
    KalmanOCVCreator ocv_kalman;
    AuctionNaive auction(0.001);
    sort.setTracker(&ocv_kalman);
    sort.setIOUSolver(&auction);
    YOLODPU yolo(yoloModel, true);
    AqSys *aq = NULL;
    /* ------------------------------- Load images -------------------------- */
    if (motDataset.length() > 2) {
        AqSysFiles *mot = new AqSysFiles(motDataset);
        // AqSysJPEGFiles files(motDataset);
        glob_t globResult;
        std::string imgPathPattern = motDataset + "/img1_hd/*.jpg";

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
            mot->addImgFile(std::string(globResult.gl_pathv[i]));
        }

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
    std::string adasLabels[3] = {"car", "person", "cycle"};
    std::string vocLabels[20] = {
        "aeroplane",   "bicycle", "bird",  "boat",      "bottle",
        "bus",         "car",     "cat",   "chair",     "cow",
        "diningtable", "dog",     "horse", "motorbike", "person",
        "pottedplant", "sheep",   "sofa",  "train",     "tvmonitor"};

    if (yoloModel == "yolov3_adas_pruned_0_9") {
        yolo.setLabels(adasLabels);
    } else if (yoloModel == "yolov3_voc_tf") {
        yolo.setLabels(vocLabels);
    } else {
        LOG_ERR("Wrong model");
        exit(EXIT_FAILURE);
    }

    // gui.toggleBb();
    auto realFPSstart = std::chrono::high_resolution_clock::now();
    bool pause        = false;
    while (true) {
        if (!pause) {
            cv::Mat frame;
            int result = gui.nextFrame(&frame);
            if (result == 1) {
                std::cout << "No more frames to show" << std::endl;
                break;
            }
            std::chrono::duration<double, std::micro> realFPSdiff =
                (std::chrono::high_resolution_clock::now() - realFPSstart);
            double realFPStime = realFPSdiff.count();
            int realFPS        = static_cast<int>(1 / (realFPStime * 1e-6));
            realFPSstart       = std::chrono::high_resolution_clock::now();

            yolo.detect();

            std::chrono::duration<double, std::micro> yoloFPSdiff =
                (std::chrono::high_resolution_clock::now() - realFPSstart);
            double yoloFPStime = yoloFPSdiff.count();
            int yoloFPS        = static_cast<int>(1 / (yoloFPStime * 1e-6));

            std::vector<Metadata> detections = yolo.getDetections();
            auto sortFPSstart = std::chrono::high_resolution_clock::now();

            if (detections.size() >= 3) {
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
            if (detections.size() >= 3) {
                correctedDetections = sort.getCorrectedDetections();
            } else {
                correctedDetections = detections;
            }
            gui.drawFromDetections(correctedDetections);
            // gui.addFPS(fps);
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