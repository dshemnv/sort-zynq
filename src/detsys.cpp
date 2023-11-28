#include "detsys.hpp"
#include "utils.hpp"
#include <fstream>
#include <regex>
#include <sstream>

std::string vocLabels[20] = {
    "aeroplane",   "bicycle", "bird",  "boat",      "bottle",
    "bus",         "car",     "cat",   "chair",     "cow",
    "diningtable", "dog",     "horse", "motorbike", "person",
    "pottedplant", "sheep",   "sofa",  "train",     "tvmonitor"};
std::string cocoLabels[80] = {"person",        "bicycle",      "car",
                              "motorcycle",    "airplane",     "bus",
                              "train",         "truck",        "boat",
                              "traffic light", "fire hydrant", "stop sign",
                              "parking meter", "bench",        "bird",
                              "cat",           "dog",          "horse",
                              "sheep",         "cow",          "elephant",
                              "bear",          "zebra",        "giraffe",
                              "backpack",      "umbrella",     "handbag",
                              "tie",           "suitcase",     "frisbee",
                              "skis",          "snowboard",    "sports ball",
                              "kite",          "baseball bat", "baseball glove",
                              "skateboard",    "surfboard",    "tennis racket",
                              "bottle",        "wine glass",   "cup",
                              "fork",          "knife",        "spoon",
                              "bowl",          "banana",       "apple",
                              "sandwich",      "orange",       "broccoli",
                              "carrot",        "hot dog",      "pizza",
                              "donut",         "cake",         "chair",
                              "couch",         "potted plant", "bed",
                              "dining table",  "toilet",       "tv",
                              "laptop",        "mouse",        "remote",
                              "keyboard",      "cell phone",   "microwave",
                              "oven",          "toaster",      "sink",
                              "refrigerator",  "book",         "clock",
                              "vase",          "scissors",     "teddy bear",
                              "hair drier",    "toothbrush"};

std::string adasLabels[3] = {"car", "person", "cycle"};

MOTData::MOTData(const std::string &folder) {
    glob_t globResult;
    std::string imgPathPattern = folder + "/img1/*.jpg";

    // Find all dataset images
    int returnVal = glob(imgPathPattern.c_str(), GLOB_ERR, NULL, &globResult);
    if (returnVal != 0) {
        globfree(&globResult);
        LOG_ERR("No files found.");
        exit(EXIT_FAILURE);
    }

    // Load dataset images in acqsys
    for (int i = 0; i < globResult.gl_pathc; i++) {
        aqSys.addImgFile(std::string(globResult.gl_pathv[i]));
    }

    globfree(&globResult);
    // Match last folder name to get the dataset name
    std::regex pattern(".*/([^/]+)/?");
    std::smatch sm;
    std::regex_search(folder, sm, pattern);

    datasetName = sm[1].str();

    // Load detection information
    std::ifstream detFile(folder + "/det/det.txt");
    std::string line;
    while (std::getline(detFile, line)) {
        std::istringstream lineStream(line);
        std::string currentField;
        int fieldcnt = 0;
        // TODO: Replace motdet with cv::Rect
        motdet currentDet;
        while ((std::getline(lineStream, currentField, ',')) &&
               (fieldcnt < 6)) {
            switch (fieldcnt) {
            case 0:
                currentDet.frameNum = std::stoi(currentField);
                break;
            case 2:
                currentDet.xtl = std::stod(currentField);
                break;
            case 3:
                currentDet.ytl = std::stod(currentField);
                break;
            case 4:
                currentDet.width = std::stod(currentField);
                break;
            case 5:
                currentDet.height = std::stod(currentField);
                break;
            default:
                break;
            }
            fieldcnt++;
        }
        if ((!detections.empty()) &&
            (currentDet.frameNum == detections.back().back().frameNum)) {
            detections.back().push_back(currentDet);
        } else {
            detections.push_back(std::vector<motdet>({currentDet}));
        }
    }
}

MOTData::MOTData() {}

MOTData::~MOTData() {}
std::vector<Metadata> MOTData::getDetections() {
    std::vector<motdet> currentBbs = detections.at(aqSys.index());

    std::vector<Metadata> detProps;

    for (std::vector<motdet>::iterator it = currentBbs.begin();
         it < currentBbs.end(); it++) {
        Metadata det(it->xtl + (it->width / 2), it->ytl + (it->height / 2),
                     it->height, it->width, std::string("person"), 50);
        detProps.push_back(det);
    }
    return detProps;
}

const std::string &MOTData::getName() { return datasetName; }

AqSysFiles &MOTData::getAqsys() { return aqSys; }

std::vector<cv::Rect> MOTData::getBb() {
    std::vector<cv::Rect> output;

    // if (frameNum == detections.size()) {
    //     return output;
    // }

    std::vector<motdet> currentDetections = detections.at(aqSys.index());

    for (std::vector<motdet>::iterator it = currentDetections.begin();
         it != currentDetections.end(); ++it) {
        cv::Rect bb;

        bb.x = it->xtl;
        bb.y = it->ytl;

        bb.width  = it->width;
        bb.height = it->height;

        output.push_back(bb);
    }

    return output;
}

void MOTData::load(const std::string &folder) {}

void MOTData::start() {}

void MOTData::stop() {}

#ifdef DPUYOLO
YOLODPU::YOLODPU(const std::string &modelName, bool needPreprocess) {
    yoloInstance = vitis::ai::YOLOv3::create(modelName, needPreprocess);
}

YOLODPU::YOLODPU(AqSys *aq) : aqsys(aq) {}

YOLODPU::~YOLODPU() {}

void YOLODPU::setYOLO(const std::string &modelName, bool needPreprocess) {
    yoloInstance = vitis::ai::YOLOv3::create(modelName, needPreprocess);
}

void YOLODPU::setLabels(std::string *labelsList) { label = labelsList; }
void YOLODPU::setAqsys(AqSys *aqsys) { this->aqsys = aqsys; }

AqSys *YOLODPU::getAqsys() { return aqsys; }

void YOLODPU::detect() {
    cv::Mat frame                   = aqsys->getCurrentFrame();
    vitis::ai::YOLOv3Result results = yoloInstance->run(frame);
    currentDetections               = yoloResultToMetadata(results);
}

void YOLODPU::detect_mt(int n_threads) {
    // Multi-threaded object detection
}

std::vector<Metadata>
YOLODPU::yoloResultToMetadata(vitis::ai::YOLOv3Result &result) {
    std::vector<Metadata> output;
    int imgHeight = aqsys->getCurrentFrame().rows;
    int imgWidth  = aqsys->getCurrentFrame().cols;
    for (std::vector<vitis::ai::YOLOv3Result::BoundingBox>::iterator it =
             result.bboxes.begin();
         it < result.bboxes.end(); it++) {
        double height = static_cast<double>(it->height * imgHeight);
        double width  = static_cast<double>(it->width * imgWidth);
        double x = static_cast<double>((it->x + (it->width / 2.0)) * imgWidth);
        double y =
            static_cast<double>((it->y + (it->height / 2.0)) * imgHeight);
        double probability = static_cast<double>(it->score);
        Metadata det(x, y, height, width, label[it->label], probability);
        det.setColor(getColor(it->label));
        output.push_back(det);
    }
    return output;
}

std::vector<Metadata> YOLODPU::getDetections(const std::string &label_filter,
                                             const double precision_filter) {
    if (!label_filter.empty()) {
        std::vector<Metadata> filtered;
        for (std::vector<Metadata>::iterator it = currentDetections.begin();
             it != currentDetections.end(); it++) {
            if (it->label == label_filter &&
                it->probability >= precision_filter) {
                filtered.push_back(*it);
            }
        }
        return filtered;
    }
    return currentDetections;
}

std::vector<cv::Rect> YOLODPU::getBb() {
    detect();
    std::vector<cv::Rect> bbs;
    for (std::vector<Metadata>::iterator it = currentDetections.begin();
         it < currentDetections.end(); it++) {
        bbs.push_back(it->toBb());
    }
    return bbs;
    ;
}

void YOLODPU::stop() {}

void YOLODPU::start() {}
#endif