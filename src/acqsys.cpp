#include "acqsys.hpp"

AqSysFiles::AqSysFiles(const std::string &folder) { currentFrameIdx = 0; }

void AqSysFiles::addImgFile(const std::string &path) {
    cv::Mat img = cv::imread(path);
    frames.push_back(img);
}

AqSysFiles::AqSysFiles() { currentFrameIdx = 0; }

AqSysFiles::~AqSysFiles() {}

bool AqSysFiles::eof() {
    if (currentFrameIdx < frames.size()) {
        return false;
    }
    return true;
}

const cv::Mat &AqSysFiles::getFrame() { return frames[currentFrameIdx++]; }

const cv::Mat &AqSysFiles::getCurrentFrame() {
    return frames[currentFrameIdx - 1];
}

int AqSysFiles::index() { return (currentFrameIdx - 1); }