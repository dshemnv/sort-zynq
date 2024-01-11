#include "acqsys.hpp"
#include <glob.h>

AqSysFiles::AqSysFiles(const std::string &folder) { currentFrameIdx = 0; }

void AqSysFiles::addImgFile(const std::string &path) {
    imgPaths.push_back(path);
}

AqSysFiles::AqSysFiles() { currentFrameIdx = 0; }

AqSysFiles::~AqSysFiles() {}

bool AqSysFiles::eof() {
    if (currentFrameIdx < imgPaths.size()) {
        return false;
    }
    return true;
}

void AqSysFiles::getFrame() {
    currentFrame = cv::imread(imgPaths[currentFrameIdx++]);
}

const cv::Mat &AqSysFiles::getCurrentFrame() { return currentFrame; }

const std::string &AqSysFiles::getName() { return name; }

int AqSysFiles::index() { return (currentFrameIdx - 1); }

int &AqSysFiles::frameCounter() { return currentFrameIdx; }

int AqSysFiles::size() { return imgPaths.size(); }

AqSysCam::AqSysCam(int devId) {
    std::string gstreamerPipeline =
        "v4l2src device=/dev/video0 ! image/jpeg, width=1920, height=1080, "
        "framerate=30/1 ! jpegdec ! videoconvert ! appsink";
    stream = cv::VideoCapture(gstreamerPipeline, cv::CAP_GSTREAMER);
    if (!stream.isOpened()) {
        LOG_ERR("Unable to open camera " << devId);
    }
}

AqSysCam::~AqSysCam() { stream.release(); }

void AqSysCam::getFrame() {
    stream >> currentFrame;
    currentFrameIdx++;
}

const cv::Mat &AqSysCam::getCurrentFrame() { return currentFrame; }

bool AqSysCam::eof() { return !stream.isOpened(); }
int AqSysCam::index() { return currentFrameIdx; }

AqSysMOT::AqSysMOT(const std::string &folder) : path(folder) {
    name = folder.substr(folder.find_last_of("/") + 1);
}

void AqSysMOT::load() {
    glob_t globResult;
    std::string imgPathPattern = path + "/img1/*.jpg";
    // LOG_INFO(imgPathPattern);

    // Find name of the last folder (seqence name)
    size_t found = path.find_last_of("/");
    if (found == std::string::npos) {
        LOG_ERR("Unable to get folder name");
        exit(EXIT_FAILURE);
    }
    name = path.substr(found + 1); // +1 to exclude '/'

    // Find all dataset images
    int returnVal = glob(imgPathPattern.c_str(), GLOB_ERR, NULL, &globResult);
    if (returnVal != 0) {
        globfree(&globResult);
        LOG_ERR("No files found.");
        exit(EXIT_FAILURE);
    }

    // Load dataset images in acqsys
    // LOG_INFO("Loading " << name);
    // TODO: Maybe use OpenMP to load images in parallel ? see
    // https://stackoverflow.com/questions/18669296/c-openmp-parallel-for-loop-alternatives-to-stdvector
    for (int i = 0; i < globResult.gl_pathc; i++) {
        //     printProgress(i, globResult.gl_pathc);
        addImgFile(std::string(globResult.gl_pathv[i]));
    }
    std::cout << std::endl;
    globfree(&globResult);
}

AqSysMOT::~AqSysMOT() {}
