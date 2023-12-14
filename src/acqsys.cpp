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

// AqSysJPEGFiles::AqSysJPEGFiles(const std::string &folder)
//     : AqSysFiles(folder) {}

// AqSysJPEGFiles::AqSysJPEGFiles() {}

// AqSysJPEGFiles::~AqSysJPEGFiles() {
//     for (auto it = rawImgBufers.begin(); it < rawImgBufers.end(); it++) {
//         tjFree(*it);
//     }
// }

// void AqSysJPEGFiles::addImgFile(const std::string &path) {
//     // Using C API for TurboJPEG

//     const char *colorspaceName[TJ_NUMCS] = {"RGB", "YCbCr", "GRAY", "CMYK",
//                                             "YCCK"};
//     FILE *jpegFile;
//     size_t jpegSize;
//     long size;

//     unsigned char *imgBuf  = NULL;
//     unsigned char *jpegBuf = NULL;

//     tjhandle tjInstance;

//     if ((jpegFile = fopen(path.c_str(), "rb")) == NULL) {
//         perror("Error opening image file");
//         bailout(imgBuf, jpegBuf, tjInstance);
//         exit(EXIT_FAILURE);
//     }
//     if (fseek(jpegFile, 0, SEEK_END) < 0 || ((size = ftell(jpegFile)) < 0) ||
//         fseek(jpegFile, 0, SEEK_SET) < 0) {
//         perror("Unable to determine input file size");
//         if (jpegFile)
//             fclose(jpegFile);
//         bailout(imgBuf, jpegBuf, tjInstance);
//         exit(EXIT_FAILURE);
//     }
//     if (size == 0) {
//         perror("Input file contains no data");
//         if (jpegFile)
//             fclose(jpegFile);
//         bailout(imgBuf, jpegBuf, tjInstance);
//         exit(EXIT_FAILURE);
//     }
//     jpegSize = size;
//     if ((jpegBuf = (unsigned char *)tjAlloc(jpegSize)) == NULL) {
//         perror("allocating JPEG buffer");
//         if (jpegFile)
//             fclose(jpegFile);
//         bailout(imgBuf, jpegBuf, tjInstance);
//         exit(EXIT_FAILURE);
//     }
//     if (fread(jpegBuf, jpegSize, 1, jpegFile) < 1) {
//         if (jpegFile)
//             fclose(jpegFile);
//         perror("reading input file");
//         bailout(imgBuf, jpegBuf, tjInstance);
//         exit(EXIT_FAILURE);
//     }
//     fclose(jpegFile);
//     jpegFile = NULL;

//     if ((tjInstance = tjInitDecompress()) == NULL) {
//         perror("initializing decompressor");
//         bailout(imgBuf, jpegBuf, tjInstance);
//         exit(EXIT_FAILURE);
//     }

//     // if (tj3Set(tjInstance, TJPARAM_FASTUPSAMPLE, 1) < 0) {
//     //     perror("setting TJPARAM_FASTUPSAMPLE");
//     //     exit(EXIT_FAILURE);
//     // }
//     // if (tj3Set(tjInstance, TJPARAM_FASTDCT, 1) < 0) {
//     //     perror("setting TJPARAM_FASTDCT");
//     //     exit(EXIT_FAILURE);
//     // }

//     int flags = 0;
//     flags |= TJFLAG_FASTUPSAMPLE;
//     flags |= TJFLAG_FASTDCT;
//     int width;
//     int height;
//     int inSubsamp;
//     int inColorspace;

//     if (tjDecompressHeader3(tjInstance, jpegBuf, jpegSize, &width, &height,
//                             &inSubsamp, &inColorspace) < 0) {
//         perror("reading JPEG header");
//         bailout(imgBuf, jpegBuf, tjInstance);
//         exit(EXIT_FAILURE);
//     }
//     // int width        = tj3Get(tjInstance, TJPARAM_JPEGWIDTH);
//     // int height       = tj3Get(tjInstance, TJPARAM_JPEGHEIGHT);
//     // int inSubsamp    = tj3Get(tjInstance, TJPARAM_SUBSAMP);
//     // int inColorspace = tj3Get(tjInstance, TJPARAM_COLORSPACE);

//     int pixelFormat = TJPF_BGR;
//     if ((unsigned long long)width * height * tjPixelSize[pixelFormat] >
//         (unsigned long long)((size_t)-1)) {
//         perror("allocating uncompressed image buffer, Image is too large");
//         bailout(imgBuf, jpegBuf, tjInstance);
//         exit(EXIT_FAILURE);
//     }
//     if ((imgBuf = (unsigned char *)malloc(sizeof(unsigned char) * width *
//                                           height * tjPixelSize[pixelFormat]))
//                                           ==
//         NULL) {

//         perror("allocating uncompressed image buffer");
//         bailout(imgBuf, jpegBuf, tjInstance);
//         exit(EXIT_FAILURE);
//     }

//     if (tjDecompress2(tjInstance, jpegBuf, jpegSize, imgBuf, width, 0,
//     height,
//                       pixelFormat, flags) < 0) {
//         perror("decompressing JPEG image");
//         bailout(imgBuf, jpegBuf, tjInstance);
//         exit(EXIT_FAILURE);
//     }
//     // Keep pointers to the images so it's possible to free memory properly
//     on
//     // destruction.
//     rawImgBufers.push_back(imgBuf);

//     cv::Mat cvImg(height, width, CV_8UC3, imgBuf);

//     imgPaths.push_back(cvImg);

//     /* ------------------------------ Cleaning ------------------------------
//     */ tjFree(jpegBuf); tjDestroy(tjInstance);

//     jpegBuf    = NULL;
//     tjInstance = NULL;
// }

// void AqSysJPEGFiles::bailout(unsigned char *imgBuf, unsigned char *jpegBuf,
//                              tjhandle tjInstance) {
//     tjFree(imgBuf);
//     tjDestroy(tjInstance);
//     tjFree(jpegBuf);
// }

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

const cv::Mat &AqSysCam::getCurrentFrame() {
    return currentFrame;
    // TODO: insérer une instruction return ici
}

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
    // TOOD: Maybe use OpenMP to load images in parallel ? see
    // https://stackoverflow.com/questions/18669296/c-openmp-parallel-for-loop-alternatives-to-stdvector
    for (int i = 0; i < globResult.gl_pathc; i++) {
        //     printProgress(i, globResult.gl_pathc);
        addImgFile(std::string(globResult.gl_pathv[i]));
    }
    std::cout << std::endl;
    globfree(&globResult);
}

AqSysMOT::~AqSysMOT() {}
