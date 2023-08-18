#ifndef ACQSYS_H
#define ACQSYS_H
#include <opencv2/opencv.hpp>
#include <turbojpeg.h>

class AqSys {
  public:
    virtual bool eof()                       = 0;
    virtual const cv::Mat &getFrame()        = 0;
    virtual const cv::Mat &getCurrentFrame() = 0;
    virtual ~AqSys(){};
    virtual int index() = 0;
};

class AqSysCam : public AqSys {
  protected:
    int currentFrameIdx;
    cv::Mat currentFrame;

  public:
    AqSysCam(int devId);
    ~AqSysCam();
    // TODO: Implement webcam, test perf.
};

class AqSysFiles : public AqSys {
  protected:
    std::vector<cv::Mat> frames;
    int currentFrameIdx;

  public:
    AqSysFiles(const std::string &folder);
    AqSysFiles();
    ~AqSysFiles();
    void addImgFile(const std::string &path);
    bool eof();
    const cv::Mat &getFrame();
    const cv::Mat &getCurrentFrame();
    int index();
};

class AqSysJPEGFiles : public AqSysFiles {
  private:
    std::vector<unsigned char *> rawImgBufers;

  public:
    AqSysJPEGFiles(const std::string &folder);
    AqSysJPEGFiles();
    ~AqSysJPEGFiles();
    void addImgFile(const std::string &path);
    void bailout(unsigned char *imgBuf, unsigned char *jpegBuf,
                 tjhandle tjInstance);
};

#endif