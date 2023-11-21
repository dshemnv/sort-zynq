#ifndef ACQSYS_H
#define ACQSYS_H
#include "utils.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <turbojpeg.h>

class AqSys {
  protected:
    int currentFrameIdx;
    cv::Mat currentFrame;

  public:
    virtual bool eof()                       = 0;
    virtual const cv::Mat &getFrame()        = 0;
    virtual const cv::Mat &getCurrentFrame() = 0;
    virtual ~AqSys(){};
    virtual int index() = 0;
};

class AqSysCam : public AqSys {
  private:
    cv::VideoCapture stream;

  public:
    AqSysCam(int devId);
    ~AqSysCam();
    const cv::Mat &getFrame();
    const cv::Mat &getCurrentFrame();
    bool eof();
    int index();
};

class AqSysFiles : public AqSys {
  protected:
    std::vector<cv::Mat> frames;
    std::string name;

  public:
    AqSysFiles(const std::string &folder);
    AqSysFiles();
    ~AqSysFiles();
    void addImgFile(const std::string &path);
    bool eof();
    const cv::Mat &getFrame();
    const cv::Mat &getCurrentFrame();
    const std::string &getName();
    int index();
    int size();
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

class AqSysMOT : public AqSysFiles {
  private:
    std::string path;

  public:
    AqSysMOT(const std::string &folder);
    ~AqSysMOT();
    void load();
};

#endif