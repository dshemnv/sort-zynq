#ifndef ACQSYS_H
#define ACQSYS_H
#include <opencv2/opencv.hpp>

class AqSys {
  public:
    virtual bool eof()                = 0;
    virtual const cv::Mat &getFrame() = 0;
    virtual ~AqSys(){};
    virtual int index() = 0;
};

class AqSysFiles : public AqSys {
  private:
    std::vector<cv::Mat> frames;
    int currentFrameIdx;

  public:
    AqSysFiles(const std::string &folder);
    AqSysFiles();
    ~AqSysFiles();
    void addImgFile(const std::string &path);
    bool eof();
    const cv::Mat &getFrame();
    int index();
};
#endif