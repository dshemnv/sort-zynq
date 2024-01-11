#ifndef ACQSYS_H
#define ACQSYS_H
#include "utils.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

class AqSys {
  protected:
    int currentFrameIdx;
    cv::Mat currentFrame;

  public:
    virtual bool eof()                       = 0;
    virtual void getFrame()                  = 0;
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
    void getFrame();
    const cv::Mat &getCurrentFrame();
    bool eof();
    int index();
};

class AqSysFiles : public AqSys {
  protected:
    std::vector<std::string> imgPaths;
    std::string name;
    cv::Mat currentFrame;

  public:
    AqSysFiles(const std::string &folder);
    AqSysFiles();
    ~AqSysFiles();
    void addImgFile(const std::string &path);
    bool eof();
    void getFrame();
    const cv::Mat &getCurrentFrame();
    const std::string &getName();
    int index();
    int &frameCounter();
    int size();
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