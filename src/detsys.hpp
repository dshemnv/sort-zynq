#ifndef DETSYS_H
#define DETSYS_H
#include "acqsys.hpp"
#include <glob.h>
#include <opencv2/opencv.hpp>

class DetSys {
  public:
    virtual void start() = 0;
    virtual void stop()  = 0;
    virtual ~DetSys(){};
    virtual std::vector<cv::Rect> getBb(int frameNum) = 0;
};

class MOTData : public DetSys {
  private:
    struct motdet {
        int frameNum;
        double xtl;
        double ytl;
        double height;
        double width;
    };
    // std::vector<std::string> imgs;
    std::vector<std::vector<motdet>> detections;
    std::string datasetName;
    AqSysFiles aqSys;

  public:
    MOTData(const std::string &folder);
    MOTData();
    ~MOTData();
    std::vector<std::vector<motdet>> getDetections();
    const std::string &getName();
    AqSysFiles &getAqsys();
    std::vector<cv::Rect> getBb(int frameNum);
    void load(const std::string &folder);
    void start();
    void stop();
};
#endif