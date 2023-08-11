#ifndef DETSYS_H
#define DETSYS_H
#include "acqsys.hpp"
#include "detection.hpp"
#include <glob.h>
#include <opencv2/opencv.hpp>

// Simple data structure that holds detection metadata
struct Metadata {
    double height;
    double width;
    double x;
    double y;
    std::string label;
    double probability;
    cv::Scalar color;

    Metadata(double x, double y, double height, double width,
             const std::string &label, double probability)
        : x(x), y(y), height(height), width(width), label(label),
          probability(probability) {
        color = cv::Scalar(0, 255, 0);
    }

    Metadata &operator=(const Metadata &det) {
        x           = det.x;
        y           = det.y;
        height      = det.height;
        width       = det.width;
        label       = det.label;
        probability = det.probability;
        color       = det.color;
        return *this;
    }
    cv::Mat toBbMat() {
        // clang-format off
         return (cv::Mat_<double>(4, 1) << x - (width  / 2.0), 
                                           y - (height / 2.0),
                                           x + (width  / 2.0),
                                           y + (height / 2.0));
// clang-format on       
    }
    const cv::Mat toSort() {
        double s = width * height;
        double r = width / height;

        return (cv::Mat_<double>(4, 1) << x, y, s, r);
    }
    const cv::Rect toBb() {
        cv::Rect bb;
        bb.x      = x - (width / 2.0);
        bb.y      = y - (height / 2.0);
        bb.width  = width;
        bb.height = height;

        return bb;
    }
    void setColor(const cv::Scalar &col){
      color = col;
    }
};

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
    std::vector<Metadata> getDetections(int frameNum);
    const std::string &getName();
    AqSysFiles &getAqsys();
    std::vector<cv::Rect> getBb(int frameNum);
    void load(const std::string &folder);
    void start();
    void stop();
};

#endif