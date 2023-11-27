#ifndef DETSYS_H
#define DETSYS_H
#include "acqsys.hpp"
#include <glob.h>
#include <opencv2/opencv.hpp>
#ifdef DPUYOLO
// #include <vitis/ai/demo.hpp>
#include <vitis/ai/yolov3.hpp>
#endif

extern std::string vocLabels[20];
extern std::string cocoLabels[80];
extern std::string adasLabels[3];

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
        return (cv::Mat_<double>(4, 1) << x - (width / 2.0), y - (height / 2.0),
                x + (width / 2.0), y + (height / 2.0));
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
    void setColor(const cv::Scalar &col) { color = col; }
};

class DetSys {
  public:
    virtual void start() = 0;
    virtual void stop()  = 0;
    virtual ~DetSys(){};
    virtual std::vector<cv::Rect> getBb() = 0;
    virtual void setAqsys(AqSys *aqsys)   = 0;
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
    std::vector<Metadata> getDetections();
    const std::string &getName();
    AqSysFiles &getAqsys();
    std::vector<cv::Rect> getBb();
    void load(const std::string &folder);
    void start();
    void stop();
};

static cv::Scalar getColor(int label) {
    int c[3];
    for (int i = 1, j = 0; i <= 9; i *= 3, j++) {
        c[j] = ((label / i) % 3) * 127;
    }
    return cv::Scalar(c[2], c[1], c[0]);
}

#ifdef DPUYOLO
class YOLODPU : public DetSys {
  private:
    std::vector<Metadata> currentDetections;
    AqSys *aqsys;
    std::unique_ptr<vitis::ai::YOLOv3> yoloInstance;
    std::string *label;
    int nClasses;

  public:
    YOLODPU(const std::string &modelName, bool needPreprocess);
    YOLODPU(AqSys *aq);
    ~YOLODPU();
    void setYOLO(const std::string &modelName, bool needPreprocess);
    void setLabels(std::string *labelsList);
    void setAqsys(AqSys *aqsys);
    AqSys *getAqsys();
    void detect();
    void detect_mt(int n_threads);
    std::vector<Metadata> yoloResultToMetadata(vitis::ai::YOLOv3Result &result);
    std::vector<Metadata> &getDetections();
    std::vector<cv::Rect> getBb();
    void start();
    void stop();
};
#endif

#endif