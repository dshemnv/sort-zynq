
#include "acqsys.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

class GUI {
  private:
    AqSys *aqsys;
    cv::Mat currentFrame;
    std::string name;

  public:
    GUI();
    GUI(AqSys &aqsys, const std::string &windowName);
    ~GUI();
    void drawBb(const cv::Rect &bb, const std::string &label,
                const cv::Scalar &color);
    int nextFrame(cv::Mat *mat);
    void show();
};