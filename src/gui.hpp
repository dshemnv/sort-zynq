
#include "acqsys.hpp"
#include "detsys.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

class GUI {
  private:
    AqSys *aqsys;
    DetSys *detsys;
    cv::Mat currentFrame;
    std::string name;
    bool showBb;

  public:
    GUI();
    GUI(AqSys &aqsys, const std::string &windowName);
    GUI(DetSys &detsys, AqSys &aqsys, const std::string &windowName);
    ~GUI();
    void toggleBb();
    void drawBb(const cv::Rect &bb, const std::string &label,
                const cv::Scalar &color);
    void drawFromDetections(std::vector<Metadata> &dets);
    int nextFrame(cv::Mat *mat);
    void show();
};