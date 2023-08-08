#include "gui.hpp"

GUI::GUI() {}

GUI::GUI(AqSys &aqsys, const std::string &windowName) : name(windowName) {
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    this->aqsys = &aqsys;
}

GUI::GUI(DetSys &detsys, AqSys &aqsys, const std::string &windowName)
    : name(windowName) {
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    this->detsys = &detsys;
    this->aqsys  = &aqsys;
}

GUI::~GUI() {}

void GUI::toggleBb() { showBb = !showBb; }

void GUI::drawBb(const cv::Rect &bb, const std::string &label,
                 const cv::Scalar &color) {
    cv::HersheyFonts font = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale      = 1.0;
    int thickness         = 2;

    cv::rectangle(currentFrame, bb, color, thickness);
    cv::Size textSize =
        cv::getTextSize(label, font, fontScale, thickness, nullptr);

    cv::Point position = bb.tl() + cv::Point(0, -textSize.height / 4);
    cv::putText(currentFrame, label, position, font, fontScale,
                cv::Scalar(255, 255, 255), thickness);
}

int GUI::nextFrame(cv::Mat *frame) {
    if (!aqsys->eof()) {
        currentFrame = aqsys->getFrame();
        int frameNum = aqsys->index();
        if (showBb) {
            std::vector<cv::Rect> bb = detsys->getBb(frameNum);
            for (std::vector<cv::Rect>::iterator it = bb.begin();
                 it != bb.end(); ++it) {
                drawBb(*it, "test", cv::Scalar(255, 255, 255));
            }
        }
        return 0;
    }
    return 1;
}

void GUI::show() { cv::imshow(name, currentFrame); }
