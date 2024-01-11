#include "gui.hpp"

GUI::GUI() {}

GUI::GUI(AqSys &aqsys, const std::string &windowName)
    : name(windowName), showBb(false) {
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    this->aqsys = &aqsys;
}

GUI::GUI(DetSys &detsys, AqSys &aqsys, const std::string &windowName)
    : name(windowName), showBb(false) {
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    this->detsys = &detsys;
    this->aqsys  = &aqsys;
}

GUI::~GUI() {}

void GUI::toggleBb() { showBb = !showBb; }

void GUI::drawBb(const cv::Rect &bb, const std::string &label,
                 const cv::Scalar &color) {
    if (currentFrame.empty()) {
        currentFrame = aqsys->getCurrentFrame();
    }
    cv::HersheyFonts font = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale      = 1.0;
    int thickness         = 2;

    cv::rectangle(currentFrame, bb, color, thickness);
    cv::Size textSize =
        cv::getTextSize(label, font, fontScale, thickness, nullptr);

    cv::Rect txtBg(bb.x, bb.y - textSize.height, textSize.width,
                   textSize.height);

    cv::rectangle(currentFrame, txtBg, cv::Scalar::all(255), -1);

    cv::Point position = bb.tl() + cv::Point(0, -textSize.height / 4);
    cv::putText(currentFrame, label, position, font, fontScale, color,
                thickness);
}

void GUI::drawFromDetections(std::vector<Metadata> &dets) {
    for (std::vector<Metadata>::iterator it = dets.begin(); it != dets.end();
         it++) {
        drawBb(it->toBb(), it->label, it->color);
    }
}

int GUI::nextFrame(cv::Mat *frame) {
    if (!aqsys->eof()) {
        aqsys->getFrame();
        currentFrame = aqsys->getCurrentFrame();
        if (showBb) {
            std::vector<cv::Rect> bb = detsys->getBb();
            for (std::vector<cv::Rect>::iterator it = bb.begin();
                 it != bb.end(); ++it) {
                drawBb(*it, "test", cv::Scalar(255, 255, 255));
            }
        }
        return 0;
    }
    return 1;
}

void GUI::addFPS(int fps, const std::string &text, const cv::Point &pos) {
    cv::putText(currentFrame, text + std::to_string(fps), pos,
                cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);
}

void GUI::show() {
    if (currentFrame.empty()) {
        currentFrame = aqsys->getCurrentFrame();
    }
    cv::imshow(name, currentFrame);
}