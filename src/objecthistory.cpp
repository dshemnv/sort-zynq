#include "objecthistory.hpp"
#include "utils.hpp"

int ObjectHistory::nextid = 0;

ObjectHistory::ObjectHistory(int maxSize) : size(maxSize), id(++nextid) {}

ObjectHistory::~ObjectHistory() {}

void ObjectHistory::setSize(int size) { this->size = size; }

int ObjectHistory::getSize() { return history.size(); }

void ObjectHistory::add(detectionprops det) { history.push(det); }

void ObjectHistory::update(KalmanBase *predictor) {
    predictor->update(history.back());
}

detectionprops ObjectHistory::predict(KalmanBase *predictor) {
    detectionprops last;
    TIMEXEC("Prediction", 10, cv::Mat prediction = predictor->predict());
    cv::Point2f barycenter =
        cv::Point2f(prediction.at<float>(0, 0), prediction.at<float>(3, 0));
    last = history.back();

    detectionprops output = {static_cast<int>(prediction.at<float>(6, 0)),
                             static_cast<int>(prediction.at<float>(7, 0)),
                             barycenter, last.label, last.probability};

    return output;
}

void ObjectHistory::showHistory() {
    ObjectHistory::dethist temp = history;
    while (!temp.empty()) {
        printDetection(temp.front());
        std::cout << std::endl;
        temp.pop();
    }
}

ObjectHistory::dethist ObjectHistory::getHistory() { return history; }