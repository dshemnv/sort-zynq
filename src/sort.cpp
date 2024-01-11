#include "sort.hpp"
#include <cstdio>

Sort::Sort(int maxAge, int minHits, double iouThreshold)
    : maxAge(maxAge), minHits(minHits), iouThreshold(iouThreshold),
      lsapSolver(nullptr), trackCreator(nullptr), hitCounter(0) {

    config.F                = cv::Mat1d::eye(KF_N, KF_N); // F
    config.F.at<double>(4)  = 1.0;
    config.F.at<double>(12) = 1.0;
    config.F.at<double>(20) = 1.0;

    config.Q     = cv::Mat1d::eye(KF_N, KF_N); // Q
    double *last = config.Q.ptr<double>(KF_N - 1) + KF_N - 1;
    cv::Rect QsubMatIdx(4, 4, 3, 3);
    *last *= 0.01;
    config.Q(QsubMatIdx) *= 0.01;

    config.R = cv::Mat1d::eye(KF_M, KF_M); // R
    cv::Rect RsubMatIdx(2, 2, 2, 2);
    config.R(RsubMatIdx) *= 10.0;

    // clang-format off
    config.H = (cv::Mat_<double>(KF_M, KF_N) <<  1, 0, 0, 0, 0, 0, 0,
                                                 0, 1, 0, 0, 0, 0, 0,
                                                 0, 0, 1, 0, 0, 0, 0,
                                                 0, 0, 0, 1, 0, 0, 0); // H
    // clang-format on

    config.P = cv::Mat1d::eye(KF_N, KF_N); // P
    config.P(QsubMatIdx) *= 1000.0;
    config.P *= 10.0;
}

Sort::Sort() {}

Sort::~Sort() { lsapSolver = nullptr; }

cv::Mat Sort::iou(const cv::Mat &bb1, const cv::Mat &bb2) {
    // Compute IOU between 2 bb of the form [x_tl, y_tl, x_br, y_br]
    // This should support asymetric arrays

    cv::Mat output = cv::Mat1d::zeros(bb1.rows, bb2.rows);

    for (int newDetIdx = 0; newDetIdx < output.rows; newDetIdx++) {
        for (int trackPredIdx = 0; trackPredIdx < output.cols; trackPredIdx++) {
            double maxXtl = cv::max(bb1.at<double>(newDetIdx, 0),
                                    bb2.at<double>(trackPredIdx, 0));
            double maxYtl = cv::max(bb1.at<double>(newDetIdx, 1),
                                    bb2.at<double>(trackPredIdx, 1));
            double maxXbr = cv::min(bb1.at<double>(newDetIdx, 2),
                                    bb2.at<double>(trackPredIdx, 2));
            double maxYbr = cv::min(bb1.at<double>(newDetIdx, 3),
                                    bb2.at<double>(trackPredIdx, 3));

            double intersectionWidth  = cv::max(0.0, maxXbr - maxXtl);
            double intersectionHeight = cv::max(0.0, maxYbr - maxYtl);

            double intersectionArea = intersectionHeight * intersectionWidth;

            double bb1Area =
                (bb1.at<double>(newDetIdx, 2) - bb1.at<double>(newDetIdx, 0)) *
                (bb1.at<double>(newDetIdx, 3) - bb1.at<double>(newDetIdx, 1));
            double bb2Area = (bb2.at<double>(trackPredIdx, 2) -
                              bb2.at<double>(trackPredIdx, 0)) *
                             (bb2.at<double>(trackPredIdx, 3) -
                              bb2.at<double>(trackPredIdx, 1));

            double iouVal =
                intersectionArea / (bb1Area + bb2Area - intersectionArea);

            output.at<double>(newDetIdx, trackPredIdx) = iouVal;
        }
    }

    // // Intersection
    // cv::Mat maxXtl = cv::max(bb1.col(0), bb2.col(0));
    // cv::Mat maxYtl = cv::max(bb1.col(1), bb2.col(1));
    // cv::Mat maxXbr = cv::max(bb1.col(2), bb2.col(2));
    // cv::Mat maxYbr = cv::max(bb1.col(3), bb2.col(3));

    // cv::Mat intersectionWidth  = cv::max(0.0, maxXbr - maxXtl);
    // cv::Mat intersectionHeight = cv::max(0.0, maxYbr - maxYtl);

    // cv::Mat intersectionArea = intersectionWidth * intersectionHeight;

    // // Union
    // cv::Mat bb1Area = (bb1.col(2) - bb1.col(0)) * (bb1.col(3) - bb1.col(1));
    // cv::Mat bb2Area = (bb2.col(2) - bb2.col(0)) * (bb2.col(3) - bb2.col(1));

    // // IOU
    // cv::Mat output = intersectionArea / (bb1Area + bb2Area -
    // intersectionArea);

    return output;
}

void Sort::setIOUSolver(SolverBase *solver) { lsapSolver = solver; }
void Sort::setTracker(KalmanCreator *tracker) { trackCreator = tracker; }

void Sort::setFrameCounter(int &counter) { frameCounter = &counter; }

void Sort::update(std::vector<Metadata> &detections) {
    // FIXME: Some tracks shouldn't appear, comparison with the Python version
    // doesn't match. Maybe check the condition line 310 in sort.py ?
    hitCounter += 1;
    assert(lsapSolver != nullptr);
    assert(trackCreator != nullptr);
    cv::Mat predictedPositions = cv::Mat1d::zeros(tracklets.size(), 4);
    std::vector<int> NaNRows;
    // Step 1: Predict position for existing tracklets
    // LOG_INFO("Tracklets before prediction");
    for (int idx = 0; idx < tracklets.size(); idx++) {
        // std::cout << tracklets[idx] << std::endl;
        // std::cout << tracklets[idx].boundingBox() << std::endl;

        Metadata predicted   = tracklets[idx].prediction();
        cv::Mat predictedPos = predicted.toBbMat().reshape(0, 1);
        if (hasNan(predictedPos)) {
            NaNRows.push_back(idx);
        }
        predictedPos.copyTo(predictedPositions.row(idx));
    }
    // LOG_INFO("====");
    // Remove NaN rows
    for (std::vector<int>::iterator it = NaNRows.begin(); it != NaNRows.end();
         it++) {
        removeRow<double>(predictedPositions, *it);
    }

    // Step 2: Associate new detections to existing tracklets
    // TODO: Check if tracklets is empty and create new tracklets instead of iou
    std::vector<int> unmatchedTrackers;
    std::vector<Metadata> unmatchedDetections;
    std::vector<std::vector<int>> matched;
    associateDetToTrack(detections, predictedPositions, unmatchedTrackers,
                        unmatchedDetections, matched);

    // Step 3: Updated matched tracklets
    for (std::vector<std::vector<int>>::iterator it = matched.begin();
         it != matched.end(); it++) {
        tracklets.at(it->at(1)).update(detections.at(it->at(0)));
    }

    // Step 4: Initiate tracklets for unmatched detections
    for (int idx = 0; idx < unmatchedDetections.size(); idx++) {
        KalmanBase *newTracker = trackCreator->create();
        newTracker->load(config);
        cv::Mat state = detToSort(unmatchedDetections[idx]);
        // std::cout << state.reshape(0, 1) << std::endl;
        newTracker->setState(state);
        Tracklet newTracklet(newTracker, unmatchedDetections[idx]);
        tracklets.push_back(newTracklet);
    }
    // if ((*frameCounter == 3) || (*frameCounter == 4) || (*frameCounter == 5))
    // {
    //     LOG_INFO("Before filtering");
    //     for (auto &trk : tracklets) {
    //         std::cout << trk << std::endl;
    //         std::cout << trk.boundingBox() << std::endl;
    //     }
    //     LOG_INFO("After filtering");
    //     for (auto &trk : tracklets) {
    //         if (isCorrect(trk)) {
    //             std::cout << trk << std::endl;
    //             std::cout << trk.boundingBox() << std::endl;
    //         }
    //     }
    // }
    // DEBUG
    // if (hitCounter >= 15 && hitCounter <= 17) {
    //     LOG_INFO("==== " << hitCounter << " ====");
    //     for (auto &trk : tracklets) {
    //         if (isCorrect(trk)) {
    //             std::cout << trk << std::endl;
    //         }
    //     }
    // } else if (hitCounter > 17) {
    //     exit(EXIT_FAILURE);
    // }
    // // END DEBUG
    // LOG_INFO("Bounding boxes for frame " << *frameCounter);
    // for (auto &trk : tracklets) {
    //     std::cout << trk.boundingBox() << std::endl;
    // }

    if (save) {
        for (std::vector<Tracklet>::reverse_iterator it = tracklets.rbegin();
             it != tracklets.rend(); it++) {
            // Filter results before saving
            if (isCorrect(*it)) {
                // cv::Rect bb = it->boundingBox();
                cv::Mat bb = it->boundingBoxMat();

                // clang-format off
                trackingResult << *frameCounter                        << ", " 
                                << it->id                              << ", "
                                << bb.at<double>(0)                    << ", "
                                << bb.at<double>(1)                    << ", " 
                                << bb.at<double>(2) - bb.at<double>(0) << ", " 
                                << bb.at<double>(3) - bb.at<double>(1) << ", "
                                << "1, "
                                << "-1, "
                                << "-1 ,"
                                << "-1 " 
                                << std::endl;
                // clang-format on
            }
        }
    }
}

bool Sort::isDead(Tracklet &track) {
    if (track.timeSinceUpdate > maxAge) {
        return true;
    }
    return false;
}

bool Sort::isCorrect(Tracklet &track) {
    if ((track.timeSinceUpdate < 1) &&
        ((track.hitStreak >= minHits) || (*frameCounter <= minHits))) {
        return true;
    }
    return false;
}

void Sort::saveResults() { save = true; }

void Sort::writeTrackingResults(const std::string &filename) {
    std::ofstream file;
    file.open(filename);
    file << trackingResult.str();
    file.close();
    LOG_INFO("Saved tracking info");
}

int Sort::getFrameCnt() { return hitCounter; }

std::ostream &operator<<(std::ostream &os, Tracklet t) {
    Metadata tmp = t.getLatestDetection();
    os << "Tracklet: " << t.id << "\n"
       << "Time since update: " << t.timeSinceUpdate << " | "
       << "Hit streak hits: " << t.hitStreak << "\n"
       << "Bounding Box:\n"
       << sortToDet(tmp, t.getState(), t.id).toBb();
    return os;
}

cv::Mat detToSort(Metadata &detection) {
    // [x, y, w, h] -> [x, y, s, r, x', y', s']

    double s = detection.width * detection.height;
    double r = detection.width / detection.height;
    // clang-format off
    cv::Mat detmat = (cv::Mat_<double>(7, 1) << detection.x,
                                               detection.y, 
                                               s, 
                                               r,
                                               0,
                                               0,
                                               0);
    // clang-format on
    return detmat;
}

cv::Mat stateToPos(const cv::Mat &state) {
    // [x,y,s,r,x',y',s'] -> [x_tl, y_tl, x_br, y_br]

    double w = cv::sqrt(state.at<double>(2) * state.at<double>(3));
    double h = state.at<double>(2) / w;
    // clang-format off
    cv::Mat output = (cv::Mat1d(4, 1) << state.at<double>(0) - (w / 2.0), 
                                         state.at<double>(1) - (h / 2.0),
                                         state.at<double>(0) + (w / 2.0),
                                         state.at<double>(1) + (h / 2.0));
    // clang-format on

    return output;
}

Metadata sortToDet(Metadata &latestDet, const cv::Mat &currentPos, int id) {
    // currentPos: [x, y, s, r, x', y', s']
    // Metadata:   [x, y, w, h, prob, label]
    // w = sqrt(s * r) | h = s / w

    double width =
        cv::sqrt(currentPos.at<double>(2) * currentPos.at<double>(3));
    double height = currentPos.at<double>(2) / width;

    // clang-format off
    Metadata det(currentPos.at<double>(0),
                 currentPos.at<double>(1), 
                 height, 
                 width,
                 "ID " + std::to_string(id) + " " + latestDet.label,
                 latestDet.probability);
    // clang-format on
    return det;
}

std::vector<Metadata> Sort::getCorrectedDetections() {
    std::vector<Metadata> output;
    for (std::vector<Tracklet>::iterator it = tracklets.begin();
         it != tracklets.end(); ++it) {
        // Filtering output
        if (isCorrect(*it)) {
            cv::Mat currentState =
                it->getState().reshape(0, 1); // [x,y,s,r,x',y',s']
            Metadata latestDet  = it->getLatestDetection(); // [x,y,w,h,l,p]
            Metadata currentDet = sortToDet(latestDet, currentState,
                                            it->id); // Convert to metadata
            currentDet.setColor(it->getColor());
            output.push_back(currentDet);
        }
    }
    return output;
}

void Sort::prune() {
    tracklets.erase(
        std::remove_if(tracklets.begin(), tracklets.end(),
                       [this](Tracklet t) { return this->isDead(t); }),
        tracklets.end());
}

void Sort::associateDetToTrack(std::vector<Metadata> &detections,
                               const cv::Mat &predictedPos,
                               std::vector<int> &unmatchedTrackers,
                               std::vector<Metadata> &unmatchedDetection,
                               std::vector<std::vector<int>> &matched) {
    cv::Mat detectionsMat = cv::Mat1d::zeros(detections.size(), 4);

    if (detections.size() == 0) {
        // Nothing to do.
        return;
    }

    if (tracklets.size() == 0) {
        for (int i = 0; i < detections.size(); i++) {
            unmatchedDetection.push_back(detections.at(i));
        }
        return;
    }

    for (int idx = 0; idx < detections.size(); idx++) {
        cv::Mat detMat = detections[idx].toBbMat().reshape(0, 1);
        detMat.copyTo(detectionsMat.row(idx));
    }

    // DEBUG
    // LOG_INFO("Running LSAP solver for frame " << *frameCounter);
    // if ((*frameCounter == 3) || (*frameCounter == 4) || (*frameCounter == 5))
    // {
    //     LOG_INFO("Detections matrix going to batch iou");
    //     std::cout << detectionsMat << std::endl;
    //     LOG_INFO("Prediction matrix going into iou")
    //     std::cout << predictedPos << std::endl;
    // }
    // END DEBUG

    // Calculate IOU
    // FIXME: Size mismatch
    cv::Mat iouMat = iou(detectionsMat, predictedPos);

    // if ((*frameCounter == 3) || (*frameCounter == 4) || (*frameCounter == 5))
    // {
    //     LOG_INFO("IOU Mat")
    //     std::cout << iouMat << std::endl;
    // }

    // Solve IOU matrix
    cv::Mat assignment;
    cv::Mat indexes;
    cv::Mat matchedResult;

    lsapSolver->solve(iouMat, assignment, indexes);

    // if ((*frameCounter == 3) || (*frameCounter == 4) || (*frameCounter == 5))
    // {
    //     LOG_INFO("Detections matrix going to batch iou");

    //     LOG_INFO("Assignment result")
    //     std::cout << assignment << std::endl;
    //     LOG_INFO("Indexes result")
    //     std::cout << indexes << std::endl;
    // }

    cv::vconcat(assignment, indexes, matchedResult);

    // Iterate over matchedResult cols to have tuples of assignment and
    // indexes
    for (int col = 0; col < matchedResult.cols; col++) {
        int idx      = matchedResult.col(col).at<int>(1); // detection idx
        int assigned = matchedResult.col(col).at<int>(0); // tracker idx

        if (iouMat.at<double>(idx, assigned) < iouThreshold) {
            unmatchedDetection.push_back(detections.at(idx));
            unmatchedTrackers.push_back(assigned);
        } else {
            std::vector<int> goodMatch = {idx, assigned};
            matched.push_back(goodMatch);
        }
    }

    // Final check for unmatched detections and trackers
    // for (int i = 0; i < detections.size(); i++) {
    //     if (!findValueInMat<int>(assignment, i)) {
    //         unmatchedTrackers.push_back(i);
    //     }
    //     if (!findValueInMat<int>(indexes, i)) {
    //         unmatchedDetection.push_back(detections.at(i));
    //     }
    // }

    // Check if some detections where not matched, don't care about trackers
    for (int i = 0; i < detectionsMat.rows; i++) {
        bool found = false;
        for (int j = 0; j < indexes.cols; j++) {
            if (indexes.at<int>(j) == i) {
                found = true;
            }
        }
        if (!found) {
            unmatchedDetection.push_back(detections.at(i));
        }
    }
}

void Sort::printState() {
    for (auto it = tracklets.begin(); it != tracklets.end(); it++) {
        std::cout << *it << "\n---\n";
        std::cout << "Current state:"
                  << "\n";
        std::cout << it->getState().reshape(0, 1) << "\n===\n";
    }
}

// Clean-up
void Sort::clean() {
    hitCounter = 0;
    trackingResult.str("");
    trackingResult.clear();
    tracklets.clear();
}

// FIXME: This will not work with multithreading
int Tracklet::idCounter = 1;

Tracklet::Tracklet(KalmanBase *tracker, Metadata &firstDetection)
    : tracker(tracker), id(idCounter++), age(0), hitStreak(0),
      timeSinceUpdate(0) {
    addedHistory.push(firstDetection);
    cv::Mat1d colors(3, 1);
    cv::randu(colors, 0, 256);
    color = cv::Scalar(colors(0), colors(1), colors(2));
}

Tracklet::~Tracklet() {}

const cv::Scalar &Tracklet ::getColor() { return color; }

cv::Rect Tracklet::boundingBox() {
    //  Get the predicted current state
    cv::Mat currentState     = tracker->getState().reshape(0, 1);
    Metadata currentPosition = sortToDet(addedHistory.back(), currentState, id);
    return currentPosition.toBb();
}

cv::Mat Tracklet::boundingBoxMat() {
    //  Get the predicted current state
    cv::Mat currentState     = tracker->getState().reshape(0, 1);
    Metadata currentPosition = sortToDet(addedHistory.back(), currentState, id);
    return currentPosition.toBbMat();
}

const Metadata &Tracklet::prediction() {
    // retunrns [x, y, w, h, p, l]
    age++;
    if (timeSinceUpdate > 0) {
        hitStreak = 0;
    }
    timeSinceUpdate++;
    cv::Mat predictedStateMat = tracker->getState().reshape(0, 1);
    if (predictedStateMat.at<double>(6) + predictedStateMat.at<double>(2) <=
        0) {
        predictedStateMat.at<double>(6) *= 0.0;
        tracker->setState(
            predictedStateMat.reshape(0, KF_N)); // Update tracker info
    }
    Metadata predictedState =
        sortToDet(addedHistory.back(), tracker->predict().reshape(0, 1), id);
    predictedState.setColor(color);
    predictedHistory.push(predictedState);
    return predictedHistory.back();
}

// Returns current state in the Kalman Filter
const cv::Mat &Tracklet::getState() { return tracker->getState(); }

// Returns latest detection added to tracklet
Metadata Tracklet::getLatestDetection() { return addedHistory.back(); }

void Tracklet::update(Metadata detection) {
    timeSinceUpdate = 0;
    hitStreak += 1;

    std::queue<Metadata> empty;
    std::swap(predictedHistory, empty); // clears predictedHistory

    addedHistory.push(detection);
    cv::Mat detMat = detection.toSort();

    tracker->update(detMat);
}

// int main(int argc, char const *argv[]) {
//     Sort sort(3, 3, 50.0);
//     return 0;
// }
