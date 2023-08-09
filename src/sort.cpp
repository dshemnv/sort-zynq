#include "sort.hpp"

Sort::Sort(int maxAge, int minHits, double iouThreshold)
    : maxAge(maxAge), minHits(minHits), iouThreshold(iouThreshold),
      lsapSolver(nullptr), trackCreator(nullptr) {

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

            output.at<double>(newDetIdx, trackPredIdx) =
                intersectionArea / (bb1Area + bb2Area - intersectionArea);
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

void Sort::update(std::vector<Metadata> detections) {
    frameCounter += 1;
    assert(lsapSolver != nullptr);
    assert(trackCreator != nullptr);
    cv::Mat predictedPositions = cv::Mat1d::zeros(tracklets.size(), 4);
    // Step 1: Predict position for existing tracklets
    for (int idx = 0; idx < tracklets.size(); idx++) {
        cv::Mat predictedState = tracklets[idx].prediction();
        cv::Mat predictedPos   = stateToPos(predictedState).reshape(0, 1);
        predictedPos.copyTo(predictedPositions.row(idx));
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
         it != matched.end(); ++it) {
        tracklets.at(it->at(1)).update(detections.at(it->at(0)));
    }

    // Step 4: Initiate tracklets for unmatched detections
    for (int idx = 0; idx < unmatchedDetections.size(); idx++) {
        KalmanBase *newTracker = trackCreator->create(KF_N, KF_M);
        newTracker->load(config);
        cv::Mat state = detToSort(unmatchedDetections[idx]);
        newTracker->setState(state);
        Tracklet newTracklet(newTracker, unmatchedDetections[idx]);
        tracklets.push_back(newTracklet);
    }
    // Step 5: Clean dead tracklets
    for (std::vector<Tracklet>::iterator it = tracklets.begin();
         it != tracklets.end(); ++it) {
        if (isDead(*it)) {
            tracklets.erase(it);
        }
    }
}

bool Sort::isDead(Tracklet &track) {
    if (track.timeSinceUpdate > maxAge) {
        return true;
    } else if (track.timeSinceUpdate < 1) {
        if ((track.hitStreak >= minHits) || (frameCounter <= minHits)) {
            return false;
        }
    }
    return false;
}

cv::Mat Sort::detToSort(Metadata &detection) {
    // [x, y, w, h] -> [x, y, s, r, x', y', s']

    double s = detection.width * detection.height;
    double r = detection.width / detection.height;
    // clang-format off
    cv::Mat detmat = (cv::Mat_<double>(KF_N, 1) << detection.x,
                                               detection.y, 
                                               s, 
                                               r,
                                               0,
                                               0,
                                               0);
    // clang-format on
    return detmat;
}

cv::Mat Sort::stateToPos(const cv::Mat &state) {
    // [x,y,s,r,x',y',s'] -> [x_tl, y_tl, x_br, y_br]

    double w = sqrt(state.at<double>(2) * state.at<double>(3));
    double h = state.at<double>(2) / w;
    // clang-format off
    cv::Mat output = (cv::Mat1d(4, 1) << state.at<double>(0) - (w / 2.f), 
                                         state.at<double>(1) - (h / 2.f),
                                         state.at<double>(0) + (w / 2.f),
                                         state.at<double>(1) + (h / 2.f));
    // clang-format on

    return output;
}

Metadata Sort::sortToDet(Metadata &latestDet, const cv::Mat &currentPos,
                         int id) {

    double height = currentPos.at<double>(3) - currentPos.at<double>(1);
    double width  = currentPos.at<double>(2) - currentPos.at<double>(0);

    Metadata det(currentPos.at<double>(0) + (width / 2),
                 currentPos.at<double>(2) + (height / 2), height, width,
                 "ID " + std::to_string(id) + " " + latestDet.label,
                 latestDet.probability);

    return det;
}

std::vector<Metadata> Sort::getCorrectedDetections() {
    std::vector<Metadata> output;
    for (std::vector<Tracklet>::iterator it = tracklets.begin();
         it != tracklets.end(); ++it) {
        cv::Mat currentState = it->getState();
        cv::Mat currentPos   = stateToPos(currentState);
        Metadata latestDet   = it->getLatestDetection();
        Metadata currentDet  = sortToDet(latestDet, currentPos, it->id);

        output.push_back(currentDet);
    }
    return output;
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

    // Calculate IOU
    // FIXME: Size mismatch
    cv::Mat iouMat = iou(detectionsMat, predictedPos);

    // Solve IOU matrix
    cv::Mat assignment;
    cv::Mat indexes;
    cv::Mat matchedResult;

    lsapSolver->solve(iouMat, assignment, indexes);

    cv::vconcat(assignment, indexes, matchedResult);

    // Iterate over matchedResult cols to have tuples of assignment and indexes
    for (int col = 0; col < matchedResult.cols; col++) {
        int assigned = matchedResult.col(col).at<int>(0);
        int idx      = matchedResult.col(col).at<int>(1);

        if (iouMat.at<double>(idx, assigned) < iouThreshold) {
            unmatchedDetection.push_back(detections.at(idx));
            unmatchedTrackers.push_back(assigned);
        } else {
            std::vector<int> goodMatch = {idx, assigned};
            matched.push_back(goodMatch);
        }
    }

    // Final check for unmatched detections and trackers
    for (int i = 0; i < detections.size(); i++) {
        if (!findValueInMat<int>(indexes, i)) {
            unmatchedTrackers.push_back(i);
        }
        if (!findValueInMat<int>(assignment, i)) {
            unmatchedDetection.push_back(detections.at(i));
        }
    }
}

// FIXME: This will not work with multithreading
int Tracklet::idCounter = 0;

Tracklet::Tracklet(KalmanBase *tracker, Metadata &firstDetection)
    : tracker(tracker), id(idCounter++), age(0), hitStreak(0),
      timeSinceUpdate(0) {
    history.push(firstDetection);
}

Tracklet::~Tracklet() {}

// Returns a bounding box
cv::Rect Tracklet::boundingBox() {
    cv::Rect bb;
    Metadata currentPosition = history.back();
    bb.x                     = currentPosition.x - (currentPosition.width / 2);
    bb.y                     = currentPosition.y - (currentPosition.height / 2);
    bb.width                 = currentPosition.width;

    return bb;
}

const cv::Mat &Tracklet::prediction() {
    age++;
    if (timeSinceUpdate > 0) {
        hitStreak = 0;
    }
    timeSinceUpdate++;
    return tracker->predict();
}

//  Return latest state
const cv::Mat &Tracklet::getState() { return tracker->getState(); }

Metadata Tracklet::getLatestDetection() { return history.back(); }

void Tracklet::update(Metadata &detection) {
    timeSinceUpdate = 0;
    hitStreak += 1;
    cv::Mat detMat = detection.toSort();
    tracker->update(detMat);
    history.push(detection);
}

// int main(int argc, char const *argv[]) {
//     Sort sort(3, 3, 50.0);
//     return 0;
// }