#include "detsys.hpp"
#include "kalman.hpp"
#include "lsap_solver.hpp"

class Tracklet {
    friend std::ostream &operator<<(std::ostream &os, Tracklet t);

  protected:
    static int idCounter;

  private:
    KalmanBase *tracker;
    std::queue<Metadata> predictedHistory;
    std::queue<Metadata> addedHistory;
    cv::Scalar color;

  public:
    Tracklet(KalmanBase *tracker, Metadata &firstDetection);
    ~Tracklet();
    int id;
    int hitStreak;
    int age;
    int timeSinceUpdate;
    cv::Rect boundingBox();
    cv::Mat boundingBoxMat();
    const Metadata &prediction();
    const cv::Mat &getState();
    Metadata getLatestDetection();
    const cv::Scalar &getColor();
    void update(Metadata detection);
};

class Sort {
  private:
    int maxAge;
    int minHits;
    double iouThreshold;
    int hitCounter;
    bool save;
    int *frameCounter;

    std::stringstream trackingResult;

    std::vector<Tracklet> tracklets;
    SolverBase *lsapSolver;
    KalmanCreator *trackCreator;

    kalmanConfig config;

  public:
    Sort(int maxAge, int minHits, double iouThreshold);
    Sort();
    ~Sort();
    cv::Mat iou(const cv::Mat &bb1, const cv::Mat &bb2);
    void setTracker(KalmanCreator *tracker);
    void setFrameCounter(int &counter);
    void setIOUSolver(SolverBase *solver);
    void update(std::vector<Metadata> &detections);
    void prune();
    bool isDead(Tracklet &track);
    bool isCorrect(Tracklet &track);
    void saveResults();
    void writeTrackingResults(const std::string &filename);
    int getFrameCnt();
    std::vector<Metadata> getCorrectedDetections();
    void associateDetToTrack(std::vector<Metadata> &detections,
                             const cv::Mat &predictedPos,
                             std::vector<int> &unmatchedTrackers,
                             std::vector<Metadata> &unmatchedDetections,
                             std::vector<std::vector<int>> &matched);
    void printState();
    void clean();
};

cv::Mat detToSort(Metadata &detection);
cv::Mat stateToPos(const cv::Mat &state);
Metadata sortToDet(Metadata &latestDet, const cv::Mat &currentPos, int id);