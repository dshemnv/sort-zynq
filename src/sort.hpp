#include "detsys.hpp"
#include "kalman.hpp"
#include "lsap_solver.hpp"

class Tracklet {
  protected:
    static int idCounter;

  private:
    KalmanBase *tracker;
    std::queue<Metadata> history;

  public:
    Tracklet(KalmanBase *tracker, Metadata &firstDetection);
    ~Tracklet();
    int id;
    int hitStreak;
    int age;
    int timeSinceUpdate;
    cv::Rect boundingBox();
    const cv::Mat &prediction();
    const cv::Mat &getState();
    Metadata getLatestDetection();
    void update(Metadata &detection);
};

class Sort {
  private:
    int maxAge;
    int minHits;
    double iouThreshold;
    int frameCounter;

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
    void setIOUSolver(SolverBase *solver);
    void update(std::vector<Metadata> detections);
    bool isDead(Tracklet &track);
    cv::Mat detToSort(Metadata &detection);
    cv::Mat stateToPos(const cv::Mat &state);
    Metadata sortToDet(Metadata &latestDet, const cv::Mat &currentPos, int id);
    std::vector<Metadata> getCorrectedDetections();
    void associateDetToTrack(std::vector<Metadata> &detections,
                             const cv::Mat &predictedPos,
                             std::vector<int> &unmatchedTrackers,
                             std::vector<Metadata> &unmatchedDetections,
                             std::vector<std::vector<int>> &matched);
};
