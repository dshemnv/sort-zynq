#ifndef LSAP_SOLVER_H
#define LSAP_SOLVER_H
#include "auction.hpp"
#include <opencv2/core/core.hpp>

class SolverBase {
  private:
    std::string name;

  public:
    virtual ~SolverBase(){};
    virtual void solve(const cv::Mat &costMat, cv::Mat &result,
                       cv::Mat &indexes) = 0;
};

class AuctionNaive : public SolverBase {
  private:
    double eps;
    int nAgents;
    int nObjects;
    cv::Mat1d prices;
    cv::Mat1b agentsMask;
    cv::Mat1s agentToObject;
    cv::Mat1s objectToAgent;

  public:
    AuctionNaive();
    AuctionNaive(double eps);
    ~AuctionNaive();
    void solve(const cv::Mat &costMat, cv::Mat &result, cv::Mat &indexes);
};

#endif