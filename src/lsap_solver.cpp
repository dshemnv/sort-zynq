#include "lsap_solver.hpp"
#include <iostream>

AuctionNaive::AuctionNaive() {}

AuctionNaive::AuctionNaive(double eps) : eps(eps) {}

AuctionNaive::~AuctionNaive() {}

void AuctionNaive::solve(const cv::Mat &costMat, cv::Mat &result,
                         cv::Mat &indexes) {
    nAgents  = costMat.rows;
    nObjects = costMat.cols;

    array<double> costMatA;
    costMatA.cols = costMat.cols;
    costMatA.rows = costMat.rows;
    costMatA.data = new double[costMatA.rows * costMatA.cols];
    std::memcpy(costMatA.data, costMat.ptr<double>(),
                sizeof(double) * costMat.rows * costMat.cols);

    matrix_t matType;
    if (nAgents == nObjects) {
        matType = MEQN;
    } else if (nAgents > nObjects) {
        matType = MGN;
    } else if (nAgents < nObjects) {
        matType = MLN;
    }

    assignments<double> res;
    assignment<double> assig = {.agent = -1, .object = -1, .value = -1};
    res.n_assignment         = 0;
    res.is_empty             = true;
    int nIter                = 0;
    if (matType == MGN) {
        res.size   = nAgents;
        res.result = new assignment<double>[res.size];
        for (int i = 0; i < res.size; i++) {
            res.result[i] = assig;
        }
        array<double> tcostMatA;
        init<double>(&tcostMatA, nObjects, nAgents, 0);
        transpose<double>(&costMatA, &tcostMatA);
        solve_jacobi<double>(&tcostMatA, eps, &res, matType, &nIter);
        delete[] tcostMatA.data;
    } else {
        res.size         = nObjects;
        res.n_assignment = 0;
        res.result       = new assignment<double>[res.size];
        for (int i = 0; i < res.size; i++) {
            res.result[i] = assig;
        }
        solve_jacobi<double>(&costMatA, eps, &res, matType, &nIter);
    }

    array<int> agent2object;
    array<int> indexes_res;

    init<int>(&agent2object, 1, res.n_assignment, -1);
    init<int>(&indexes_res, 1, res.n_assignment, -1);

    assignements_to_arrays<double>(&res, &agent2object, &indexes_res, matType);

    cv::Mat tmp  = cv::Mat(1, agent2object.cols, CV_32SC1, agent2object.data);
    cv::Mat tmpi = cv::Mat(1, indexes_res.cols, CV_32SC1, indexes_res.data);
    result       = tmp.clone();
    indexes      = tmpi.clone();

    delete[] res.result;
    delete[] agent2object.data;
    delete[] indexes_res.data;
    delete[] costMatA.data;
}
