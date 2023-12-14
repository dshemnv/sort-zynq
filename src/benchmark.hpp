#ifndef _BENCHMARK_HPP
#define _BENCHMARK_HPP
#include "acqsys.hpp"
#include "detsys.hpp"
#include "lsap_solver.hpp"
#include "sort.hpp"
#include <queue>

namespace sortzynq {

struct benchResults {
    std::vector<int> fps;
};

class MOTBenchmark {
  private:
    benchResults results;
    Sort sort;
    SolverBase *solver;
    DetSys *detector;
    KalmanCreator *tracker;
    std::queue<AqSysMOT> *dataset;
    std::string rootFolder;
    bool showImg;

  public:
    MOTBenchmark();
    MOTBenchmark(std::queue<AqSysMOT> *dataset, DetSys *detector,
                 SolverBase *lsap_solver, KalmanCreator *trackerFactory,
                 const std::string &name);
    ~MOTBenchmark();
    void start(bool save);
    void show();
    void save();
};
} // namespace sortzynq
#endif