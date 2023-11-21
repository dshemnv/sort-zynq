#include "benchmark.hpp"
#include <bits/stdc++.h>
#include <boost/filesystem.hpp>
#include <chrono>
#include <iostream>

using namespace sortzynq;
namespace fs  = boost::filesystem;
namespace clk = std::chrono;

MOTBenchmark::MOTBenchmark() {}

MOTBenchmark::~MOTBenchmark() {}

MOTBenchmark::MOTBenchmark(std::queue<AqSysMOT> *dataset, YOLODPU *detector,
                           SolverBase *solver, KalmanCreator *trackerFactory,
                           const std::string &name)
    : detector(detector), solver(solver), dataset(dataset),
      tracker(trackerFactory), sort(1, 3, 0.3) {
    rootFolder = "data/" + name;
    if (!fs::exists(rootFolder)) {
        fs::create_directories(rootFolder);
    }
}

void MOTBenchmark::start(bool save) {
    sort.setIOUSolver(solver);
    sort.setTracker(tracker);
    sort.saveResults();

    while (!dataset->empty()) {
        AqSysMOT *it = &dataset->front();
        it->load();
        detector->setAqsys(&*it);
        LOG_INFO("Running on " + it->getName());
        int detectionFPS = 0;
        int sortFPS      = 0;
        std::vector<int> v_detectionFPS;
        std::vector<int> v_sortFPS;
        while (!it->eof()) {
            // Advance frame
            it->getFrame();

            // Perform detection
            auto detectionStart = clk::high_resolution_clock::now();
            detector->detect();
            auto detectionStop = clk::high_resolution_clock::now();
            clk::duration<double, std::micro> detectionDuration =
                (detectionStop - detectionStart);

            int detectionFPS =
                static_cast<int>(1 / (detectionDuration.count() * 1e-6));
            v_detectionFPS.push_back(detectionFPS);

            // If there are detections, update trackers
            if (detector->getDetections().size() > 0) {
                auto sortStart = clk::high_resolution_clock::now();
                sort.update(detector->getDetections());
                auto sortStop = clk::high_resolution_clock::now();
                clk::duration<double, std::micro> sortDuration =
                    (sortStop - sortStart);

                int sortFPS =
                    static_cast<int>(1 / (sortDuration.count() * 1e-6));
                v_sortFPS.push_back(sortFPS);
            }
            printProgress(it->index(), it->size());
        }
        std::cout << std::endl;
        float avgSortFPS =
            std::accumulate(v_sortFPS.begin(), v_sortFPS.end(), 0.0) /
            v_sortFPS.size();
        float avgDetFPS =
            std::accumulate(v_detectionFPS.begin(), v_detectionFPS.end(), 0.0) /
            v_detectionFPS.size();
        LOG_INFO("Finished");
        LOG_INFO("SORT FPS: min "
                 << *min_element(v_sortFPS.begin(), v_sortFPS.end()) << " max "
                 << *max_element(v_sortFPS.begin(), v_sortFPS.end()) << " avg "
                 << avgSortFPS);
        LOG_INFO("Detector FPS: min "
                 << *min_element(v_detectionFPS.begin(), v_detectionFPS.end())
                 << " max "
                 << *max_element(v_detectionFPS.begin(), v_detectionFPS.end())
                 << " avg " << avgDetFPS);
        std::string outputFile = it->getName() + ".txt";
        fs::path outputPath    = rootFolder / fs::path(outputFile);
        sort.writeTrackingResults(outputPath.string());
        dataset->pop();
        sort.clean();
    }
}