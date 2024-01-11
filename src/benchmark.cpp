#include "benchmark.hpp"
#include "gui.hpp"
#include <bits/stdc++.h>
#include <boost/filesystem.hpp>
#include <chrono>
#include <iostream>
#ifdef YOLODPU
#include <vitis/ai/profiling.hpp>
#endif

using namespace sortzynq;
namespace fs  = boost::filesystem;
namespace clk = std::chrono;

MOTBenchmark::MOTBenchmark() {}

MOTBenchmark::~MOTBenchmark() {}

MOTBenchmark::MOTBenchmark(std::queue<AqSysMOT> *dataset, DetSys *detector,
                           SolverBase *solver, KalmanCreator *trackerFactory,
                           const std::string &name)
    : detector(detector), solver(solver), dataset(dataset),
      tracker(trackerFactory), sort(1, 3, 0.3), showImg(false) {
    rootFolder = "data/" + name;
    if (!fs::exists(rootFolder)) {
        fs::create_directories(rootFolder);
    }
}

void MOTBenchmark::start(bool save) {
    sort.setIOUSolver(solver);
    sort.setTracker(tracker);
    if (save) {
        sort.saveResults();
    }

    std::stringstream fpsStats;
    while (!dataset->empty()) {
        AqSysMOT *it = &dataset->front();
        it->load();
        detector->setAqsys(&*it);
        sort.setFrameCounter(it->frameCounter());
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

            // As in SORT original paper, only "person" at min 50%
            std::vector<Metadata> detections =
                detector->getDetections("person", 0.5);

            // Update trackers (even if detections is empty)
            auto sortStart = clk::high_resolution_clock::now();
#ifdef YOLODPU
            __TIC__(SORT_UPDATE)
#endif
            sort.update(detections);
#ifdef YOLODPU
            __TOC__(SORT_UPDATE)
#endif
            auto sortStop = clk::high_resolution_clock::now();

            clk::duration<double, std::micro> sortDuration =
                (sortStop - sortStart);

            int sortFPS = static_cast<int>(1 / (sortDuration.count() * 1e-6));
            // Don't record value when there are no detections.
            if (!detections.empty()) {
                v_sortFPS.push_back(sortFPS);
            }
            std::vector<Metadata> corrected = sort.getCorrectedDetections();
            if (showImg) {
                GUI gui(*it, "demo");
                std::vector<Metadata> correctedDetections =
                    sort.getCorrectedDetections();
                gui.drawFromDetections(detections);
                gui.show();
                char c = (char)cv::waitKey(1);
                if (c == 27) {
                    exit(EXIT_SUCCESS);
                }
            }
            sort.prune();
            printProgress(it->index(), it->size());
        }
        std::cout << std::endl;
        float avgSortFPS =
            std::accumulate(v_sortFPS.begin(), v_sortFPS.end(), 0.0) /
            v_sortFPS.size();
        float avgDetFPS =
            std::accumulate(v_detectionFPS.begin(), v_detectionFPS.end(), 0.0) /
            v_detectionFPS.size();
        LOG_INFO("Finished processing " << it->getName());
        LOG_INFO("SORT FPS: min "
                 << *min_element(v_sortFPS.begin(), v_sortFPS.end()) << " max "
                 << *max_element(v_sortFPS.begin(), v_sortFPS.end()) << " avg "
                 << avgSortFPS);
        LOG_INFO("Detector FPS: min "
                 << *min_element(v_detectionFPS.begin(), v_detectionFPS.end())
                 << " max "
                 << *max_element(v_detectionFPS.begin(), v_detectionFPS.end())
                 << " avg " << avgDetFPS);
        if (save) {

            fpsStats << it->getName() << ":\n"
                     << "  det_fps:"
                     << "\n"
                     << "    min: "
                     << *min_element(v_detectionFPS.begin(),
                                     v_detectionFPS.end())
                     << "\n"
                     << "    max: "
                     << *max_element(v_detectionFPS.begin(),
                                     v_detectionFPS.end())
                     << "\n"
                     << "    mean: " << avgDetFPS << "\n"
                     << "  sort_fps:"
                     << "\n"
                     << "    min: "
                     << *min_element(v_sortFPS.begin(), v_sortFPS.end()) << "\n"
                     << "    max: "
                     << *max_element(v_sortFPS.begin(), v_sortFPS.end()) << "\n"
                     << "    mean: " << avgSortFPS << "\n";
            std::string outputFile = it->getName() + ".txt";
            fs::path outputPath    = rootFolder / fs::path(outputFile);
            sort.writeTrackingResults(outputPath.string());
        }

        dataset->pop();
        sort.clean();
    }
    if (save) {
        fs::path fpsOutputFile = rootFolder / fs::path("fps_stats.yml");
        std::ofstream fpsFile(fpsOutputFile.string());
        fpsFile << fpsStats.str();
    }
}

void sortzynq::MOTBenchmark::show() { showImg = true; }