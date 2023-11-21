#include "acqsys.hpp"
#include "benchmark.hpp"
#include "detsys.hpp"
#include "lsap_solver.hpp"
#include <boost/filesystem.hpp>
#include <iterator>
#include <queue>

namespace fs = boost::filesystem;

int main(int argc, char const *argv[]) {
    fs::path rootMOTFolder(argv[1]);
    if (!fs::exists(rootMOTFolder)) {
        LOG_ERR("Folder doesn't exist");
        exit(EXIT_FAILURE);
    }

    std::vector<std::string> subfolders;

    for (auto i = fs::directory_iterator(rootMOTFolder);
         i != fs::directory_iterator(); ++i) {
        subfolders.push_back(i->path().string());
    }

    // Load datasets in a vector
    std::queue<AqSysMOT> datasets;

    for (std::vector<std::string>::iterator it = subfolders.begin();
         it != subfolders.end(); ++it) {
        AqSysMOT d(*it);
        datasets.push(d);
    }

    KalmanCreatorFactory<KalmanEigen<KF_N, KF_M>> kalmanFactory;
    AuctionNaive auction(0.001);

    std::string yoloModel = argv[2];
    YOLODPU yolo(yoloModel, true);

    std::string adasLabels[3] = {"car", "person", "cycle"};
    std::string vocLabels[20] = {
        "aeroplane",   "bicycle", "bird",  "boat",      "bottle",
        "bus",         "car",     "cat",   "chair",     "cow",
        "diningtable", "dog",     "horse", "motorbike", "person",
        "pottedplant", "sheep",   "sofa",  "train",     "tvmonitor"};

    if (yoloModel == "yolov3_adas_pruned_0_9") {
        yolo.setLabels(adasLabels);
    } else if (yoloModel == "yolov3_voc_tf") {
        yolo.setLabels(vocLabels);
    } else {
        LOG_ERR("Wrong model");
        exit(EXIT_FAILURE);
    }

    // Execute benchmark
    sortzynq::MOTBenchmark bench(&datasets, &yolo, &auction, &kalmanFactory,
                                 "SORTZYNQ" + std::string("_") + yoloModel);
    bench.start(true);

    // Write results
    return 0;
}
