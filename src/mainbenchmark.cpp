#include "acqsys.hpp"
#include "benchmark.hpp"
#include "lsap_solver.hpp"
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <iterator>
#include <queue>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

int main(int argc, char const *argv[]) {

    try {
        // program options
        po::options_description desc("SORT-ZYNQ Benchmark Options");
        // clang-format off
        desc.add_options()
            ("help", "outputs help")
            ("mot_folder", po::value<std::string>(),"path to MOT folder (should end with test or train)")
            ("model", po::value<std::string>(),"YOLO model name (the actual model should be located in '/usr/share/vitis_ai'")
            ("classes", po::value<std::string>(),"classes dataset name")
        ;
        // clang-format on

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }
        if (!vm.count("mot_folder")) {
            LOG_ERR("No dataset provided as input, see usage");
            std::cout << desc << std::endl;
            exit(EXIT_FAILURE);
        }
        fs::path rootMOTFolder(vm["mot_folder"].as<std::string>());
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
            // LOG_INFO(*it);
            AqSysMOT d(*it);
            datasets.push(d);
        }

        KalmanCreatorFactory<KalmanEigen<KF_N, KF_M>> kalmanFactory;
        AuctionNaive auction(0.001);

        std::string yoloModel;
        if (!vm.count("model")) {
            LOG_INFO("No model provided, using default yolov3_voc");
            yoloModel = "yolov3_voc";
        } else {
            yoloModel = vm["model"].as<std::string>();
        }
        YOLODPU yolo(yoloModel, true);

        std::string *labels;
        if (!vm.count("classes")) {
            LOG_INFO("No classes file provided, using default VOC classes");
            labels = vocLabels;
        } else {
            if (vm["classes"].as<std::string>() == "coco") {
                labels = cocoLabels;
            } else if (vm["classses"].as<std::string>() == "voc") {
                labels = vocLabels;
            } else if (vm["classes"].as<std::string>() == "adas") {
                labels = adasLabels;
            }
        }

        yolo.setLabels(labels);

        // Execute benchmark
        sortzynq::MOTBenchmark bench(&datasets, &yolo, &auction, &kalmanFactory,
                                     "SORTZYNQ" + std::string("_") + yoloModel);
        LOG_INFO("Starting benchmark on " << rootMOTFolder << " with "
                                          << yoloModel);
        bench.start(true);
        LOG_INFO("Benchmark end");

    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        exit(EXIT_FAILURE);
    }
    return 0;
}
