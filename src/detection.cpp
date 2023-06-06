#include "detection.hpp"

void printDetection(detectionprops det) {
    printf("Barycenter: (%f,%f)\nHeight: %f\nWidth: %f\nLabel: "
           "%s\nProbability: %f\n",
           det.barycenter.x, det.barycenter.y, det.height, det.width, det.label,
           det.probability);
}