#include "detection.hpp"

void printDetection(detectionprops det) {
    printf("Barycenter: (%f,%f)\nHeight: %f\nWidth: %f\nLabel: "
           "%s\nProbability: %f\n",
           det.barycenter.x, det.barycenter.y, det.height, det.width, det.label,
           det.probability);
}

detectionprops detpropFromBox(Box &box, const char *label, float probability) {
    detectionprops detprop = {.height      = box.getBoxMeasures().height,
                              .width       = box.getBoxMeasures().width,
                              .barycenter  = box.getCoordinates(),
                              .label       = label,
                              .probability = probability};

    return detprop;
}

Box boxFromDetprop(detectionprops &detprop) {
    return Box(detprop.barycenter, cv::Size(detprop.height, detprop.width));
}