#include "kalman.hpp"
#include <cstdlib>
#include <stdio.h>
#include <unistd.h>

using namespace cv;
Box::Box() {}
Box::Box(Point barycenter, Size boxSize)
    : barycenter(barycenter), boxSize(boxSize), blocked(false) {}

Box::~Box() {}

bool Box::isBlocked() { return blocked; }
void Box::setVelocity(Point2f vel) { velocity = vel; }

Point2f Box::getVelocity() { return velocity; }

void Box::setZone(Zone zone) { currentZone = zone; }
Zone Box::getZone() { return currentZone; }
bool Box::boxInZone(Zone zone) {
    return (zone == destinationZone ? true : false);
}

void Box::setDestinationZone(Zone zone) { destinationZone = zone; }
Zone Box::getDestinationZone() { return destinationZone; }

Size Box::getBoxMeasures() {
    Point tl = getTopLeft();
    Point br = getBottomRight();
    return Size(br.y - tl.y, br.x - tl.x);
}

void Box::setDestination(Point dest) { destination = dest; }

Point Box::getDestination() { return destination; }
Point Box::getCoordinates() { return barycenter; }

Point Box::getTopLeft() {
    Point topLeft;
    topLeft.x = barycenter.x - (int)(boxSize.width / 2);
    topLeft.y = barycenter.y + (int)(boxSize.height / 2);
    return topLeft;
}

Point Box::getBottomRight() {
    Point bottomRight;
    bottomRight.x = barycenter.x + (int)(boxSize.width / 2);
    bottomRight.y = barycenter.y - (int)(boxSize.height / 2);
    return bottomRight;
}

void Box::move() {
    Point previous_pos = barycenter;
    if (destination.x < barycenter.x) {
        barycenter.x -= velocity.x;
        barycenter.x =
            barycenter.x < destination.x ? destination.x : barycenter.x;
    } else if (destination.x > barycenter.x) {
        barycenter.x += velocity.x;
        barycenter.x =
            barycenter.x > destination.x ? destination.x : barycenter.x;
    }
    if (destination.y < barycenter.y) {
        barycenter.y -= velocity.y;
        barycenter.y =
            barycenter.y < destination.y ? destination.y : barycenter.y;
    } else if (destination.y > barycenter.y) {
        barycenter.y += velocity.y;
        barycenter.y =
            barycenter.y > destination.y ? destination.y : barycenter.y;
    }
    // std::cout << "Current position " << barycenter << " Destination "
    //           << destination << std::endl;
    if (barycenter == previous_pos) {
        blocked = true;
    } else {
        blocked = false;
    }
}

BoxManager::BoxManager() {}
BoxManager::BoxManager(Size imageSize, int randomSeed)
    : randomSeed(randomSeed), imageSize(imageSize) {
    namedWindow("Output");
    canvas = Mat::zeros(imageSize, CV_8UC3);
}

BoxManager::~BoxManager() {}

std::vector<Box> BoxManager::getBoxes() { return boxes; }

void BoxManager::addBox(Box box) { boxes.push_back(box); }

void BoxManager::show() { imshow("Output", canvas); }

void BoxManager::cleanCanvas() { canvas = Mat::zeros(imageSize, CV_8UC3); }

Box BoxManager::generateRandomBox() {
    int borderWidth = 5;
    int minArea     = imageSize.area() / 30;

    Point barycenter;
    Point maxDist;
    Size boxSize;
    Zone zone;

    do {
        barycenter.x = std::rand() % (int)(imageSize.width - borderWidth);
        barycenter.y = std::rand() % (int)(imageSize.height - borderWidth);

        int dw = imageSize.width - barycenter.x;
        int dh = imageSize.height - barycenter.y;

        if (std::min(dw, barycenter.x) == dw) {
            maxDist.x = dw - borderWidth;
            if (std::min(dh, barycenter.y) == dh) {
                maxDist.y = dh - borderWidth;
                zone      = BOTTOM_RIGHT;
            } else {
                maxDist.y = barycenter.y + borderWidth;
                zone      = TOP_RIGHT;
            }
        } else {
            maxDist.x = barycenter.x + borderWidth;
            if (std::min(dh, barycenter.y) == dh) {
                maxDist.y = dh - borderWidth;
                zone      = BOTTOM_LEFT;
            } else {
                maxDist.y = barycenter.y + borderWidth;
                zone      = BOTTOM_RIGHT;
            }
        }

        boxSize.height = std::rand() % maxDist.y;
        boxSize.width  = std::rand() % maxDist.x;
    } while (boxSize.area() < minArea);

    Box box = Box(barycenter, boxSize);
    box.setZone(zone);
    return box;
}

Point BoxManager::genRandomDestination() {
    Point dest;

    dest.x = std::rand() % (imageSize.width - 10);
    dest.y = std::rand() % (imageSize.height - 10);
    return dest;
}

Point BoxManager::genRandomDestination(const Zone zone) {
    Point dest;

    switch (zone) {
    case TOP_LEFT:
        dest.x = std::rand() % (imageSize.width / 2);
        dest.y = std::rand() % (imageSize.height / 2);
        break;
    case TOP_RIGHT:
        dest.x = std::rand() % (imageSize.width / 2) + (imageSize.width / 2);
        dest.y = std::rand() % (imageSize.height / 2);
        break;
    case BOTTOM_LEFT:
        dest.x = std::rand() % (imageSize.width / 2);
        dest.y = std::rand() % (imageSize.height / 2) + (imageSize.height / 2);
        break;
    case BOTTOM_RIGHT:
        dest.x = std::rand() % (imageSize.width / 2) + (imageSize.width) / 2;
        dest.y = std::rand() % (imageSize.height / 2) + (imageSize.height / 2);
        break;
    default:
        break;
    }
    return dest;
}

bool BoxManager::boxInCanvas(Box &box) {
    Point2f br = box.getBottomRight();
    Point2f tl = box.getTopLeft();

    if ((tl.x <= 0) || (tl.y <= 0)) {
        return false;
    } else if ((br.x >= imageSize.width) || (br.y >= imageSize.height)) {
        return false;
    }
    return true;
}

void BoxManager::drawBox(Box &box, Scalar color) {
    setZone(box);
    if (box.isBlocked() || !boxInCanvas(box) ||
        box.getCoordinates() == box.getDestination()) {
        updateBoxPosition(box);
    }
    box.move();
    rectangle(canvas, box.getTopLeft(), box.getBottomRight(), color, 1, LINE_4);
}

void BoxManager::setZone(Box &box) {
    Point currentLocation = box.getCoordinates();

    int dw = imageSize.width - currentLocation.x;
    int dh = imageSize.height - currentLocation.y;

    if (std::min(dw, currentLocation.x) == dw) {
        if (std::min(dh, currentLocation.y) == dh) {
            box.setZone(BOTTOM_RIGHT);
        } else {
            box.setZone(TOP_RIGHT);
        }
    } else {
        if (std::min(dh, currentLocation.y) == dh) {
            box.setZone(BOTTOM_LEFT);
        } else {
            box.setZone(TOP_LEFT);
        }
    }
}

void BoxManager::updateBoxPosition(Box &box) {
    std::vector<Zone> possibleDirections({Zone::TOP_LEFT, Zone::TOP_RIGHT,
                                          Zone::BOTTOM_LEFT,
                                          Zone::BOTTOM_RIGHT});

    std::remove(possibleDirections.begin(), possibleDirections.end(),
                box.getZone());

    // Get a random new direction
    const Zone direction =
        possibleDirections[std::rand() % possibleDirections.size()];

    // Get a random new destination in zone
    const Point dest = genRandomDestination(direction);
    box.setDestinationZone(direction);
    box.setDestination(dest);
}

// int main(int argc, char const *argv[]) {
//     srand(time(NULL));
//     Size canvasSize = Size(640, 480);
//     BoxManager boxm = BoxManager(canvasSize, 4);

//     Box box1 = boxm.generateRandomBox();
//     box1.setVelocity(Point2f(13, 13));

//     Box box2 = boxm.generateRandomBox();
//     box2.setVelocity(Point2f(13, 13));
//     while (true) {
//         boxm.cleanCanvas();
//         boxm.drawBox(box1);
//         boxm.drawBox(box2);
//         boxm.show();
//         char code = (char)waitKey(50);
//         if (code == 'q' || code == 'Q' || code == 27) {
//             break;
//         }
//     }

//     return 0;
// }
