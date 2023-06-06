#include "genboxes.hpp"
#include <cstdlib>
#include <stdio.h>
#include <unistd.h>

using namespace cv;
Box::Box(Point barycenter, Size boxSize)
    : barycenter(barycenter), boxSize(boxSize) {}

Box::~Box() {}

void Box::setVelocity(Point2f vel) { velocity = vel; }

Point2f Box::getVelocity() { return velocity; }

void Box::setZone(Zone zone) { this->zone = zone; }

Zone Box::getZone() { return zone; }

void Box::setDestination(Point dest) { destination = dest; }

Point Box::getDestination() { return destination; }

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

void Box::updatePosition() {
    std::cout << "Prev bar " << barycenter << std::endl;
    std::cout << "Destination " << destination << std::endl;
    if (barycenter.x != destination.x) {
        barycenter.x += (int)(velocity.x * destination.x);
        if (barycenter.y < destination.y) {
            barycenter.y += (int)(velocity.y * destination.y);
        } else {
            barycenter.y -= (int)(velocity.y * destination.y);
        }
    } else {
        barycenter.x -= (int)(velocity.x * destination.x);
        if (barycenter.y < destination.y) {
            barycenter.y += (int)(velocity.y * destination.y);
        } else {
            barycenter.y -= (int)(velocity.y * destination.y);
        }
    }
    std::cout << "New bar " << barycenter << std::endl;
}

BoxManager::BoxManager(Size imageSize, int randomSeed)
    : randomSeed(randomSeed), imageSize(imageSize) {
    box = generateRandomBox();
}

BoxManager::~BoxManager() {}

Box BoxManager::getBox() { return box; }

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
    std::cout << dest << std::endl;
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

void BoxManager::drawBox() {
    Point2f vel(0.03, 0.03);
    namedWindow("Output");
    box = generateRandomBox();

    box.setDestination(Point_<int>(120, 50));
    box.setVelocity(vel);
    char code;
    while (boxInCanvas(box)) {
        Point dest = box.getDestination();
        std::cout << "Dest in loop " << dest << std::endl;
        Mat canvas = Mat::zeros(imageSize, CV_8UC3);
        box.updatePosition();
        rectangle(canvas, box.getTopLeft(), box.getBottomRight(),
                  Scalar(0, 0, 255), 1, LINE_4);
        // rectangle(canvas, Point(10, 10), Point(30, 30), Scalar(0, 0, 255), 1,
        // LINE_4);
        imshow("Output", canvas);
        code = (char)waitKey(100);
        if (code == 'q' || code == 'Q' || code == 27) {
            break;
        }
    }
}

int main(int argc, char const *argv[]) {
    srand(time(NULL));
    Size canvasSize = Size(640, 480);
    BoxManager boxm = BoxManager(canvasSize, 4);

    boxm.drawBox();
    return 0;
}
