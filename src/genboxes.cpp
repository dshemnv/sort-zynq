#include <stdio.h>
#include <cstdlib>
#include "genboxes.hpp"

using namespace cv;
Box::Box(Point topLeft, Point bottomRight) : topLeft(topLeft), bottomRight(bottomRight)
{
	calcBarycentre();
}

Box::~Box() {}

void Box::calcBarycentre()
{
	barycentre.x = bottomRight.x - topLeft.x;
	barycentre.y = bottomRight.y - topLeft.y;
}

Point Box::getTopLeft()
{
	return topLeft;
}

Point Box::getBottomRight()
{
	return bottomRight;
}

BoxManager::BoxManager(Size imageSize, int randomSeed) : randomSeed(randomSeed), imageSize(imageSize)
{
	box = generateRandomBox();
}

BoxManager::~BoxManager() {}

Box BoxManager::getBox()
{
	return box;
}

Box BoxManager::generateRandomBox()
{
	float scaleFactor = 1.0 / 3.0;
	int borderWidth = 5;
	int maxSizeHeight = std::rand() % static_cast<int>(scaleFactor * this->imageSize.height);
	int maxSizeWidth = std::rand() % static_cast<int>(scaleFactor * this->imageSize.width);

	Point topLeft;
	Point bottomRight;

	topLeft.x = std::rand() % static_cast<int>(this->imageSize.width - borderWidth);
	topLeft.y = std::rand() % static_cast<int>(this->imageSize.height - borderWidth);

	int signX = 1;
	int signY = 1;
	if (topLeft.x + maxSizeWidth > static_cast<int>(scaleFactor * this->imageSize.width))
	{
		signX = -1;
	}
	if (topLeft.y + maxSizeHeight > static_cast<int>(scaleFactor * this->imageSize.height))
	{
		signY = -1;
	}

	bottomRight.x = abs(topLeft.x + signX * maxSizeWidth);
	bottomRight.y = abs(topLeft.y + signY * maxSizeHeight);

	std::cout << "( " << topLeft.x << "," << topLeft.y << " )" << std::endl;
	std::cout << "( " << bottomRight.x << "," << bottomRight.y << " )" << std::endl;

	Box box = Box(topLeft, bottomRight);
	return box;
}

void BoxManager::generateRandomTrajectory()
{
}

int main(int argc, char const *argv[])
{
	srand(time(NULL));
	Size canvasSize = Size(640, 480);
	BoxManager boxm = BoxManager(canvasSize, 4);
	Mat canvas = Mat::zeros(canvasSize, CV_8UC3);

	rectangle(canvas, boxm.getBox().getTopLeft(), boxm.getBox().getBottomRight(), Scalar(0, 0, 255), 1, LINE_4);
	imshow("Image", canvas);
	waitKey(0);
	return 0;
}
