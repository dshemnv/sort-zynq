#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include "genboxes.hpp"

using namespace cv;
Box::Box(Point barycenter, Size boxSize) : barycenter(barycenter), boxSize(boxSize)
{
	pos.x = 0;
	pos.y = 0;
}

Box::~Box() {}

void Box::setVelocity(Point2f vel)
{
	velocity = vel;
}

void Box::setZone(Zone zone)
{
	this->zone = zone;
}

void Box::setDestination(Point dest)
{
	destination = dest;
}

Point Box::getTopLeft()
{
	Point topLeft;
	topLeft.x = barycenter.x - (int)(boxSize.width / 2);
	topLeft.y = barycenter.y + (int)(boxSize.height / 2);
	return topLeft;
}

Point Box::getBottomRight()
{
	Point bottomRight;
	bottomRight.x = barycenter.x + (int)(boxSize.width / 2);
	bottomRight.y = barycenter.y - (int)(boxSize.height / 2);
	return bottomRight;
}

void Box::updatePosition()
{
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
	int borderWidth = 5;
	int minArea = imageSize.area() / 30;

	Point barycenter;
	Point maxDist;
	Size boxSize;
	Zone zone;

	do
	{
		barycenter.x = std::rand() % (int)(imageSize.width - borderWidth);
		barycenter.y = std::rand() % (int)(imageSize.height - borderWidth);

		int dw = imageSize.width - barycenter.x;
		int dh = imageSize.height - barycenter.y;

		if (std::min(dw, barycenter.x) == dw)
		{
			maxDist.x = dw - borderWidth;
			if (std::min(dh, barycenter.y) == dh)
			{
				maxDist.y = dh - borderWidth;
				zone = BOTTOM_RIGHT;
			}
			else
			{
				maxDist.y = barycenter.y + borderWidth;
				zone = TOP_RIGHT;
			}
		}
		else
		{
			maxDist.x = barycenter.x + borderWidth;
			if (std::min(dh, barycenter.y) == dh)
			{
				maxDist.y = dh - borderWidth;
				zone = BOTTOM_LEFT;
			}
			else
			{
				maxDist.y = barycenter.y + borderWidth;
				zone = BOTTOM_RIGHT;
			}
		}

		boxSize.height = std::rand() % maxDist.y;
		boxSize.width = std::rand() % maxDist.x;
	} while (boxSize.area() < minArea);

	Box box = Box(barycenter, boxSize);
	box.setZone(zone);
	return box;
}

Point BoxManager::genRandomDestination()
{
	Point dest;

	dest.x = std::rand() % static_cast<int>(imageSize.width - 10);
	dest.y = std::rand() % static_cast<int>(imageSize.height - 10);

	return dest;
}

void BoxManager::drawBox()
{
	box.setVelocity(Point_<float>(0.3, 0.3));
	box.setDestination(genRandomDestination());
	namedWindow("Output");

	char code;
	for (;;)
	{
		box = generateRandomBox();
		Mat canvas = Mat::zeros(imageSize, CV_8UC3);
		// box.updatePosition();
		rectangle(canvas, box.getTopLeft(), box.getBottomRight(), Scalar(0, 0, 255), 1, LINE_4);
		// rectangle(canvas, Point(10, 10), Point(30, 30), Scalar(0, 0, 255), 1, LINE_4);
		imshow("Output", canvas);
		code = (char)waitKey(100);
		if (code == 'q' || code == 'Q' || code == 27)
		{
			break;
		}
	}
}

int main(int argc, char const *argv[])
{
	srand(time(NULL));
	Size canvasSize = Size(640, 480);
	BoxManager boxm = BoxManager(canvasSize, 4);

	boxm.drawBox();
	return 0;
}
