#include "objecthistory.hpp"

int ObjectHistory::nextid = 0;

ObjectHistory::ObjectHistory(int maxSize) : size(maxSize), id(++nextid)
{
}

ObjectHistory::~ObjectHistory()
{
}

void ObjectHistory::setSize(int size)
{
	this->size = size;
}

int ObjectHistory::getSize()
{
	return history.size();
}

void ObjectHistory::add(detectionprops det)
{
	history.push(det);
}

void ObjectHistory::predict(KalmanWrapper *predictor)
{
	Mat prediction = predictor->predict();
}