#ifndef OBJECTHISTORY_H
#define OBJECTHISTORY_H
#include <queue>
#include "detection.hpp"
#include "kalman.hpp"

class ObjectHistory
{
protected:
	static int nextid;

private:
	std::queue<detectionprops> history;
	std::queue<detectionprops> predictedHistory;
	int size;
	bool confirmed;
	bool validatd;

public:
	int id;
	ObjectHistory(int maxSiz);
	~ObjectHistory();
	void add(detectionprops det);
	int getSize();
	void setSize(int size);
	void predict(KalmanWrapper *predictor);
};

#endif