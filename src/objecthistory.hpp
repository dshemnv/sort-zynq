#ifndef OBJECTHISTORY_H
#define OBJECTHISTORY_H
#include <queue>
#include "detection.hpp"
#include "kalman.hpp"

class ObjectHistory
{
	typedef std::queue<detectionprops> dethist;

protected:
	static int nextid;

private:
	dethist history;
	dethist predictedHistory;
	int size;
	bool confirmed;
	bool validatd;
	bool updated;

public:
	int id;
	ObjectHistory(int maxSize);
	~ObjectHistory();
	void add(detectionprops det);
	int getSize();
	void setSize(int size);
	detectionprops predict(KalmanWrapper *predictor);
	void update(KalmanWrapper *predictor);
	dethist getHistory();
	void showHistory();
};

#endif