#pragma once
#include <urg/Urg_driver.h>

struct LRFPoint {
	float x;
	float y;
	LRFPoint()
	{
		x = 0.0f;
		y = 0.0f;
	}
	LRFPoint(float x_, float y_)
	{
		x = x_;
		y = y_;
	}
};

class LRFSensor
{
public:
	LRFSensor(int argc, char *argv[]);

	int grab(void);
	bool retrieve(std::vector<LRFPoint> &LRF_data);

	bool isOpened = false;
private:
	qrk::Urg_driver urg;
	std::vector<long> data;

	// double radian = urg.index2rad(int(i));
	// index2cos : cos(radian)
	std::vector<double> index2cos;
	// index2sin : sin(radian)
	std::vector<double> index2sin;

};

