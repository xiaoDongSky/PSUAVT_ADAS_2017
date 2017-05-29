#pragma once
#include "../VehicleDetectionTracking/objects.h"
#include "opencv2\opencv.hpp"
#include <iostream>
#include <fstream>
#include <ctime>
#include <Windows.h>

class csvwriter
{
public:
	csvwriter();
	~csvwriter();
	void csvwriter::writeData(double imgTime, double procTime, bool obstructed, cv::Mat frame, objectTracker stops, objectTracker vehicles, objectTracker speeds, objectTracker yields, objectTracker peds);
	void writeVehicleData(objectTracker vehicleTracker);
	void writeSignData(objectTracker signTracker);
	void writePedData(objectTracker pedTracker);

	std::ofstream csvFile;
	cv::VideoWriter videoWriter;


private:
	double lastFrameElapsedTime = 0;
	bool initFileName();
	bool initFileHeaders();

};

