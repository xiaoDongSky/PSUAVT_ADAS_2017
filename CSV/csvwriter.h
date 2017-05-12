#pragma once
#include "../VehicleDetectionTracking/objects.h"


class csvwriter
{
public:
	csvwriter();
	~csvwriter();

private:
	static bool initFileName();
	static bool initFileHeaders();

};

