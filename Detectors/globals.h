#ifndef GLOBAL
#define GLOBAL
#include <opencv2\opencv.hpp>

#define WIDTH 864
#define HEIGHT 480


enum ObjectType{
	VEHICLE, STOP, YIELD, PED, SPEED
};

enum Lane{
	NODETECT = 0,
	LEFT = 1,
	HOST = 2,
	RIGHT = 3
};

enum Detected{
	NO = 0,
	YES = 1
};

#endif