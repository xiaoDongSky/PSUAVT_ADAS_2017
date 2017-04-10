
#include "ImageCapture.h"
#include <thread>

using namespace std;

cv::Mat leftFrame;
cv::Mat rightFrame;

int main(int argc, char * argv[]) {
	thread vehicleDetection;
	thread stopSignDetection;
	thread yiedlSignDetection;
	thread speedLimitDetection;
	thread lineDetection;
	thread pedestrianDetection;

	cv::VideoCapture cam(0);
	cv::VideoCapture cam2(1);

	while (1) {
		cam.read(leftFrame);
		cam2.read(rightFrame);
		cv::imshow("Camera Left", leftFrame);
		cv::imshow("Camera Right", rightFrame);
		cvWaitKey(1);
	}

}