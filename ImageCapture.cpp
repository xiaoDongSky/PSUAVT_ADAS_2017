#include "ImageCapture.h"
#include <Windows.h>
#include <dshow.h>
#include <mutex>
#include <thread>

class StereoCamera
{
public:
	StereoCamera(int leftCamera, int rightCamera);
	StereoCamera();
	~StereoCamera();

	cv::Mat leftFrame;
	cv::Mat leftFrameGray;
	cv::Mat rightFrame;
	cv::Mat rightFrameGray;

private:
	void updateFrames();

	std::mutex imageLock;
	std::thread cameraCaptureThread;
	cv::VideoCapture leftCam;
	cv::VideoCapture rightCam;
};

StereoCamera::StereoCamera()
{
}

/**
leftCamera - Left camera device number (As user must know what value this is)
rightCamera - Right camera device number

Initializes cameras and begins camera capture threads
*/
StereoCamera::StereoCamera(int leftCamera, int rightCamera)
{
	leftCam = cv::VideoCapture(leftCamera);
	rightCam = cv::VideoCapture(rightCamera);
}

StereoCamera::~StereoCamera()
{
	leftCam.release();
	rightCam.release();

}

void StereoCamera::updateFrames(){
	imageLock.lock();
	leftCam.s
}

