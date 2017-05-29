#pragma once
#include <mutex>
#include <thread>
#include <Windows.h>
class StereoCamera
{
public:
	StereoCamera();
	StereoCamera(int leftCamera, int rightCamera);
	~StereoCamera();
	void cleanUp();
	LARGE_INTEGER getColorImages(cv::Mat *left, cv::Mat *right);
	void setStartTime(LARGE_INTEGER st);
	void getGrayImages(cv::Mat *left, cv::Mat *right);

private:
	void updateFrames();
	cv::VideoCapture leftCam;
	cv::VideoCapture rightCam;
	cv::Mat leftFrame;
	cv::Mat leftFrameGray;
	cv::Mat rightFrame;
	cv::Mat rightFrameGray;
};
