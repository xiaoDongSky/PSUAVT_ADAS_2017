#pragma once
#include <mutex>
#include <thread>
class StereoCamera
{
public:
	StereoCamera();
	StereoCamera(int leftCamera, int rightCamera);
	~StereoCamera();
	void cleanUp();
	void getColorImages(cv::Mat *left, cv::Mat *right);
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
