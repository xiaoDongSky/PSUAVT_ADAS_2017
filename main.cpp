
//#include "ImageCapture.h"
#include <opencv2\gpu\gpu.hpp>
#include <opencv2\opencv.hpp>
#include <time.h>
#include "ImageCapture.h"
#include <Windows.h>
using namespace std;

cv::Mat leftFrame;
cv::Mat rightFrame;
bool stop = false;

BOOL WINAPI consoleHandler(DWORD signal) {

	if (signal == CTRL_C_EVENT){
		printf("Ctrl-C handled\n"); // do cleanup
		stop = true;
		return 0;
	}	
}

int main(int argc, char * argv[]) {

	if (!SetConsoleCtrlHandler(consoleHandler, TRUE)) {
		printf("\nERROR: Could not set control handler");
		return 1;
	}
	cv::gpu::CascadeClassifier_GPU HOG2;
	cv::CascadeClassifier HOG;
	StereoCamera* cameras = new StereoCamera(2,3);
	
	Sleep(1000);
	HOG.load("CarsOnly_13Stages_HOG.xml");
	HOG2.load("haarcascade_eye.xml");

	while (!stop) {
		cameras->getColorImages(&leftFrame, &rightFrame);

		cv::imshow("Left", leftFrame);
		cv::imshow("Right", rightFrame);
		cv::waitKey(1);

		//cv::Mat gray;
		//cv::cvtColor(leftFrame,gray, cv::COLOR_BGR2GRAY);
		////cam2.read(rightFrame);
		//std::vector<cv::Rect> pedsLeft;
		//cv::gpu::GpuMat pedsGpuLeft;
		////std::vector<cv::Rect> pedsRight;
		//clock_t begin = clock();
		//cv::gpu::GpuMat gpuImg;
		//gpuImg.upload(gray);
		//int objNum = HOG2.detectMultiScale(gpuImg, pedsGpuLeft,2,4);
		//printf("Num Objs: %d\n", objNum);
		//HOG.detectMultiScale(leftFrame, pedsLeft);
		////HOG.detectMultiScale(rightFrame, pedsRight);
		//cv::Mat obj_host;
		//pedsGpuLeft.colRange(0, objNum).download(obj_host);
		//cv::Rect* peds = obj_host.ptr<cv::Rect>();
		//for (int i = 0; i < objNum; i++) {
		//	cv::rectangle(leftFrame,peds[i],cv::Scalar(255,0,0));
		//}
		///*for (int i = 0; i < pedsRight.size(); i++) {
		//	cv::rectangle(leftFrame, pedsRight[i], cv::Scalar(255, 0, 0));
		//}*/
		//cv::imshow("Camera Left", leftFrame);
		//clock_t after = clock();
		//printf("Elapsed Time: %f\n", (double)(after-begin)/CLOCKS_PER_SEC);

		////cv::imshow("Camera Right", rightFrame);
		//cvWaitKey(1);
	}
	cameras->cleanUp();
	return 0;
}