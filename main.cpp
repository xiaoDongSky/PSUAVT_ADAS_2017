
//#include "ImageCapture.h"
#include <opencv2\opencv.hpp>
#include <opencv2\gpu\gpu.hpp>
#include <thread>
#include <time.h>

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
	cv::gpu::CascadeClassifier_GPU HOG2;
	cv::VideoCapture cam(0);
	cv::VideoCapture cam2(1);
	cv::CascadeClassifier HOG;
	
	//HOG.load("CarsOnly_5Stages_HOG.xml");
	HOG2.load("haarcascade_eye.xml");

	while (1) {
		cam.read(leftFrame);
		cv::Mat gray;
		cv::cvtColor(leftFrame,gray, cv::COLOR_BGR2GRAY);
		//cam2.read(rightFrame);
		std::vector<cv::Rect> pedsLeft;
		cv::gpu::GpuMat pedsGpuLeft;
		//std::vector<cv::Rect> pedsRight;
		clock_t begin = clock();
		cv::gpu::GpuMat gpuImg;
		gpuImg.upload(gray);
		int objNum = HOG2.detectMultiScale(gpuImg, pedsGpuLeft,2,4);
		printf("Num Objs: %d\n", objNum);
		HOG.detectMultiScale(leftFrame, pedsLeft);
		//HOG.detectMultiScale(rightFrame, pedsRight);
		cv::Mat obj_host;
		pedsGpuLeft.colRange(0, objNum).download(obj_host);
		cv::Rect* peds = obj_host.ptr<cv::Rect>();
		for (int i = 0; i < objNum; i++) {
			cv::rectangle(leftFrame,peds[i],cv::Scalar(255,0,0));
		}
		/*for (int i = 0; i < pedsRight.size(); i++) {
			cv::rectangle(leftFrame, pedsRight[i], cv::Scalar(255, 0, 0));
		}*/
		cv::imshow("Camera Left", leftFrame);
		clock_t after = clock();
		printf("Elapsed Time: %f\n", (double)(after-begin)/CLOCKS_PER_SEC);

		//cv::imshow("Camera Right", rightFrame);
		cvWaitKey(1);
	}

}