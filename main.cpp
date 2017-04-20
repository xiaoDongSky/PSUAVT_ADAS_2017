
//#include "ImageCapture.h"
#include <opencv2\gpu\gpu.hpp>
#include <opencv2\opencv.hpp>
#include <time.h>
#include <Windows.h>
#include "StereoCalculation.h"
#include "ImageCapture.h"
#include "Detectors\CPUDetector.h"
#include "Detectors\GPUDetector.h"


using namespace std;



//Image Globals
cv::Mat leftFrame;			//Left Camera Frame
cv::Mat leftFrameGray;		//Left Camera Frame Grayscale
cv::Mat rightFrame;			//Right Camera Frame
cv::Mat rightFrameGray;		//Right Camera Frame Grayscale
cv::Mat leftRectified;		//Left Frame Rectified
cv::Mat rightRectified;		//Right Frame Rectified


StereoCamera cameras(0,1);	//Stereo Cameras based on index in current system
							//Surface Book: Left = 2, Right = 3

//Disparity Globals
std::thread disparityThread;
cv::Mat disparity;
cv::Mat points3D;
cv::Mat disparityColorMap;




std::thread vehicleThread;
std::thread pedestrianThread;
std::thread stopSignThread;
std::thread yieldSignThread;


bool stop = false;			//Stop Processing

//Handle Ctrl+C event and shuts down program
BOOL WINAPI consoleHandler(DWORD signal) {

	if (signal == CTRL_C_EVENT){
		printf("Ctrl-C handled\n"); // do cleanup
		stop = true;
		cameras.cleanUp();
		return 0;
	}	
}

int main(int argc, char * argv[]) {

	//Timing Values
	LARGE_INTEGER frequency;        // ticks per second
	LARGE_INTEGER t1, t2;           // ticks
	double elapsedTime;

	if (!SetConsoleCtrlHandler(consoleHandler, TRUE)) {
		printf("\nERROR: Could not set control handler");
		system("pause");
		return 1;
	}
	
	cv::Mat camMat1 = createCameraMatrix(586.9294, 410.3327 - 1.0, 582.0715, 221.8771 - 1.);
	cv::Mat camMat2 = createCameraMatrix(587.1801, 381.0929 - 1.0, 582.0565, 216.1757 - 1.0);
	cv::Mat distCoeffs1 = createDistCoefficients(0.1492, -0.3730, -0.0053, -0.0019, 0.2049);
	cv::Mat distCoeffs2 = createDistCoefficients(0.1392, -0.3392, -9.9481e-5, -0.0061, 0.1296);
	cv::Mat R = createR(0.9998, -0.0029, -0.0175, 0.0032, 0.998, 0.0192, 0.0175, -0.0193, 0.9997);
	cv::Mat T = createT(-226.6373, 0.3245, 4.4172);
	
	initDisparity(camMat1, distCoeffs1, camMat2, distCoeffs2, R, T);
	//CPUDetector stopSignDetector("C:/Users/andre/Documents/Andrew's_Docs/Ecocar/EcoCAR_Y3/PSUAVT_ADAS_2017/StopSign_Occluded_HOG_99_30_13.xml", 1.3, 4);
	CPUDetector yieldSignDetector("C:/Users/andre/Documents/Andrew's_Docs/Ecocar/EcoCAR_Y3/PSUAVT_ADAS_2017/YieldSigns_NonOccluded_HOG_0.9_0.1_11.xml", 1.3, 4);
	GPUDetector faceDetector("C:/Users/andre/Documents/Andrew's_Docs/Ecocar/EcoCAR_Y3/PSUAVT_ADAS_2017/haarcascade_frontalface_alt.xml", 1.3, 4);
	std::vector<cv::Rect> stopSigns;
	std::vector<cv::Rect> yieldSigns;
	cv::Rect * faces;
	int numFaces;
	QueryPerformanceFrequency(&frequency);
	Sleep(1000);
	cameras.getColorImages(&leftFrame, &rightFrame);
	cv::imshow("Left", leftFrame);
	//cv::imshow("Right", rightFrame);

	cv::moveWindow("Left", 0, 0);
	//cv::moveWindow("Right", 0, 500);
	cv::waitKey(1);
	system("pause");

	while (!stop){
		//Begin Timing
		QueryPerformanceCounter(&t1);

		cameras.getColorImages(&leftFrame, &rightFrame);
		rectifyImages(leftFrame, rightFrame, &leftRectified, &rightRectified);
		
		void (*noColorMap)(cv::Mat, cv::Mat, cv::Mat*, cv::Mat*) = &calculateDisparity;
		disparityThread = std::thread(noColorMap,leftRectified,rightRectified,&disparity,&points3D);
		stopSignThread = std::thread(&GPUDetector::multiScaleDetection, faceDetector, leftRectified, faces,&numFaces);
		
		//stopSignThread = std::thread(&CPUDetector::multiScaleDetection, stopSignDetector, leftRectified, &stopSigns);
		disparityThread.join();
		stopSignThread.join();
		
		//yieldSignThread = std::thread(&CPUDetector::multiScaleDetection, yieldSignDetector, leftRectified, &yieldSigns);

		//yieldSignThread.join();

		for (int i = 0; i < stopSigns.size(); i++) {
				cv::rectangle(leftRectified, (stopSigns)[i], cv::Scalar(255, 0, 0),4);
		}
		for (int i = 0; i < yieldSigns.size(); i++) {
			cv::rectangle(leftRectified, (yieldSigns)[i], cv::Scalar(0,255, 0),4);
		}
		for (int i = 0; i < numFaces; i++) {
			cv::rectangle(leftRectified,faces[i],cv::Scalar(255,0,0));
		}

		cv::imshow("Left", leftRectified);
		cv::waitKey(1);
		//End Timing
		QueryPerformanceCounter(&t2);
		elapsedTime = (t2.QuadPart - t1.QuadPart)*1000.0 / frequency.QuadPart;
		printf("Total Time Elapsed: %f ms\n", elapsedTime);
	}


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
		//*for (int i = 0; i < pedsRight.size(); i++) {
		//	cv::rectangle(leftFrame, pedsRight[i], cv::Scalar(255, 0, 0));
		//}*/
		//cv::imshow("Camera Left", leftFrame);
		//clock_t after = clock();
		//printf("Elapsed Time: %f\n", (double)(after-begin)/CLOCKS_PER_SEC);

	/*for (size_t i = 0; i < detectedBoundingBoxes.size(); ++i)
	{
		cv::Mat mask(inputGrayFrame.size(), CV_8UC1, cv::Scalar::all(0));
		mask(detectedBoundingBoxes[i]).setTo(cv::Scalar::all(255));
		std::vector<cv::Point2f> detectedCorners;
		cv::goodFeaturesToTrack(inputGrayFrame, detectedCorners, 100, 0.001, 0, mask);
		detectedObjects.push_back(object(detectedBoundingBoxes[i], detectedCorners, 0));
	}*/

		
	system("pause");
	return 0;
}