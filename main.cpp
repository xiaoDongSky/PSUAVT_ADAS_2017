//#include "ImageCapture.h"
#include <opencv2\gpu\gpu.hpp>
#include <opencv2\opencv.hpp>
#include <time.h>
#include <Windows.h>
#include "StereoCalculation.h"
#include "ImageCapture.h"
#include "Detectors\CPUDetector.h"
#include "Detectors\GPUDetector.h"
#include "Detectors\lane.h"


using namespace std;



//Image Globals
cv::Mat leftFrame;			//Left Camera Frame
cv::Mat leftFrameGray;		//Left Camera Frame Grayscale
cv::Mat rightFrame;			//Right Camera Frame
cv::Mat rightFrameGray;		//Right Camera Frame Grayscale
cv::Mat leftRectified;		//Left Frame Rectified
cv::Mat rightRectified;		//Right Frame Rectified
cv::Mat leftRectifiedGray;		//Left Frame Rectified
cv::Mat rightRectifiedGray;		//Right Frame Rectified

StereoCamera cameras(3,2);	//Stereo Cameras based on index in current system
							//Surface Book: Left = 2, Right = 3

//Disparity Globals
std::thread disparityThread;
cv::Mat disparity;
cv::Mat points3D;
//cv::Mat disparityColorMap;

//Pedestrian Detection
std::thread pedestrianThread;
objectTracker pedestrianTracker(PED);
PedestrianDetector pedDetector(cv::Size(8, 8), .3,1.1);

//Lane Detection
std::thread laneThread;
const double horizonPercentage = 0.5;

//Vehicle Detection
std::thread vehicleThread;
CPUDetector vehicleDetector("./TrainedDetectors/Vehicles/Vehicles_NonOccluded_HOG_0.9_0.3_8.xml",1.1,6);
objectTracker vehicleTracker(VEHICLE);

//StopSign Detection
std::thread stopSignThread;
CPUDetector stopSignDetector("./TrainedDetectors/StopSigns/StopSigns_Occluded_HOG_0.8_0.2_10.xml", 1.1, 6);
objectTracker stopSignTracker(STOP);

//SpeedSign Detection
std::thread speedSignThread;
CPUDetector speedSignDetector();
objectTracker speedSignTracker(SPEED);

//YieldSign Detection
std::thread yieldSignThread;
CPUDetector yieldSignDetector;
objectTracker yieldSignTracker(YIELD);


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
	LARGE_INTEGER totalStart, totalEnd;           // total ticks 
	double elapsedTime;

	if (!SetConsoleCtrlHandler(consoleHandler, TRUE)) {
		printf("\nERROR: Could not set control handler");
		system("pause");
		return 1;
	}

	void(*gpuDisp)(cv::Mat, cv::Mat, cv::Mat*) = &gpuCalculateDisparity;
	void(CPUDetector::*vehDet)(cv::Mat, objectTracker*)  =  &CPUDetector::multiScaleDetection;

	
	//Stereo Camera Values Returned from MATLAB's Stereo Camera Calibrator
	cv::Mat camMat1 = createCameraMatrix(586.9294, 410.3327 - 1.0, 582.0715, 221.8771 - 1.);
	cv::Mat camMat2 = createCameraMatrix(587.1801, 381.0929 - 1.0, 582.0565, 216.1757 - 1.0);
	cv::Mat distCoeffs1 = createDistCoefficients(0.1492, -0.3730, -0.0053, -0.0019, 0.2049);
	cv::Mat distCoeffs2 = createDistCoefficients(0.1392, -0.3392, -9.9481e-5, -0.0061, 0.1296);
	cv::Mat R = createR(0.9998, -0.0029, -0.0175, 0.0032, 0.998, 0.0192, 0.0175, -0.0193, 0.9997);
	cv::Mat T = createT(-226.6373, 0.3245, 4.4172);
	
	initDisparity(camMat1, distCoeffs1, camMat2, distCoeffs2, R, T);
	QueryPerformanceFrequency(&frequency);
	Sleep(1000);
	cameras.getColorImages(&leftFrame, &rightFrame);
	lane laneDet(leftFrame);
	hostLaneDetector(leftFrame, leftFrame, laneDet, horizonPercentage);
	cv::imshow("Left", leftFrame);

	cv::moveWindow("Left", 0, 0);
	cv::waitKey(1);
	system("pause");

	while (!stop){
		//Begin Timing
		QueryPerformanceCounter(&totalStart);
		cv::Mat annotatedFrame;
		cameras.getColorImages(&leftFrame, &rightFrame);
		rectifyImages(leftFrame, rightFrame, &leftRectified, &rightRectified);
		hostLaneDetector(leftRectified, annotatedFrame, laneDet, horizonPercentage);
		pedestrianThread = std::thread(&PedestrianDetector::multiScaleDetection, &pedDetector, leftRectified, &pedestrianTracker);
		vehicleThread = std::thread(vehDet, &vehicleDetector, leftRectified, &vehicleTracker);
		disparityThread = std::thread(gpuDisp, leftRectified, rightRectified, &points3D);
		
		disparityThread.join();
		pedestrianThread.join();
		calculateDistances(points3D, pedestrianTracker,laneDet);
		frameAnnotator(annotatedFrame, pedestrianTracker);
		vehicleThread.join();
		calculateDistances(points3D, vehicleTracker, laneDet);
		frameAnnotator(annotatedFrame, vehicleTracker);
		cv::imshow("Left", annotatedFrame);
		printf("Main Tracker Mem = %x\n", &vehicleTracker);
		cv::waitKey(1);


		//End Timing
		QueryPerformanceCounter(&totalEnd);
		elapsedTime = (totalEnd.QuadPart - totalStart.QuadPart)*1000.0 / frequency.QuadPart;
		printf("Total Time Elapsed: %f ms\n", elapsedTime);
		vehicleTracker.update();
		pedestrianTracker.update();
	}
		
	system("pause");
	return 0;
}