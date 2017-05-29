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
#include <cstdlib>
#include "CSV\csvwriter.h"


using namespace std;



//Image Globals
cv::Mat leftFrame;			//Left Camera Frame
cv::Mat rightFrame;			//Right Camera Frame
cv::Mat leftRectified;		//Left Frame Rectified
cv::Mat rightRectified;		//Right Frame Rectified

csvwriter fileWriter;

StereoCamera cameras(3,2);	//Stereo Cameras based on index in current system
							//Surface Book: Left = 3, Right = 2

//Disparity Globals
std::thread disparityThread;
cv::Mat disparity;
cv::Mat points3D;
//cv::Mat disparityColorMap;

//Pedestrian Detection
std::thread pedestrianThread;
objectTracker pedestrianTracker(PED, .15, 3);
PedestrianDetector pedDetector(cv::Size(8, 8), .3,1.1);

//Lane Detection
//std::thread laneThread;
const double horizonPercentage = 0.75;

//Vehicle Detection
std::thread vehicleThread;
//CPUDetector vehicleDetector("./TrainedDetectors/Vehicles/CarsOnly_13Stages_HOG.xml",1.2,3);
GPUDetector vehicleDetector("./TrainedDetectors/Vehicles/Cars3.xml", 1.1, 4);
objectTracker vehicleTracker(VEHICLE, .15, 3);

//StopSign Detection
std::thread stopSignThread;
CPUDetector stopSignDetector("./TrainedDetectors/StopSigns/StopSigns_Occluded_HOG_0.95_0.2_12.xml", 1.2, 10);
objectTracker stopSignTracker(STOP, .2,1);

//SpeedSign Detection
std::thread speedSignThread;
CPUDetector speedSignDetector("./TrainedDetectors/SpeedSigns/SpeedSigns_NonOccluded_LBP_0.95_0.2_12.xml", 1.2, 10);
objectTracker speedSignTracker(SPEED, .2,1);

//YieldSign Detection
std::thread yieldSignThread;
CPUDetector yieldSignDetector("./TrainedDetectors/YieldSigns/YieldSigns_Occluded_LBP_0.9_0.1_18.xml", 1.2, 12);
objectTracker yieldSignTracker(YIELD, .2,1);




bool stop = false;			//Stop Processing

//Handle Ctrl+C event and shuts down program
BOOL WINAPI consoleHandler(DWORD signal) {

	if (signal == CTRL_C_EVENT){
		printf("Ctrl-C handled\n"); // do cleanup
		stop = true;
		cameras.cleanUp();
		fileWriter.~csvwriter();
		return 0;
	}	
}

int main(int argc, char * argv[]) {

	//Timing Values
	LARGE_INTEGER frequency;// ticks per second
	LARGE_INTEGER startTime;
	startTime.QuadPart = 0;
	LARGE_INTEGER totalStart, totalEnd;           // total ticks 
	double elapsedTime;

	if (!SetConsoleCtrlHandler(consoleHandler, TRUE)) {
		printf("\nERROR: Could not set control handler");
		system("pause");
		return 1;
	}

	void(*gpuDisp)(cv::Mat, cv::Mat, cv::Mat*) = &gpuCalculateDisparity;
	void(CPUDetector::*cpuDet)(cv::Mat, objectTracker*)  =  &CPUDetector::multiScaleDetection;
	void(GPUDetector::*gpuDet)(cv::Mat, objectTracker*) = &GPUDetector::multiScaleDetection;

	
	//Stereo Camera Values Returned from MATLAB's Stereo Camera Calibrator
	cv::Mat camMat1 = createCameraMatrix(691.9560, 449.2018 - 1.0, 687.8611, 176.7419 - 1.);
	cv::Mat camMat2 = createCameraMatrix(693.3619, 470.7558 - 1.0, 687.6097, 187.9147 - 1.0);
	cv::Mat distCoeffs1 = createDistCoefficients(0.2630, -0.5640, -0.0250, 0.0132, 0.4455);
	cv::Mat distCoeffs2 = createDistCoefficients(0.2505, -0.4060, -0.0253, 0.0181, 0.2226);
	cv::Mat R = createR(0.999988412180442, -0.000223414646173171, 0.00480890743671751, 0.000250454854091212, 0.999984159027278, -0.00562307450363253, -0.00480757498174535, -0.00562421375867022, 0.999972627346565);
	cv::Mat T = createT(-229.556109007784, 2.52610761065687, -5.04023446185492);
	
	initDisparity(camMat1, distCoeffs1, camMat2, distCoeffs2, R, T);
	QueryPerformanceFrequency(&frequency);
	Sleep(1000);
	cameras.getColorImages(&leftFrame, &rightFrame);
	lane laneDet(leftFrame);
	hostLaneDetector(leftFrame, &leftFrame, &laneDet, horizonPercentage);
	cv::imshow("Left", leftFrame);
	cv::imshow("Right", rightFrame);
	cv::moveWindow("Left", 0, 0);
	cv::moveWindow("Right", 500, 0);
	cv::waitKey(1);
	system("pause");
	QueryPerformanceCounter(&startTime);
	cameras.setStartTime(startTime);


	cv::Mat pastRight = cv::Mat::zeros(cv::Size(864, 480), CV_8UC3);
	cv::Mat pastLeft = cv::Mat::zeros(cv::Size(864, 480), CV_8UC3);
	int obstruction = 0;
	while (!stop){
		//Begin Timing
		QueryPerformanceCounter(&totalStart);
		cv::Mat annotatedFrame;
		LARGE_INTEGER time = cameras.getColorImages(&leftFrame, &rightFrame);
		double imgTime = (time.QuadPart - startTime.QuadPart) * 1000.0 / frequency.QuadPart;
		obstruction = checkObstruction(leftFrame, pastLeft, rightFrame, pastRight);
		if (obstruction == 0){

			rectifyImages(leftFrame, rightFrame, &leftRectified, &rightRectified);
			std::thread laneDetThread = std::thread(&hostLaneDetector, leftRectified, &annotatedFrame, &laneDet, horizonPercentage);//hostLaneDetector(leftRectified, annotatedFrame, laneDet, horizonPercentage);
			pedestrianThread = std::thread(&PedestrianDetector::multiScaleDetection, &pedDetector, leftRectified, &pedestrianTracker);
			laneDetThread.join();
			std::thread disparityThread = std::thread(gpuDisp, leftRectified, rightRectified, &points3D);

			disparityThread.join();

			pedestrianThread.join();
			vehicleThread = std::thread(gpuDet, &vehicleDetector, leftRectified, &vehicleTracker);
			calculateDistances(points3D, pedestrianTracker, laneDet);
			frameAnnotator(annotatedFrame, pedestrianTracker);

			vehicleThread.join();
			std::thread stopSignThread = std::thread(cpuDet, &stopSignDetector, leftRectified, &stopSignTracker);
			calculateDistances(points3D, vehicleTracker, laneDet);
			frameAnnotator(annotatedFrame, vehicleTracker);

			stopSignThread.join();
			yieldSignThread = std::thread(cpuDet, &yieldSignDetector, leftRectified, &stopSignTracker);
			calculateDistances(points3D, stopSignTracker, laneDet);
			frameAnnotator(annotatedFrame, stopSignTracker);

			yieldSignThread.join();
			speedSignThread = std::thread(cpuDet, &speedSignDetector, leftRectified, &speedSignTracker);
			calculateDistances(points3D, yieldSignTracker, laneDet);
			frameAnnotator(annotatedFrame, yieldSignTracker);

			speedSignThread.join();
			calculateDistances(points3D, speedSignTracker, laneDet);
			frameAnnotator(annotatedFrame, speedSignTracker);

			//End Timing
			QueryPerformanceCounter(&totalEnd);
			elapsedTime = (totalEnd.QuadPart - time.QuadPart) * 1000.0 / frequency.QuadPart;
			//printf("Image Time: %.4f s", imgTime);

			char buf[50];
			sprintf_s(buf, "%.4f s", imgTime / 1000.0);
			cv::putText(annotatedFrame, buf, cv::Point(780, 470), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

			cv::imshow("Left", annotatedFrame);
			cv::imshow("Right", rightRectified);
			fileWriter.writeData(imgTime/1000.0, elapsedTime/1000, false, annotatedFrame, stopSignTracker, vehicleTracker, speedSignTracker, yieldSignTracker, pedestrianTracker);
			
		}
		else if (obstruction == 1){
			QueryPerformanceCounter(&totalEnd);
			elapsedTime = (totalEnd.QuadPart - time.QuadPart) * 1000.0 / frequency.QuadPart;
			//printf("Image Time: %.4f s", imgTime);

			char buf[50];
			sprintf_s(buf, "%.4f s", imgTime / 1000.0);
			cv::putText(leftFrame, buf, cv::Point(780, 470), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
			cv::putText(leftFrame, "Left Frame Obscured", cv::Point(10, 470), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
			fileWriter.writeData(imgTime/1000.0, elapsedTime/1000,true,leftFrame,stopSignTracker,vehicleTracker,speedSignTracker, yieldSignTracker, pedestrianTracker);
			cv::imshow("Left", leftFrame);
			cv::imshow("Right", rightFrame);
		}
		else{
			QueryPerformanceCounter(&totalEnd);
			elapsedTime = (totalEnd.QuadPart - time.QuadPart) * 1000.0 / frequency.QuadPart;
			//printf("Image Time", imgTime);

			char buf[50];
			sprintf_s(buf, "%.4f s", imgTime / 1000.0);
			cv::putText(leftFrame, buf, cv::Point(780, 470), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
			cv::putText(leftFrame, "Right Frame Obscured", cv::Point(10, 470), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
			fileWriter.writeData(imgTime/1000.0, elapsedTime/1000, true, leftFrame, stopSignTracker, vehicleTracker, speedSignTracker, yieldSignTracker, pedestrianTracker);
			cv::imshow("Left", leftFrame);
			cv::imshow("Right", rightFrame);
		}
		pastLeft = leftFrame;
		pastRight = rightFrame;
		vehicleTracker.update();
		pedestrianTracker.update();
		stopSignTracker.update();
		yieldSignTracker.update();
		speedSignTracker.update();
		cv::waitKey(1);
	}
		
	system("pause");
	return 0;
}