#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#include "lane.h"


//int main(int argc, char **argv)
//{
//	cv::VideoCapture cap("../media/rawActivity3Video.mp4");
//	cv::Mat frame, resizedFrame, annotatedFrame;
//
//	const double horizonPercentage = 0.3;
//
//	cap.read(frame);
//	cv::resize(frame, resizedFrame, cv::Size(), 0.5, 0.5);
//	lane hostLane(resizedFrame);
//
//	while (cap.read(frame))
//	{
//		cv::resize(frame, resizedFrame, cv::Size(), 0.5, 0.5);
//		hostLaneDetector(resizedFrame, annotatedFrame, hostLane, horizonPercentage);
//
//		cv::Point2i externalVehicleCenterPoint = cv::Point2i(200, 250);
//		cv::rectangle(annotatedFrame, cv::Point2i(externalVehicleCenterPoint.x - 75, externalVehicleCenterPoint.y - 50),
//			cv::Point2i(externalVehicleCenterPoint.x + 75, externalVehicleCenterPoint.y + 50), cv::Scalar(255, 0, 0), -1);
//		switch (externalVehicleLocator(hostLane, externalVehicleCenterPoint))
//		{
//		case HOST:
//		{
//			cv::putText(annotatedFrame, "VEHICLE IN LANE", cv::Point2i(350, 525), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 3);
//			break;
//		}
//		case LEFT:
//		{
//			cv::putText(annotatedFrame, "VEHICLE LEFT OF LANE", cv::Point2i(350, 525), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 3);
//			break;
//		}
//		case RIGHT:
//		{
//			cv::putText(annotatedFrame, "VEHICLE RIGHT OF LANE", cv::Point2i(350, 525), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 3);
//			break;
//		}
//		case NODETECT:
//		{
//			cv::putText(annotatedFrame, "LANE UNDEFINED", cv::Point2i(350, 525), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 3);
//			break;
//		}
//		default: break;
//		}
//
//
//
//		cv::imshow("Annotated Frame", annotatedFrame);
//		cv::waitKey(1);
//		cv::imwrite("laneDetectorAnnotatedFrame.png", annotatedFrame);
//
//
//		if (cv::waitKey(30) == 27)
//		{
//			break;
//		}
//
//	}
//
//	return 0;
//}


lane::lane(const cv::Mat &inputFrame)
{
	pts = std::vector<cv::Point2i>(4);
	lastSeenIndexes = std::vector<int>(2);
	indexesInMemory = 10;
	frameWidth = inputFrame.size().width;
	frameHeight = inputFrame.size().height;
	resetPts();
}

void lane::resetPts()
{
	setLanePts(cv::Point2i(0, frameHeight - 1), cv::Point2i(0, 0), 0);
	setLanePts(cv::Point2i(frameWidth - 1, frameHeight - 1), cv::Point2i(frameWidth - 1, 0), 1);
	lastSeenIndexes[0] = std::numeric_limits<int>::min();
	lastSeenIndexes[1] = std::numeric_limits<int>::min();
}

void lane::update()
{
	for (std::size_t i = 0; i < lastSeenIndexes.size(); ++i)
	{
		if (lastSeenIndexes[i] >= 0)
		{
			++lastSeenIndexes[i];
		}
		if (lastSeenIndexes[i] > indexesInMemory)
		{
			resetPts();
		}
	}
}

void lane::setLanePts(cv::Point2i nearPt, cv::Point2i farPt, bool rightLane)
{
	if (rightLane)
	{
		pts[0] = farPt;
		pts[3] = nearPt;
		lastSeenIndexes[rightLane] = 0;
	}
	else
	{
		pts[1] = farPt;
		pts[2] = nearPt;
		lastSeenIndexes[rightLane] = 0;
	}
}

std::vector<cv::Point2i> lane::getPts()
{
	return pts;
}

bool lane::validLaneInMemory()
{
	if (lastSeenIndexes[0] >= 0 && lastSeenIndexes[1] >= 0 &&
		lastSeenIndexes[0] < indexesInMemory && lastSeenIndexes[1] < indexesInMemory)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void hostLaneDetector(const cv::Mat &inputFrame, cv::Mat *annotatedFrame, lane *hostLane, const double horizonPercentage = 0.3)
{
	cv::Mat hsvMask, edgeFrame, hsvAndEdgeMask;
	static int yHorizon = inputFrame.size().height * horizonPercentage;
	std::vector<cv::Vec2f> lines;
	//std::vector<cv::Vec4i> lines;
	float percentXFromCenterOneWay = .25;
	float nearPointDistanceForNewLine = .05;

	int xShift = (inputFrame.size().width / 2) - 1 - percentXFromCenterOneWay*inputFrame.size().width;
	cv::Rect rectROI = cv::Rect(xShift, yHorizon,inputFrame.size().width * 2 * percentXFromCenterOneWay, inputFrame.size().height - 1 - yHorizon);

	static int centerX = inputFrame.size().width / 2;

	cv::Mat yellowWhiteColor = hlsMaskCreator(inputFrame, yHorizon, hsvMask);
	edgeImageCreator(yellowWhiteColor, edgeFrame);
	//lineFinder(edgeFrame, lines, rectROI);
	gpulineFinder(edgeFrame, lines, rectROI);
	inputFrame.copyTo(*annotatedFrame);

	double leftLaneOffset = std::numeric_limits<int>::min();
	double rightLaneOffset = std::numeric_limits<int>::max();

	//For GPU Line Detection
	/*for (size_t i = 0; i<lines.size(); ++i)
	{
		cv::Vec4i l = lines[i];
		cv::Point2i detectedNearPt, detectedFarPt;
		detectedNearPt.x = l[0];
		detectedNearPt.y = l[1];
		detectedFarPt.x = l[2];
		detectedFarPt.y = l[3];


		cv::line(*annotatedFrame, detectedNearPt, detectedNearPt, cv::Scalar(0, 0, 255), 1, 8);
		double offsetFrac = (detectedNearPt.x - centerX) / double(centerX);
		if (fabs(offsetFrac) > 0.2)
		{
			if (offsetFrac > 0 && offsetFrac < rightLaneOffset)
			{
				rightLaneOffset = offsetFrac;
				hostLane->setLanePts(detectedNearPt, detectedFarPt, true);

			}
			else if (offsetFrac < 0 && offsetFrac > leftLaneOffset)
			{
				leftLaneOffset = offsetFrac;
				hostLane->setLanePts(detectedNearPt, detectedFarPt, false);
			}
		}
		else
		{
			std::cout << "Changing Lanes!" << std::endl;
		}*/


	//}
	std::vector<cv::Point2i> placedNearPoints;
	for (std::size_t i = 0; i < lines.size(); ++i)
	{
		bool pointFound = false;
		float rho = lines[i][0], theta = lines[i][1];
		// Plot ONLY non-horizontal lines:
		if ((theta < 3 * CV_PI / 8) || (theta > 7 * CV_PI / 8) ||
			(theta > 5 * CV_PI / 8 && theta < 11 * CV_PI / 8))
		{
			cv::Point2i detectedNearPt, detectedFarPt;
			detectedFarPt.y = cvRound(yHorizon);
			detectedNearPt.y = cvRound(inputFrame.size().height - 1);
			detectedFarPt.x = cvRound((rho - detectedFarPt.y*sin(theta)) / cos(theta));
			detectedNearPt.x = cvRound((rho - detectedNearPt.y*sin(theta)) / cos(theta));
			
			if (placedNearPoints.size() > 0){
				for (int point = 0; point < placedNearPoints.size(); point++){
					if (abs(placedNearPoints[point].x - detectedNearPt.x) < inputFrame.size().width*nearPointDistanceForNewLine){
						pointFound = true;
					}
				}
			}
			if (!pointFound){
				cv::line(*annotatedFrame, detectedNearPt, detectedFarPt, cv::Scalar(0, 0, 255), 1);
				placedNearPoints.push_back(detectedNearPt);

				double offsetFrac = (detectedNearPt.x - centerX) / double(centerX);
				if (fabs(offsetFrac) > 0.2)
				{
					if (offsetFrac > 0 && offsetFrac < rightLaneOffset)
					{
						rightLaneOffset = offsetFrac;
						hostLane->setLanePts(detectedNearPt, detectedFarPt, true);

					}
					else if (offsetFrac < 0 && offsetFrac > leftLaneOffset)
					{
						leftLaneOffset = offsetFrac;
						hostLane->setLanePts(detectedNearPt, detectedFarPt, false);
					}
				}
				else
				{
					std::cout << "Changing Lanes!" << std::endl;
				}
			}
		}
	}

	if (hostLane->validLaneInMemory()) frameLaneAnnotator(*annotatedFrame, *hostLane);

	hostLane->update();
}


cv::Mat hlsMaskCreator(const cv::Mat &bgrFrame, const int yHorizon, cv::Mat &hsvMask)
{
	/*static const double yellowHueLow = 0.0;
	static const double yellowHueHigh = 0.15;
	static const double yellowLightLow = 0.6;
	static const double yellowLightHigh = 1.0;
	static const double yellowSaturationLow = 0.25;
	static const double yellowSaturationHigh = 1.0;

	static const double whiteHueLow = 0.2;
	static const double whiteHueHigh = 1.0;
	static const double whiteSaturationLow = 0.0;
	static const double whiteSaturationHigh = 0.5;
	static const double whiteValueLow = 0.6;
	static const double whiteValueHigh = 1.0;*/

	/*static const cv::Scalar yellowLineHSV_low = cv::Scalar(yellowHueLow * 179, yellowSaturationLow * 255, yellowValueLow * 255);
	static const cv::Scalar yellowLineHSV_high = cv::Scalar(yellowHueHigh * 179, yellowSaturationHigh * 255, yellowValueHigh * 255);

	static const cv::Scalar whiteLineHSV_low = cv::Scalar(whiteHueLow * 179, whiteSaturationLow * 255, whiteValueLow * 255);
	static const cv::Scalar whiteLineHSV_high = cv::Scalar(whiteHueHigh * 179, whiteSaturationHigh * 255, whiteValueHigh * 255);*/

	cv::Scalar lowerWhite(0, 150, 0);
	cv::Scalar upperWhite(255, 255, 255);
	cv::Scalar lowerYellow(10, 0, 70);
	cv::Scalar upperYellow(40, 255, 255);

	cv::Mat hslFrame, yellowHSLMask, whiteHSLMask;

	cv::cvtColor(bgrFrame, hslFrame, CV_BGR2HLS);

	cv::inRange(hslFrame, lowerYellow, upperYellow, yellowHSLMask);
	cv::inRange(hslFrame, lowerWhite, upperWhite, whiteHSLMask);
	cv::bitwise_or(yellowHSLMask, whiteHSLMask, hsvMask);
	cv::Mat mask;
	cv::cvtColor(hsvMask, mask, CV_GRAY2BGR);
	cv::Mat returnMat;
	cv::bitwise_and(bgrFrame,mask,returnMat);
	return returnMat;


	//hsvMask(cv::Rect(0, 0, bgrFrame.size().width, yHorizon)) = cv::Scalar(0);

}

void edgeImageCreator(const cv::Mat &bgrFrame, cv::Mat &edgeFrame)
{
	static const int cannyLowerThreshold = 50;
	static const int cannyHigherThreshold = 2.5 * cannyLowerThreshold; // ratio in middle of recommended range

	cv::Mat grayFrame;

	cv::cvtColor(bgrFrame, grayFrame, CV_BGR2GRAY);
	cv::Canny(grayFrame, edgeFrame, cannyLowerThreshold, cannyHigherThreshold);
}

void gpuedgeImageCreator(const cv::Mat &bgrFrame, cv::Mat &edgeFrame)
{
	static const int cannyLowerThreshold = 75;
	static const int cannyHigherThreshold = 2.5 * cannyLowerThreshold; // ratio in middle of recommended range

	cv::gpu::GpuMat frame, grayFrame, edges;

	frame.upload(bgrFrame);
	cv::gpu::cvtColor(frame, grayFrame, CV_BGR2GRAY);
	cv::gpu::Canny(grayFrame, edges, cannyLowerThreshold, cannyHigherThreshold);
	edges.download(edgeFrame);
}
void lineFinder(const cv::Mat &lineMask, std::vector<cv::Vec2f> &detectedLines, cv::Rect roi)
{
	double houghLinesLowerThreshold = 45;
	cv::Mat mask = cv::Mat::zeros(cv::Size(lineMask.size()), CV_8UC1);
	cv::Mat roiMat = mask(roi);
	roiMat.setTo(255);

	cv::bitwise_and(lineMask, mask,lineMask);

	cv::imshow("ROI", lineMask);
	cv::HoughLines(lineMask, detectedLines, 1, CV_PI / 180, houghLinesLowerThreshold);
}

void gpulineFinder(const cv::Mat &lineMask, std::vector<cv::Vec2f> &detectedLines, cv::Rect roi)
{
	double houghLinesLowerThreshold = 50;
	cv::gpu::GpuMat gpuMask, gpuROIMat, gpuLinesMask, gpuLines(cv::Size(lineMask.size()), lineMask.type());
	gpuLinesMask.upload(lineMask);
	cv::Mat mask = cv::Mat::zeros(cv::Size(lineMask.size()), CV_8UC1);
	gpuMask.upload(mask);
	gpuROIMat = gpuMask(roi);
	gpuROIMat.setTo(255);
	cv::gpu::bitwise_and(gpuLinesMask, gpuMask, gpuLinesMask);
	cv::gpu::HoughLinesBuf buf;
	//cv::gpu::HoughLinesP(gpuLinesMask, gpuLines, buf, 1, CV_PI / 180, houghLinesLowerThreshold,100);
	cv::gpu::HoughLines(gpuLinesMask, gpuLines, 1, CV_PI / 180, houghLinesLowerThreshold);
	if (!gpuLines.empty())
	{
		detectedLines.resize(gpuLines.cols);
		cv::Mat temp_Mat(1, gpuLines.cols, CV_32FC2, &detectedLines[0]);
		gpuLines.download(temp_Mat);
	}
}

void frameLaneAnnotator(cv::Mat &frame, lane &lane)
{
	cv::line(frame, lane.getPts()[0], lane.getPts()[1], cv::Scalar(0, 255, 0), 2);
	cv::line(frame, lane.getPts()[1], lane.getPts()[2], cv::Scalar(0, 255, 0), 2);
	cv::line(frame, lane.getPts()[2], lane.getPts()[3], cv::Scalar(0, 255, 0), 2);
	cv::line(frame, lane.getPts()[3], lane.getPts()[0], cv::Scalar(0, 255, 0), 2);
}


Lane externalVehicleLocator(lane &laneToCheck, const cv::Point2i externalVehicleCenterPoint)
{
	if (!laneToCheck.validLaneInMemory()) return NODETECT;

	//int inLane = cv::pointPolygonTest(laneToCheck.getPts(), externalVehicleCenterPoint, false);
	
	std::vector<cv::Point2i> pts = laneToCheck.getPts();
	cv::Point2i nearPointRight = pts[3];
	cv::Point2i nearPointLeft = pts[2];
	
	if (nearPointRight.x >= externalVehicleCenterPoint.x && externalVehicleCenterPoint.x >= nearPointLeft.x )
	{
		return HOST;
	}
	else
	{
		int laneMidX = (laneToCheck.getPts()[0].x + laneToCheck.getPts()[3].x) / 2;
		if (externalVehicleCenterPoint.x > laneMidX) return RIGHT;
		else return LEFT;
	}
}