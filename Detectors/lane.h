#ifndef LANE
#define LANE
#include "globals.h"

class lane
{
public:
	lane(const cv::Mat &inputFrame);
	void resetPts();
	void update();
	void setLanePts(cv::Point2i nearPt, cv::Point2i farPt, bool rightLane);
	std::vector<cv::Point2i> getPts();
	bool validLaneInMemory();
private:
	std::vector<cv::Point2i> pts;
	std::vector<int> lastSeenIndexes;
	int indexesInMemory;
	int frameWidth;
	int frameHeight;
};
cv::Mat hlsMaskCreator(const cv::Mat &bgrFrame, const int yHorizon, cv::Mat &hsvMask);
void edgeImageCreator(const cv::Mat &bgrFrame, cv::Mat &edgeFrame);
void gpuedgeImageCreator(const cv::Mat &bgrFrame, cv::Mat &edgeFrame);
void hostLaneDetector(const cv::Mat &inputFrame, cv::Mat *annotatedFrame, lane *hostLane, const double horizonPercentage);
void lineFinder(const cv::Mat &lineMask, std::vector<cv::Vec2f> &detectedLines, cv::Rect roi);
void gpulineFinder(const cv::Mat &lineMask, std::vector<cv::Vec2f> &detectedLines, cv::Rect roi);
void frameLaneAnnotator(cv::Mat &frame, lane &hostLane);
Lane externalVehicleLocator(lane &laneToCheck, const cv::Point2i externalVehicleCenterPoint);

#endif