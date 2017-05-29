#pragma once
#include <opencv2\opencv.hpp>
#include <opencv2\gpu\gpu.hpp>
#include "../VehicleDetectionTracking/objects.h"

void nms(const std::vector<cv::Rect>& srcRects, std::vector<cv::Rect>& resRects, double NMSThreshold, int neighbors = 0);

class GPUDetector
{
public:
	GPUDetector();
	GPUDetector(char * fileName);
	GPUDetector(char * fileName, double scaleFactor, int mergeThreshold);
	~GPUDetector();
	void setMergeThreshold(int mergeThreshold);
	void setScaleFactor(double scaleFactor);
	void loadFile(char * fileName);
	void multiScaleDetection(cv::Mat image, cv::Rect* objects,int *objNum);
	void GPUDetector::multiScaleDetection(cv::Mat image, objectTracker* tracker);
private:
	double scaleFactor;
	int minNeighbors;
	char * fileName;
};

class PedestrianDetector{
public:
	PedestrianDetector(cv::Size win_stride, double nmsThres, double scale0 = 1.05, int group_threshold = 2, double hitThreshold = 0);
	void multiScaleDetection(cv::Mat image, objectTracker* tracker);

private:
	cv::gpu::HOGDescriptor detector;
	cv::Size winStride;
	double scale;
	int groupThreshold;
	double hitThreshold;
	double NMSThreshold;
};

