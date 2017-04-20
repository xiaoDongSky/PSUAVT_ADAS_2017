#pragma once
#include <opencv2\opencv.hpp>
#include <opencv2\gpu\gpu.hpp>

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

private:
	double scaleFactor;
	int minNeighbors;
	char * fileName;
};

