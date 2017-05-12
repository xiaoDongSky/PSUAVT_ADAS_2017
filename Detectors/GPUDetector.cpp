#include "GPUDetector.h"

cv::gpu::CascadeClassifier_GPU detector;
bool fileLoaded = false;
double scaleFactor = 1.1;
int minNeighbors = 3;
char * fileName;

GPUDetector::GPUDetector()
{
}

GPUDetector::GPUDetector(char * fileName) {
	if (fileLoaded = detector.load(fileName)) {
		printf("Loaded detector: %s\n", fileName);
		this->fileName = fileName;
	}
	else {
		printf("Failed to Load Detector: %s\n", fileName);
	}
}

GPUDetector::GPUDetector(char * fileName, double scaleFactor, int mergeThreshold) {
	if (fileLoaded = detector.load(fileName)) {
		printf("Loaded detector: %s\n", fileName);
		this->fileName = fileName;
	}
	else {
		printf("Failed to Load Detector: %s\n", fileName);
	}
	this->scaleFactor = scaleFactor;
	this->minNeighbors = mergeThreshold;
}

GPUDetector::~GPUDetector()
{
}

void GPUDetector::setMergeThreshold(int mergeThreshold) {
	this->minNeighbors = mergeThreshold;
}

void GPUDetector::setScaleFactor(double scaleFactor) {
	this->scaleFactor = scaleFactor;
}

void GPUDetector::loadFile(char * fileName) {
	if (fileLoaded = detector.load(fileName)) {
		printf("Loaded detector: %s\n", fileName);
		this->fileName = fileName;
	}
	else {
		printf("Failed to Load Detector: %s\n", fileName);
	}
}

void GPUDetector::multiScaleDetection(cv::Mat image, cv::Rect* objects, int *objNum) {
	if (fileLoaded == true) {
		cv::gpu::GpuMat imageGPU;
		cv::gpu::GpuMat detectedObjs;
		cv::Mat obj_host;

		imageGPU.upload(image);
		*objNum = detector.detectMultiScale(imageGPU, detectedObjs,scaleFactor,minNeighbors);
		imageGPU.release();
		detectedObjs.colRange(0, *objNum).download(obj_host);
		objects = obj_host.ptr<cv::Rect>();
	}
	else {
		printf("No File Loaded\n");
	}
}
