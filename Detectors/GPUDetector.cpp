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
		//printf("Loaded detector: %s\n", fileName);
		this->fileName = fileName;
	}
	else {
		printf("Failed to Load Detector: %s\n", fileName);
	}
}

void GPUDetector::multiScaleDetection(cv::Mat image, cv::Rect* objects, int *objNum) {
	if (fileLoaded == true) {
		cv::gpu::GpuMat imageGPU(image);
		cv::gpu::GpuMat detectedObjs;
		cv::Mat obj_host;

		*objNum = detector.detectMultiScale(imageGPU, detectedObjs,scaleFactor,minNeighbors);
		imageGPU.release();
		detectedObjs.colRange(0, *objNum).download(obj_host);
		objects = obj_host.ptr<cv::Rect>();
	}
	else {
		printf("No File Loaded\n");
	}
}

void GPUDetector::multiScaleDetection(cv::Mat image, objectTracker* tracker){

	//printf("Tracker Mem = %x\n", tracker);
	std::vector<object> objects;
	cv::gpu::GpuMat gpuImg(image);
	cv::gpu::GpuMat gray;
	cv::gpu::GpuMat detectedObjs;
	cv::gpu::cvtColor(gpuImg, gray, CV_BGR2GRAY);
	cv::Mat objs;

	int objNum = detector.detectMultiScale(gray, detectedObjs, scaleFactor, minNeighbors);
	gpuImg.release();
	detectedObjs.colRange(0, objNum).download(objs);
	cv::Rect* rects = objs.ptr<cv::Rect>();

	for (size_t i = 0; i < objNum; ++i)
	{
		cv::Mat mask(gray.size(), CV_8UC1, cv::Scalar::all(0));
		mask(rects[i]).setTo(cv::Scalar::all(255));
		std::vector<cv::Point2f> detectedCorners;
		cv::goodFeaturesToTrack(cv::Mat(gray), detectedCorners, 100, 0.001, 0, mask);
		objects.push_back(object(rects[i], detectedCorners, 0));
	}
	if (objects.size()> 0)
		tracker->track(objects, image);
}
