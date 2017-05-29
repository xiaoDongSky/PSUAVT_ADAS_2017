#include "CPUDetector.h"

cv::CascadeClassifier detector;
bool fileLoaded1 = false;
double scaleFactor1=1.1;
int minNeighbors1=3;
char * fileName1;

CPUDetector::CPUDetector()
{
}

CPUDetector::CPUDetector(char * fileName1) {
	if (fileLoaded1 = detector.load(fileName1)) {
		printf("Loaded detector: %s\n", fileName1);
		this->fileName1 = fileName1;
	}
	else {
		printf("Failed to Load Detector: %s\n", fileName1);
	}
}

CPUDetector::CPUDetector(char * fileName1, double scaleFactor1, int mergeThreshold) {
	if (fileLoaded1 = detector.load(fileName1)) {
		printf("Loaded detector: %s\n", fileName1);
		this->fileName1 = fileName1;
	}
	else {
		printf("Failed to Load Detector: %s\n", fileName1);
	}
	this->scaleFactor1 = scaleFactor1;
	this->minNeighbors1 = mergeThreshold;
}

CPUDetector::~CPUDetector()
{
}

void CPUDetector::setMergeThreshold(int mergeThreshold) {
	this->minNeighbors1 = mergeThreshold;
}

void CPUDetector::setScaleFactor(double scaleFactor1) {
	this->scaleFactor1 = scaleFactor1;
}

void CPUDetector::loadFile(char * fileName1) {
	if (fileLoaded1 = detector.load(fileName1)) {
		//printf("Loaded detector: %s\n", fileName1);
		this->fileName1 = fileName1;
	}
	else {
		//printf("Failed to Load Detector: %s\n", fileName1);
	}
}

int CPUDetector::multiScaleDetection(cv::Mat image, std::vector<cv::Rect>* objects ) {
	cv::Mat copyImage = image.clone();
	if (fileLoaded1 == true) {
		detector.detectMultiScale(copyImage, *objects, scaleFactor1, minNeighbors1);
		return objects->size();
	}
	else {
		printf("No File Loaded\n");
		return -1;
	}
}

void multiScaleDetection(cv::Mat image, std::vector<object> &objects){
	objects.clear();
	std::vector<cv::Rect> detectedBoundingBoxes;

	detector.detectMultiScale(image, detectedBoundingBoxes, scaleFactor1, minNeighbors1);

	for (size_t i = 0; i < detectedBoundingBoxes.size(); ++i)
	{
		cv::Mat mask(image.size(), CV_8UC1, cv::Scalar::all(0));
		mask(detectedBoundingBoxes[i]).setTo(cv::Scalar::all(255));
		std::vector<cv::Point2f> detectedCorners;
		cv::goodFeaturesToTrack(image, detectedCorners, 100, 0.001, 0, mask);
		objects.push_back(object(detectedBoundingBoxes[i], detectedCorners, 0));
	}
}

void CPUDetector::multiScaleDetection(cv::Mat image, objectTracker* tracker){

	//printf("Tracker Mem = %x\n", tracker);
	std::vector<object> objects;
	cv::Mat gray;
	cv::cvtColor(image, gray, CV_BGR2GRAY);
	
	std::vector<cv::Rect> detectedBoundingBoxes;

	detector.detectMultiScale(gray, detectedBoundingBoxes, scaleFactor1, minNeighbors1);

	for (size_t i = 0; i < detectedBoundingBoxes.size(); ++i)
	{
		cv::Mat mask(gray.size(), CV_8UC1, cv::Scalar::all(0));
		mask(detectedBoundingBoxes[i]).setTo(cv::Scalar::all(255));
		std::vector<cv::Point2f> detectedCorners;
		cv::goodFeaturesToTrack(gray, detectedCorners, 100, 0.001, 0, mask);
		objects.push_back(object(detectedBoundingBoxes[i], detectedCorners, 0));
	}
	if (objects.size()> 0)
		tracker->track(objects,image);
}
