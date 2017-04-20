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
		printf("Loaded detector: %s\n", fileName1);
		this->fileName1 = fileName1;
	}
	else {
		printf("Failed to Load Detector: %s\n", fileName1);
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
