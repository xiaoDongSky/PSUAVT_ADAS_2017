#include "GPUDetector.h"



void nms(const std::vector<cv::Rect>& srcRects, std::vector<cv::Rect>& resRects, double NMSThreshold, int neighbors = 0);

PedestrianDetector::PedestrianDetector(cv::Size win_stride, double nmsThres, double scale0, int group_threshold, double hitThres){
	detector = cv::gpu::HOGDescriptor(cv::Size(64, 128), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9, -1.0, 0.2, true, 48);
	detector.setSVMDetector(cv::gpu::HOGDescriptor::getDefaultPeopleDetector());
	winStride = win_stride;
	scale = scale0;
	groupThreshold = group_threshold;
	hitThreshold = hitThres;
	NMSThreshold = nmsThres;	
}


void PedestrianDetector::multiScaleDetection(cv::Mat image, objectTracker* tracker){
	std::vector<object> objects;
	std::vector<cv::Rect> detectedObjects;
	cv::Mat gray;
	cv::cvtColor(image, gray, CV_BGR2GRAY);
	cv::gpu::GpuMat gpuImgGray(gray);


	
	detector.detectMultiScale(gpuImgGray, detectedObjects, hitThreshold, winStride, cv::Size(0,0), scale, groupThreshold);

	gpuImgGray.release();
	
	//std::vector<cv::Rect> filteredObjects = detectedObjects;
	std::vector<cv::Rect> filteredObjects;
	nms(detectedObjects, filteredObjects,NMSThreshold);
	for (size_t i = 0; i < filteredObjects.size(); ++i)
	{
		cv::Mat mask(image.size(), CV_8UC1, cv::Scalar::all(0));
		mask(filteredObjects[i]).setTo(cv::Scalar::all(255));
		std::vector<cv::Point2f> detectedCorners;
		cv::goodFeaturesToTrack(gray, detectedCorners, 100, 0.001, 0, mask);
		objects.push_back(object(filteredObjects[i], detectedCorners, 0));
	}
	if (objects.size()>0)
		tracker->track(objects, image);

}


//std::vector<cv::Rect> vectorizedNMS(const std::vector<cv::Rect> srcRects, float thresh)
//{
//	
//}

 void nms(const std::vector<cv::Rect>& srcRects, std::vector<cv::Rect>& resRects, double NMSThreshold, int neighbors)
{
	resRects.clear();

	const size_t size = srcRects.size();
	if (!size)
	{
		return;
	}

	// Sort the bounding boxes by the bottom - right y - coordinate of the bounding box
	std::multimap<int, size_t> idxs;
	for (size_t i = 0; i < size; ++i)
	{
		idxs.insert(std::pair<int, size_t>(srcRects[i].br().y, i));
	}

	// keep looping while some indexes still remain in the indexes list
	while (idxs.size() > 0)
	{
		// grab the last rectangle
		auto lastElem = --std::end(idxs);
		const cv::Rect& rect1 = srcRects[lastElem->second];

		int neigborsCount = 0;

		idxs.erase(lastElem);

		for (auto pos = std::begin(idxs); pos != std::end(idxs);)
		{
			// grab the current rectangle
			const cv::Rect& rect2 = srcRects[pos->second];

			float intArea = (rect1 & rect2).area();
			float unionArea = rect1.area() + rect2.area() - intArea;
			float overlap = intArea / unionArea;

			// if there is sufficient overlap, suppress the current bounding box
			if (overlap > NMSThreshold)
			{
				pos = idxs.erase(pos);
				++neigborsCount;
			}
			else
			{
				++pos;
			}
		}
		if (neigborsCount >= neighbors)
		{
			resRects.push_back(rect1);
		}
	}
}

