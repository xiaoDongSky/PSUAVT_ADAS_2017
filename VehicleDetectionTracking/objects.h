#ifndef OBJ
#define OBJ


#include <opencv2/opencv.hpp>
#include "../Detectors/globals.h"
#include <string>
#include <deque>
#include <map>


class object
{
    public:
        object(const cv::Rect &_boundingBox, const std::vector<cv::Point2f> &_corners, const int &_lastSeenIndex);
        cv::Rect boundingBox;
		cv::Point2i center;
        std::vector<cv::Point2f> corners;
        int lastSeenIndex;
		float distance;
		Lane lane;
		int objectID;

};

class objectTracker
{
    public:
		ObjectType obj;
		objectTracker(ObjectType o);
        void storeFrame(const cv::Mat &frame);
        void update();
        void track(const std::vector<object> &detectedObjects, const cv::Mat &frame);
        std::vector<object> trackedObjects;
        std::deque<cv::Mat> prevFrames;
    private:
		int numObjects = 1;
        int framesInMemory;
        int maxTrackedObjects;
        double minTrackingPercentage;
};


void frameAnnotator(cv::Mat &frame, objectTracker tracker);

#endif