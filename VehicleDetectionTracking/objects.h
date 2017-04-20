#include <opencv2/opencv.hpp>
#include <string>
#include <deque>
#include <map>

class object
{
    public:
        object(const cv::Rect &_boundingBox, const std::vector<cv::Point2f> &_corners, const int &_lastSeenIndex);
        cv::Rect boundingBox;
        std::vector<cv::Point2f> corners;
        int lastSeenIndex;
};

class objectTracker
{
    public:
        objectTracker();
        void storeFrame(const cv::Mat &frame);
        void update();
        void track(const std::vector<object> &detectedObjects, const cv::Mat &frame);
        std::vector<object> trackedObjects;
        std::deque<cv::Mat> prevFrames;
    private:
        int framesInMemory;
        int maxTrackedObjects;
        double minTrackingPercentage;
};


void frameAnnotator(cv::Mat &frame, const std::vector<object> &detectedObjects);
