#include <opencv2/opencv.hpp>
#include "objects.h"

object::object(const cv::Rect &_boundingBox, const std::vector<cv::Point2f> &_corners, const int &_lastSeenIndex)
{
    boundingBox = _boundingBox;
	center = cv::Point2i(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
    corners = _corners;
    lastSeenIndex = _lastSeenIndex;
}