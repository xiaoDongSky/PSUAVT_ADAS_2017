#include <opencv2/opencv.hpp>
#include "objects.h"

object::object(const cv::Rect &_boundingBox, const std::vector<cv::Point2f> &_corners, const int &_lastSeenIndex)
{
    boundingBox = _boundingBox;
    corners = _corners;
    lastSeenIndex = _lastSeenIndex;
}