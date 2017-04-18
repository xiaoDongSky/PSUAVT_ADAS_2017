#include <opencv2\opencv.hpp>

cv::Mat createCameraMatrix(double f_x, double c_x, double f_y, double c_y);

cv::Mat createDistCoefficients(double k1, double k2, double p1, double p2, double k3);

cv::Mat createR(double zero_zero, double zero_one, double zero_two, double one_zero, double one_one, double one_two, double two_zero, double two_one, double two_two);

cv::Mat createT(double zero_zero, double one_zero, double two_zero);

void initDisparity(cv::Mat camMat1, cv::Mat distCoeffs1, cv::Mat camMat2, cv::Mat distCoeffs2, cv::Mat R, cv::Mat T);

void rectifyImages(cv::Mat leftFrame, cv::Mat rightFrame, cv::Mat* leftRectified, cv::Mat* rightRectified);

void calculateDisparity(cv::Mat leftRectified, cv::Mat rightRectified, cv::Mat* disp, cv::Mat* p3d);

void calculateDisparity(cv::Mat leftRectified, cv::Mat rightRectified, cv::Mat* disp, cv::Mat* p3d, cv::Mat *dispColorMap);