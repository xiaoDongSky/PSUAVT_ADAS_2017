#include <stdio.h>
#include "StereoCalculation.h"


cv::Mat R1, R2, P1, P2, Q, newCameraMatrix1, map11, map21, newCameraMatrix2, map12, map22;
cv::StereoBM disparityCalculator;

cv::Mat createCameraMatrix(double f_x, double c_x, double f_y, double c_y){
	cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
	cameraMatrix.at<double>(0, 0) = f_x;   // f_x
	cameraMatrix.at<double>(0, 2) = c_x;   // c_x
	cameraMatrix.at<double>(1, 1) = f_y;   // f_y
	cameraMatrix.at<double>(1, 2) = c_y;   // c_y
	cameraMatrix.at<double>(2, 2) = 1.000;   // 1

	return cameraMatrix;
}

cv::Mat createDistCoefficients(double k1, double k2, double p1, double p2, double k3){
	cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64FC1);
	distCoeffs.at<double>(0, 0) = k1; // k1
	distCoeffs.at<double>(0, 1) = k2; // k2
	distCoeffs.at<double>(0, 2) = p1; // p1
	distCoeffs.at<double>(0, 3) = p2; // p2
	distCoeffs.at<double>(0, 4) = k3; // k3

	return distCoeffs;
}

cv::Mat createR(double zero_zero, double zero_one, double zero_two, double one_zero, double one_one, double one_two, double two_zero, double two_one, double two_two){
	cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);
	R.at<double>(0, 0) = zero_zero;
	R.at<double>(0, 1) = zero_one;
	R.at<double>(0, 2) = zero_two;
	R.at<double>(1, 0) = one_zero;
	R.at<double>(1, 1) = one_one;
	R.at<double>(1, 2) = one_two;
	R.at<double>(2, 0) = two_zero;
	R.at<double>(2, 1) = two_one;
	R.at<double>(2, 2) = two_two;
	return R;
}

cv::Mat createT(double zero_zero, double one_zero, double two_zero){
	cv::Mat T = cv::Mat::zeros(3, 1, CV_64FC1);
	T.at<double>(0, 0) = zero_zero;
	T.at<double>(1, 0) = one_zero;
	T.at<double>(2, 0) = two_zero;
	return T;
}

void initDisparity(cv::Mat camMat1, cv::Mat distCoeffs1, cv::Mat camMat2, cv::Mat distCoeffs2, cv::Mat R, cv::Mat T){
	cv::Size frameSize(864, 480);
	cv::stereoRectify(camMat1, distCoeffs1, camMat2, distCoeffs2, frameSize, R, T, R1, R2, P1, P2, Q, CV_CALIB_ZERO_DISPARITY, 1, frameSize);
	cv::initUndistortRectifyMap(camMat1, distCoeffs1, R1, newCameraMatrix1, frameSize, CV_32FC1, map11, map21);
	cv::initUndistortRectifyMap(camMat2, distCoeffs2, R2, newCameraMatrix2, frameSize, CV_32FC1, map12, map22);
	disparityCalculator = cv::StereoBM();
}

void rectifyImages(cv::Mat leftFrame, cv::Mat rightFrame, cv::Mat* leftRectified,cv::Mat* rightRectified){
	cv::Mat leftFrameRectified,
		rightFrameRectified;

	cv::remap(leftFrame, leftFrameRectified, map11, map21, CV_INTER_LINEAR);
	cv::remap(rightFrame, rightFrameRectified, map12, map22, CV_INTER_LINEAR);
	leftFrameRectified.copyTo(*leftRectified);
	rightFrameRectified.copyTo(*rightRectified);
}

void calculateDisparity(cv::Mat leftRectified, cv::Mat rightRectified, cv::Mat* disp, cv::Mat* p3d){
	cv::Mat leftRectGray, rightRectGray;

	cv::cvtColor(leftRectified, leftRectGray, CV_BGR2GRAY);
	cv::cvtColor(rightRectified, rightRectGray, CV_BGR2GRAY);
	cv::Mat disparity, points3D, disparityColorMap;
	disparityCalculator(leftRectGray, rightRectGray, disparity);
	cv::reprojectImageTo3D(disparity, points3D, Q, true);  // handles missing values
	disparity.copyTo(*disp);
}

void calculateDisparity(cv::Mat leftRectified, cv::Mat rightRectified, cv::Mat* disp,cv::Mat* p3d,cv::Mat * dispColorMap){
	cv::Mat copyLeft = leftRectified.clone();
	cv::Mat copyRight = rightRectified.clone();
	cv::Mat leftRectGray, rightRectGray;

	cv::cvtColor(copyLeft, leftRectGray, CV_BGR2GRAY);
	cv::cvtColor(copyRight, rightRectGray, CV_BGR2GRAY);
	cv::Mat disparity, points3D, disparityColorMap;
	disparityCalculator(leftRectGray, rightRectGray, disparity);
	cv::reprojectImageTo3D(disparity, points3D, Q, true);  // handles missing values
	disparity.copyTo(*disp);
	cv::applyColorMap(disparity, disparityColorMap, cv::COLORMAP_JET);
	disparityColorMap.copyTo(*dispColorMap);
}
