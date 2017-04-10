#include "ImageCapture.h"
#include <mutex>

class StereoCamera
{
public:
	StereoCamera(int leftCamera, int rightCamera);
	StereoCamera();
	~StereoCamera();

private:
	std::mutex imageLock;
};

StereoCamera::StereoCamera()
{
}

StereoCamera::StereoCamera(int leftCamera, int rightCamera)
{
}

StereoCamera::~StereoCamera()
{
}

