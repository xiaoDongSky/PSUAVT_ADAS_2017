#include <opencv2\opencv.hpp>
#include "ImageCapture.h"
#include <comdef.h>
#include <Windows.h>
#include <dshow.h>
#pragma comment(lib, "strmiids")


HRESULT EnumerateDevices(REFGUID category, IEnumMoniker **ppEnum);
void FindCameras(IEnumMoniker *pEnum, int *leftCamera, int *rightCamera);
std::mutex imageLock;
std::thread cameraCaptureThread;
bool stopHere = false;

/**
Initializes cameras and begins camera capture threads. Uses DirectShow to find the cameras
*/
StereoCamera::StereoCamera()
{
	int left=0;
	int right=0;
	HRESULT hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);
	if (SUCCEEDED(hr))
	{
		IEnumMoniker *pEnum;

		hr = EnumerateDevices(CLSID_VideoInputDeviceCategory, &pEnum);
		if (SUCCEEDED(hr))
		{
			FindCameras(pEnum,&left,&right);
			pEnum->Release();
		}
	}
	if (right <= left){
		printf("Something went wrong!\n");
		printf("Left Camera Index : %d\n", left);
		printf("Right Camera Index : %d\n", right);
		printf("Make sure both cameras are plugged in. Plug Left Camera in before Right Camera as well.\n");
		this->~StereoCamera();
		system("pause");
		exit(1);
	}
	else
		StereoCamera(left, right);
}

/**
leftCamera - Left camera device number (As user must know what value this is)
rightCamera - Right camera device number

Opens Cameras using OpenCV interface.
*/
StereoCamera::StereoCamera(int leftCamera, int rightCamera){
	leftCam = cv::VideoCapture(leftCamera);
	rightCam = cv::VideoCapture(rightCamera);
	if (!leftCam.isOpened() || !rightCam.isOpened())
		CV_Assert("Failed to open cameras");
	leftCam.set(CV_CAP_PROP_FOCUS, 0);
	leftCam.set(CV_CAP_PROP_ZOOM, 100);
	leftCam.set(CV_CAP_PROP_FRAME_WIDTH, 864);
	leftCam.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	rightCam.set(CV_CAP_PROP_FOCUS, 0);
	rightCam.set(CV_CAP_PROP_FRAME_WIDTH, 864);
	rightCam.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	cameraCaptureThread = std::thread(&StereoCamera::updateFrames,this);
	cameraCaptureThread.detach();
}

StereoCamera::~StereoCamera()
{

}

void StereoCamera::cleanUp(){
	imageLock.lock();
	stopHere = true;
	leftCam.release();
	rightCam.release();
	imageLock.unlock();
	this->~StereoCamera();
}

void StereoCamera::updateFrames(){
	while (!stopHere){
		cv::Mat tmp;
		cv::Mat tmp2;
		leftCam >> tmp;
		rightCam >> tmp2;

		if (tmp.empty() && tmp2.empty()) continue;

		imageLock.lock();
		tmp.copyTo(leftFrame);
		tmp2.copyTo(rightFrame);
		cv::cvtColor(leftFrame, leftFrameGray, cv::COLOR_BGR2GRAY);
		cv::cvtColor(rightFrame, rightFrameGray, cv::COLOR_BGR2GRAY);
		imageLock.unlock();
	}
}

void StereoCamera::getColorImages(cv::Mat *left,cv::Mat *right){
	imageLock.lock();
	leftFrame.copyTo(*left);
	rightFrame.copyTo(*right);
	imageLock.unlock();
}

void StereoCamera::getGrayImages(cv::Mat *left, cv::Mat *right){
	imageLock.lock();
	*left = leftFrameGray.clone();
	*right = rightFrameGray.clone();
	imageLock.unlock();
}

HRESULT EnumerateDevices(REFGUID category, IEnumMoniker **ppEnum)
{
	// Create the System Device Enumerator.
	ICreateDevEnum *pDevEnum;
	HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL,
		CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pDevEnum));

	if (SUCCEEDED(hr))
	{
		// Create an enumerator for the category.
		hr = pDevEnum->CreateClassEnumerator(category, ppEnum, 0);
		if (hr == S_FALSE)
		{
			hr = VFW_E_NOT_FOUND;  // The category is empty. Treat as an error.
		}
		pDevEnum->Release();
	}
	return hr;
}

void FindCameras(IEnumMoniker *pEnum, int *leftCamera, int *rightCamera)
{
	IMoniker *pMoniker = NULL;
	bool leftCameraSet = false;
	int cameraNum = 0;

	while (pEnum->Next(1, &pMoniker, NULL) == S_OK)
	{
		IPropertyBag *pPropBag;
		HRESULT hr = pMoniker->BindToStorage(0, 0, IID_PPV_ARGS(&pPropBag));
		if (FAILED(hr))
		{
			pMoniker->Release();
			continue;
		}

		VARIANT var;
		VariantInit(&var);

		// Get description or friendly name.

		hr = pPropBag->Read(L"FriendlyName", &var, 0);
		if (FAILED(hr))
		{
			hr = pPropBag->Read(L"Description", &var, 0);
		}
		if (SUCCEEDED(hr))
		{
			char *p = _com_util::ConvertBSTRToString(var.bstrVal);
			if (strncmp(p, "Logitech HD Pro Webcam C920", 30)==0){
				if (!leftCameraSet){
					*leftCamera = cameraNum;
					leftCameraSet = true;
				}
				else
					*rightCamera = cameraNum;
				
				/*IBaseFilter *pFilter;
				IAMCameraControl *pCameraControl;
				HRESULT hRes;
				hRes = pMoniker->BindToObject(NULL, NULL, IID_IBaseFilter,(void**)&pFilter);
				hRes = pFilter->QueryInterface(IID_IAMCameraControl, (void **)&pCameraControl);
				if (hRes == S_OK){
					printf("Success! Setting Focus.\n");
					hRes = pCameraControl->Set(CameraControl_Focus,0, CameraControl_Flags_Manual);
					if (SUCCEEDED(hRes))
						printf("Success! Setting Zoom.\n");
					hRes = pCameraControl->Set(CameraControl_Zoom, 100, CameraControl_Flags_Manual);
					if (SUCCEEDED(hRes))
						printf("Success!\n");
				}
				pFilter->Release();*/
			}
			
		}
		cameraNum++;
		pPropBag->Release();
		pMoniker->Release();
		
	}
}