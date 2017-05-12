#include <opencv2/opencv.hpp>
#include "objects.h"
#include "stdio.h"

objectTracker::objectTracker(ObjectType o)
{
	obj = o;
    framesInMemory = 10;
    maxTrackedObjects = 10;
    minTrackingPercentage = 20.0;
}


void objectTracker::storeFrame(const cv::Mat &frame)
{
    if (prevFrames.size() < framesInMemory)
    {
        prevFrames.push_front(frame);
    }
    else
    {
        prevFrames.pop_back();
        prevFrames.push_front(frame);
    }
    
}

void objectTracker::update()
{
    for (size_t trackedObject = 0; trackedObject < trackedObjects.size(); ++trackedObject)
    {
        trackedObjects[trackedObject].lastSeenIndex++;
    }
}

void objectTracker::track(const std::vector<object> &detectedObjects, const cv::Mat &frame)
{
    storeFrame(frame);

    if (trackedObjects.size() > 0)
    {
        std::vector<bool> detectedObjectsThatWereTracked(detectedObjects.size());

        for (size_t trackedObject = 0; trackedObject < trackedObjects.size(); ++trackedObject)
        {
            if (trackedObjects[trackedObject].lastSeenIndex >= prevFrames.size()) continue;

            std::vector<cv::Point2f> trackedPts;
            std::vector<float> err;
            std::vector<uchar> status;

            cv::calcOpticalFlowPyrLK(prevFrames[trackedObjects[trackedObject].lastSeenIndex], 
                                    prevFrames[0], 
                                    trackedObjects[trackedObject].corners, 
                                    trackedPts, 
                                    status, 
                                    err);
            
            double trackingPercentage = 0;
            int correspondingDetectedObjectIdx = -1;
            double bestTrackingPercentage = 0.0; 

            for (size_t detectedObject = 0; detectedObject < detectedObjects.size(); ++detectedObject)
            {
                int nValidTrackedFeatures = 0;

                float max_x = detectedObjects[detectedObject].boundingBox.x + detectedObjects[detectedObject].boundingBox.width;
                float max_y = detectedObjects[detectedObject].boundingBox.y + detectedObjects[detectedObject].boundingBox.height;
                
                for (size_t trackedPt = 0; trackedPt < trackedObjects[trackedObject].corners.size(); ++trackedPt)
                {
                    if (status[trackedPt] && detectedObjects[detectedObject].boundingBox.contains(trackedPts[trackedPt]))
                        {
                            ++nValidTrackedFeatures;
                        }
                }

                trackingPercentage = 100.0*nValidTrackedFeatures/trackedObjects[trackedObject].corners.size();

                if (trackingPercentage > minTrackingPercentage && trackingPercentage > bestTrackingPercentage)
                {
                    bestTrackingPercentage = trackingPercentage;
                    correspondingDetectedObjectIdx = detectedObject;
                }
            }

            if (correspondingDetectedObjectIdx >= 0)
            {
                trackedObjects[trackedObject] = detectedObjects[correspondingDetectedObjectIdx];
                detectedObjectsThatWereTracked[correspondingDetectedObjectIdx] = true;   
				trackedObjects[trackedObject].lastSeenIndex = 0;
            }
        }

        if (trackedObjects.size() < maxTrackedObjects)
        {
            for(size_t detectedObject = 0; detectedObject < detectedObjects.size(); ++detectedObject)
            {
                if(!detectedObjectsThatWereTracked[detectedObject])
                {
                    trackedObjects.push_back(detectedObjects[detectedObject]);
					trackedObjects[trackedObjects.size() - 1].lastSeenIndex = 0;
					trackedObjects[trackedObjects.size() - 1].objectID = numObjects;
					numObjects++;
                 }
            }
        }

    }
    else
    {
        trackedObjects = detectedObjects;
		for (int i = 0; i < trackedObjects.size(); i++){
			trackedObjects[i].lastSeenIndex = 0;
			trackedObjects[i].objectID = numObjects;
			numObjects++;
		}
    }
}