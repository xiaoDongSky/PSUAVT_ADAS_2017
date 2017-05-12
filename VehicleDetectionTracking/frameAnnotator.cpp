#include <opencv2/opencv.hpp>
#include <stdio.h>
#include "objects.h"

void frameAnnotator(cv::Mat &frame, objectTracker tracker)
{
        char buffer[20];
        for (int trackedObject = 0; trackedObject < tracker.trackedObjects.size(); ++trackedObject)
        {
            if (tracker.trackedObjects[trackedObject].lastSeenIndex == 0)
            {
                int thickness = 2;
				if (tracker.obj == VEHICLE){
					sprintf(buffer, "Vehicle %i", tracker.trackedObjects[trackedObject].objectID);
					if (tracker.trackedObjects[trackedObject].lane != HOST){
						cv::rectangle(frame, tracker.trackedObjects[trackedObject].boundingBox, cv::Scalar(0, 255, 0), thickness);
					}
					else{
						if (tracker.trackedObjects[trackedObject].distance < 25.0)
							cv::rectangle(frame, tracker.trackedObjects[trackedObject].boundingBox, cv::Scalar(0, 0, 255), thickness);
						else
							cv::rectangle(frame, tracker.trackedObjects[trackedObject].boundingBox, cv::Scalar(0, 255, 255 ), thickness);
					}

				}
				else if (tracker.obj == STOP){
					sprintf(buffer, "StopSign %i", tracker.trackedObjects[trackedObject].objectID);
					cv::rectangle(frame, tracker.trackedObjects[trackedObject].boundingBox, cv::Scalar(255, 255, 255), thickness);
				}
				else if (tracker.obj == SPEED){
					sprintf(buffer, "SpeedSign %i", tracker.trackedObjects[trackedObject].objectID);
					cv::rectangle(frame, tracker.trackedObjects[trackedObject].boundingBox, cv::Scalar(255, 255, 255), thickness);
				}
				else if (tracker.obj == YIELD){
					sprintf(buffer, "YieldSign %i", tracker.trackedObjects[trackedObject].objectID);
					cv::rectangle(frame, tracker.trackedObjects[trackedObject].boundingBox, cv::Scalar(255, 255, 255), thickness);
				}
				else{
					sprintf(buffer, "Ped %i", tracker.trackedObjects[trackedObject].objectID);
					cv::rectangle(frame, tracker.trackedObjects[trackedObject].boundingBox, cv::Scalar(255,0.0), thickness);
				}
				cv::putText(frame, buffer,
					cv::Point(tracker.trackedObjects[trackedObject].boundingBox.x,
					tracker.trackedObjects[trackedObject].boundingBox.y),
					cv::FONT_HERSHEY_SIMPLEX, 0.5,
					cv::Scalar(0, 0, 255), 2);
				for (size_t corner = 0; corner < tracker.trackedObjects[trackedObject].corners.size(); ++corner)
				{
					cv::circle(frame, tracker.trackedObjects[trackedObject].corners[corner], 2, cv::Scalar(0, 255, 0));
				}
            }
        }

}