#include <opencv2/opencv.hpp>
#include <stdio.h>
#include "objects.h"

void frameAnnotator(cv::Mat &frame, const std::vector<object> &trackedObjects)
{
        static int frameNum = 0; 
        char buffer[12];
        for (int trackedObject = 0; trackedObject < trackedObjects.size(); ++trackedObject)
        {
            if (trackedObjects[trackedObject].lastSeenIndex == 0)
            {
                int thickness = 2;
                cv::rectangle(frame, trackedObjects[trackedObject].boundingBox, cv::Scalar(0, 0, 255), thickness);
                sprintf(buffer, "VEHICLE %i", trackedObject+1);
                cv::putText(frame, buffer, 
                            cv::Point(trackedObjects[trackedObject].boundingBox.x, 
                                        trackedObjects[trackedObject].boundingBox.y), 
                                        cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                                        cv::Scalar(0, 0, 255), 2);
                for (size_t corner = 0; corner < trackedObjects[corner].corners.size(); ++corner)
                {
                    cv::circle(frame, trackedObjects[trackedObject].corners[corner], 2, cv::Scalar(0, 255, 0));
                }
            }
        }

}