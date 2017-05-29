#include "csvwriter.h"


csvwriter::csvwriter()
{
	if (!initFileName() || !initFileHeaders()){
		this->~csvwriter();
		return;
	}
}


csvwriter::~csvwriter()
{
	csvFile.close();
	videoWriter.release();
	printf("Files Closed\n");
}

bool csvwriter::initFileName(){
	//Get system time and break into Time components 
	time_t curTime = time(0);
	tm *time = localtime(&curTime);
	int year = time->tm_year + 1900;
	int month = time->tm_mon + 1;
	int day = time->tm_mday;
	int hour = time->tm_hour;
	int min = time->tm_min;

	//Convert integers to characters
	char yrString [6]="\0";
	_itoa(year, yrString, 10);
	char mon [5]="\0";
	_itoa(month, mon, 10);
	char dayString [5]="\0";
	_itoa(day, dayString, 10);
	char hourString [5]="\0";
	_itoa(hour, hourString, 10);
	char minString [5]="\0";
	_itoa(min, minString, 10);

	//Create CSV file name
	char fileName[50] = "PSU_StereoCamera_";
	strcat(fileName, yrString);
	strcat(fileName, mon);
	strcat(fileName, dayString);
	strcat(fileName, "_");
	strcat(fileName, hourString);
	strcat(fileName, minString);

	char csvName [50]="\0";
	strcat(csvName, fileName);
	strcat(csvName, ".csv");

	char videoName [50]="\0";
	strcat(videoName, fileName);
	strcat(videoName, ".avi");
	//strcat(fileName, ".csv");

	csvFile.open(csvName, std::ofstream::out);
	if (csvFile.is_open()){
		printf("Created CSV File: %s\n", csvFile);
		videoWriter = cv::VideoWriter(videoName, CV_FOURCC('M', 'P', '4', 'V'), 5, cv::Size(864, 480));
		if (videoWriter.isOpened()){
			printf("Opened Video File: %s\n", videoName);
			return true;
		}
		printf("Could not write video\n");
		return false;
	}
	else{
		printf("Failed to create CSV File: %s\n", fileName);
		return false;
	}
}

bool csvwriter::initFileHeaders(){
	if (csvFile.is_open()){
		csvFile.write("Time(s),Veh1ID,Veh1Distance(m),Veh1LanePos(0|1|2|3)," \
			"Veh2ID,Veh2Distance(m),Veh2LanePos(0|1|2|3),Veh3ID,Veh3Distance(m),Veh3LanePos(0|1|2|3),Ped1ID,Ped1Distance(m)," \
			"Ped1Pos(0|1|2|3),Ped2ID,Ped2Distance(m),Ped2Pos(0|1|2|3),Sign1ID,Sign1Distance(m),Sign1Type(0|1|2|3),Sign2ID,Sign2Distance(m)," \
			"Sign2Type(0|1|2|3),Sign3ID,Sign3Distance(m),Sign3Type(0|1|2|3)\n",800);
		return true;
	}
	else{
		//printf("CSV File is not open!");
		return false;
	}
}

void csvwriter::writeData(double imgTime, double procTime, bool obstructed, cv::Mat frame, objectTracker stops, objectTracker vehicles, objectTracker speeds, objectTracker yields, objectTracker peds){
	if (obstructed){
		csvFile << imgTime << "," << "VIEW OBSTRUCTED" << std::endl;
	}
	else{
		int wholeNumFrames = int(procTime / 0.200);
		for (int i = 0; i < wholeNumFrames+1; i++){
			csvFile << imgTime + double(i * 0.200) << ",";
			writeVehicleData(vehicles);
			writePedData(peds);
			writeSignData(stops);
			writeSignData(speeds);
			writeSignData(yields);
			csvFile << std::endl;
		}
	}
	
	videoWriter.write(frame);
	
	
}

//void csvwriter::sendFrame(cv::Mat frameToWrite



void csvwriter::writeVehicleData(objectTracker vehicleTracker){
	for (int i = 0; i < vehicleTracker.maxTrackedObjects; i++){
		if (i < vehicleTracker.trackedObjects.size()){
			if (vehicleTracker.trackedObjects[i].lastSeenIndex == 0){
				csvFile << vehicleTracker.trackedObjects[i].objectID << "," << vehicleTracker.trackedObjects[i].distance << "," << vehicleTracker.trackedObjects[i].lane << ",";
			}
		}
		else{
			csvFile << "-1,-1,-1,";
		}
	}
}

void csvwriter::writeSignData(objectTracker signTracker){
	for (int i = 0; i < signTracker.maxTrackedObjects; i++){
		if (i < signTracker.trackedObjects.size()){
			if (signTracker.trackedObjects[i].lastSeenIndex == 0){
				csvFile << signTracker.trackedObjects[i].objectID << "," << signTracker.trackedObjects[i].distance <<  "," << signTracker.obj << ",";
			}
		}
		else{
			csvFile << "-1,-1,-1,";
		}
	}
}

void csvwriter::writePedData(objectTracker pedTracker){
	for (int i = 0; i < pedTracker.maxTrackedObjects; i++){
		if (i < pedTracker.trackedObjects.size()){
			if (pedTracker.trackedObjects[i].lastSeenIndex == 0){
				csvFile << pedTracker.trackedObjects[i].objectID << "," << pedTracker.trackedObjects[i].distance << "," << pedTracker.trackedObjects[i].lane << ",";
			}
		}
		else{
			csvFile << "-1,-1,-1,";
		}
	}
}

