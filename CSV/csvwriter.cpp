#include "csvwriter.h"
#include <iostream>
#include <fstream>
#include <ctime>

std::ofstream csvFile;

csvwriter::csvwriter()
{
	if (!initFileName() || initFileHeaders()){
		this->~csvwriter();
		return;
	}
}


csvwriter::~csvwriter()
{
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
	char yrString[4];
	_itoa(year, yrString, 10);
	char mon[2];
	_itoa(month, mon, 10);
	char dayString[2];
	_itoa(day, dayString, 10);
	char hourString[2];
	_itoa(hour, hourString, 10);
	char minString[2];
	_itoa(min, minString, 10);

	//Create CSV file name
	char fileName[50] = "PSU_StereoCamera_";
	strcat(fileName, yrString);
	strcat(fileName, mon);
	strcat(fileName, dayString);
	strcat(fileName, "_");
	strcat(fileName, hourString);
	strcat(fileName, minString);
	strcat(fileName, ".csv");

	csvFile.open(fileName, std::ofstream::out);
	if (csvFile.is_open()){
		printf("Created CSV File: %s\n", fileName);
		return true;
	}
	else{
		printf("Failed to create CSV File: %s\n", fileName);
		return false;
	}
}

bool csvwriter::initFileHeaders(){
	if (csvFile.is_open()){
		csvFile.write("Time(s),Veh1ID,Veh1Distance(m),Veh1LanePos(0|1|2|3)," \
			"Veh2ID,Veh2Distance(m),Veh2LanePos(0|1|2|3),Veh3ID	Veh3Distance(m),Veh3LanePos(0|1|2|3),Veh4ID," \
			"Veh4Distance(m),Veh4LanePos(0|1|2|3),Veh5ID,Veh5Distance(m),Veh5LanePos(0|1|2|3),Ped1ID,Ped1Distance(m)," \
			"Ped1Pos(0|1|2|3),Ped2ID,Ped2Distance(m),Ped2Pos(0|1|2|3),Ped3ID,Ped3Distance(m),Ped3Pos(0|1|2|3),Ped4ID,Ped4Distance(m)," \
			"Ped4Pos(0|1|2|3),Ped5ID,Ped5Distance(m),Ped5Pos(0|1|2|3),Sign1ID,Sign1Distance(m),Sign1Type(0|1|2|3),Sign2ID,Sign2Distance(m)," \
			"Sign2Type(0|1|2|3),Sign3ID,Sign3Distance(m),Sign3Type(0|1|2|3),Sign4ID,Sign4Distance(m)," \
			"Sign4Type(0|1|2|3),Sign5ID,Sign5Distance(m),Sign5Type(0|1|2|3)\n",800);

	}
	else{
		printf("CSV File is not open!");
		return false;
	}
}

//void csvwriter::writeVehicleData(objectTracker vehicleTracker){
//
//}
//
//void csvwriter::writeSignData(objectTracker signTracker){
//
//}
//
//void csvwriter::writePedData(objectTracker pedTracker){
//
//}

