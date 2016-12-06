/*
* Copyright (C) 2014  RoboPeak
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*/
/*
*  RoboPeak Lidar System
*  Simple Data Grabber Demo App
*
*  Copyright 2009 - 2014 RoboPeak Team
*  http://www.robopeak.com
*
*  An ultra simple app to fetech RPLIDAR data continuously....
*
*/




#include <stdio.h>
#include <stdlib.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include "opencv_lidar.h"

#include <iostream>  
#include <cmath>  
#include "io.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;


bool checkRPLIDARHealth(RPlidarDriver * drv)
{
	u_result     op_result;
	rplidar_response_device_health_t healthinfo;


	op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
		printf("RPLidar health status : %d\n", healthinfo.status);
		if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
			fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
			// enable the following code if you want rplidar to be reboot by software
			// drv->reset();
			return false;
		}
		else {
			return true;
		}

	}
	else {
		fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
		return false;
	}
}

int main(int argc, const char * argv[]) {
	const char * opt_com_path = NULL;
	_u32         opt_com_baudrate = 115200;
	u_result     op_result;

	// read serial port from the command line...
	if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

	// read baud rate from the command line if specified...
	if (argc>2) opt_com_baudrate = strtoul(argv[2], NULL, 10);


	if (!opt_com_path) {
#ifdef _WIN32
		// use default com port
		opt_com_path = "\\\\.\\com3";
#else
		opt_com_path = "/dev/ttyUSB0";
#endif
	}

	// create the driver instance
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

	//creat opencv image
	IplImage* RadarImage = cvCreateImage(cvSize(RadarImageWdith, RadarImageHeight), IPL_DEPTH_8U, 3);

	LidarImage lidarImage;

	if (!drv) {
		fprintf(stderr, "insufficent memory, exit\n");
		exit(-2);
	}


	// make connection...
	if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
		fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
			, opt_com_path);
		goto on_finished;
	}



	// check health...
	if (!checkRPLIDARHealth(drv)) {
		goto on_finished;
	}


	// start scan...
	drv->startScan();

	// fetech result and print it out...
	char key;
	cvNamedWindow("Radar", 1);

	while (1) {
		rplidar_response_measurement_node_t nodes[360 * 2];
		size_t   count = _countof(nodes);

		op_result = drv->grabScanData(nodes, count);

		if (IS_OK(op_result)) {
			float frequency = 0;
			drv->getFrequency(nodes, count, frequency);
			lidarImage.scanData(nodes, count, frequency);
			lidarImage.draw(RadarImage);
			
			cvShowImage("Radar", RadarImage);
			key = cvWaitKey(30);
			if (key == 27)//escÍË³ö  
			{
				break;
			}		
		}
	}

	cvReleaseImage(&RadarImage);
	cvDestroyWindow("Radar");
	// done!
on_finished:
	RPlidarDriver::DisposeDriver(drv);
	return 0;
}

