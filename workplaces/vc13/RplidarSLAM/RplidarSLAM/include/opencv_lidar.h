#pragma once
#include <iostream>
#include <cmath>

#include <opencv2\opencv.hpp>
#include <highgui.h> 

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

using namespace std;
using namespace cv;

static int RadarImageWdith = 600;
static int RadarImageHeight = 600;

struct scanDot {
	_u8   quality;
	float angle;
	float dist;
};

class LidarImage
{
public:
	LidarImage(void);
	~LidarImage(void);

	vector<scanDot> scan_data;
	float scan_speed;
	void scanData(rplidar_response_measurement_node_t *buffer, size_t count, float frequency);
	void draw(IplImage* RadarImage);
};