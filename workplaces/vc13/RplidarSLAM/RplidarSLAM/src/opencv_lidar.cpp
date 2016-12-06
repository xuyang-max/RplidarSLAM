#include "opencv_lidar.h"
#define PI 3.141592653

LidarImage::LidarImage(void)
{
}

LidarImage::~LidarImage(void)
{

}

void LidarImage::scanData(rplidar_response_measurement_node_t *buffer, size_t count, float frequency)
{
	scan_data.clear();
	for (int pos = 0; pos < (int)count; ++pos) {
		scanDot dot;
		if (!buffer[pos].distance_q2) continue;

		dot.quality = (buffer[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
		dot.angle = (buffer[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
		dot.dist = buffer[pos].distance_q2 / 4.0f;
		scan_data.push_back(dot);
	}

	scan_speed = frequency;
}

void LidarImage::draw(IplImage* RadarImage)
{
	//RadarImage = cvCreateImage(cvSize(RadarImageWdith,RadarImageHeight),IPL_DEPTH_8U,1);  
	cvZero(RadarImage);
	//在中心加上一个圆心  
	cvCircle(RadarImage, cvPoint(RadarImageWdith / 2, RadarImageHeight / 2), 3, CV_RGB(0, 255, 255), -1, 8, 0);

	int x, y;
	double theta, rho;
	unsigned char * pPixel = 0;
	int halfWidth = RadarImageWdith / 2;
	int halfHeight = RadarImageHeight / 2;

	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.5, 0.5, 0, 1, 8);

	for (int i = 0; i < scan_data.size(); i++)
	{
		scanDot dot;
		dot = scan_data[i];
		theta = dot.angle*PI / 180;
		rho = dot.dist;

		x = (int)(rho*sin(theta) / 20) + halfWidth;
		y = (int)(-rho*cos(theta) / 20) + halfHeight;

		cvCircle(RadarImage, cvPoint(x, y), 1, CV_RGB(0, 255, 0), 1, 8, 3);
	}

	char s[35];
	sprintf_s(s, "Value Count: %d, Scan Speed: %0.2f", scan_data.size(), scan_speed);
	cvPutText(RadarImage, s, cvPoint(50, 50), &font, cvScalar(255, 0, 0));
}