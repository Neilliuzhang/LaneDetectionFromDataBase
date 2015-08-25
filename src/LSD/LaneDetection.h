#ifndef LANEDETECTION_H
#define LANEDETECTION_H

#include <opencv2\opencv.hpp>
using namespace cv;




class LaneDetection
{
public:
	LaneDetection();
	LaneDetection(const Mat &ipmImage);



	void detectionLineLSD(Mat &processImage);
	void blobProcess(Mat &processImage);
	void run();

	Mat ipmImage;
	Mat processImage;


};








#endif // LANEDETECTION_H