#ifndef LANEDETECTION_H
#define LANEDETECTION_H

#include <opencv2\opencv.hpp>
using namespace cv;
using namespace std;
#include <vector>
#include "lsd.h"

class Segment2d
{
public:
	Point2d p1;
	Point2d p2;
	int indexZone;
	Segment2d(){};
	Segment2d(Point2d _p1, Point2d _p2);

	double getSlope(){
		if (!_ini_slope) computeSlope();
		return slope;
	}
	double getLength(){
		if (!_ini_length) computeLength();
		return length;
	}

	bool isNeighbor(Segment2d s);

	std::vector<Segment2d> getValidPoly(Segment2d s, int rows);

private:
	bool _ini_slope;
	bool _ini_length;
	double slope;
	double length;
	void computeSlope();
	void computeLength();
};

class LaneDetection
{
public:
	LaneDetection();
	LaneDetection(const Mat &ipmImage, const int center_x = 690);
	LaneDetection(const Mat &ipmImage, const Mat &ipmMask, const int center_x = 690);


	void detectionLineLSD(Mat &processImage);
	void detectionLineLSDInPerspImage(Mat &processImage);
	void blobProcess(Mat &processImage);
	void blobProcess2(Mat &processImage, const Mat &ipmImage);

	void method1();
	void method2();
	void method3(int);
	void run(int method = 1, int nameIndex_showImage = 0);

	Mat ipmImage;
	Mat processImage;
	Mat processGrayImage;
	Mat ipmMask;
	int center_x;

	void AlgoFilterLanes(ntuple_list lines_out);
	void AlgoFilterLanes_back(ntuple_list lines_out);

	int nameIndex;
};






#endif // LANEDETECTION_H