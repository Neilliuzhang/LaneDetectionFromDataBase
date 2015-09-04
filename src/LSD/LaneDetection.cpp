#include "LaneDetection.h"
#include "lsd.h"
#include <iostream>
using namespace std;
LaneDetection::LaneDetection(){}

LaneDetection::LaneDetection(const Mat &_ipmImage){
	_ipmImage.copyTo(ipmImage);
	_ipmImage.copyTo(processImage);
}

void LaneDetection::detectionLineLSD(Mat &processImage){
	using namespace cv;

	unsigned int X = processImage.cols;  /* x image size */
	unsigned int Y = processImage.rows;  /* y image size */
	unsigned int XY = X * Y; /* numer of pixels */

	/* create a simple image: left half black, right half gray */
	//Vec3b intensity;
	//image = new_image_double(X,Y);
	//for(x=0;x<X;x++)
	//	for(y=0;y<Y;y++){
	//		//image->data[ x + y * image->xsize ] = x<X/2 ? 0.0 : 64.0; /* image(x,y) */
	//		intensity = inputImage.at<Vec3b>(y,x);
	//		image->data[ x + y * image->xsize ] = (double) (intensity.val[2]+intensity.val[1]+intensity.val[0])/3;
	//	}

	Mat grayImage;
	if (processImage.channels() == 1)
		processImage.copyTo(grayImage);
	else if (processImage.channels() == 3)
		cvtColor(processImage, grayImage, CV_BGR2GRAY);
	

	/*blur the image*/
	blur(grayImage, grayImage, Size(5, 5));


	image_double image = new_image_double(X, Y);
	/*copy the data*/
	uchar* ptr_gray_image = grayImage.ptr<uchar>(0);
	for (int _pixel = 0; _pixel < XY; _pixel++)
	{
		image->data[_pixel] = (double)ptr_gray_image[_pixel];
	}

	/* call LSD */
	ntuple_list out = lsd(image);

	/* print output */
	//LOG_INFO("line segments found: " << out->size);

	Mat colorImage;
	cvtColor(processImage, processImage, CV_GRAY2BGR);

	/*draw the lines*/
	for (int i = 0; i < out->size; i++)
	{
		int b = (unsigned)theRNG() & 255;
		int g = (unsigned)theRNG() & 255;
		int r = (unsigned)theRNG() & 255;
		int thickness = (int)out->values[i * out->dim + 4] / 2;
		int lineType = 8;
		Point start = cv::Point(out->values[i * out->dim + 0], out->values[i * out->dim + 1]),
			end = cv::Point(out->values[i * out->dim + 2], out->values[i * out->dim + 3]);

		line(processImage, start, end, Scalar(b, g, r), 1, lineType);
	}

	/* free memory */
	free_image_double(image);
	free_ntuple_list(out);


}

void LaneDetection::blobProcess(Mat &processImage)
{
	
	ConnectedComponentsTypes;
	Mat labelImage, stats, centroids;
	int num = connectedComponentsWithStats(processImage, labelImage, stats, centroids);

	//cout << "stats: " << stats << endl;
	//cout << "centroids: " << centroids << endl;

	int *ptr_labelImage = labelImage.ptr<int>(0);
	int rows = labelImage.rows, cols = labelImage.cols;
	int pixels = rows *cols;
	//Mat colorImage;
	//cvtColor(processImage, colorImage, CV_GRAY2BGR);

	for (int i = 0; i < num; i++)
	{
		int *ptr_row_stats = stats.ptr<int>(i);
		bool possible_noise = ptr_row_stats[CC_STAT_HEIGHT] < 10 || ptr_row_stats[CC_STAT_AREA] < 5;
		if (!possible_noise) continue;

		for (int y = ptr_row_stats[CC_STAT_TOP]; y < ptr_row_stats[CC_STAT_TOP] + ptr_row_stats[CC_STAT_HEIGHT]; y++)
		{
			int *ptr_row_label = labelImage.ptr<int>(y);
			uchar *ptr_row_processImage = processImage.ptr<uchar>(y);
			for (int x = ptr_row_stats[CC_STAT_LEFT]; x < ptr_row_stats[CC_STAT_LEFT] + ptr_row_stats[CC_STAT_WIDTH]; x++)
			{
				if (ptr_row_label[x] == i)
				{
					ptr_row_processImage[x] = 0;
				}
			}
		}
		
	}

	
}



void LaneDetection::run(){
	if (!processImage.data)
	{
		cout << "processImage is empty. " << endl;
		return;
	}
	imshow("LaneDetection input image", processImage);
	imwrite("ipm.png", processImage);

	Mat image;
	processImage.copyTo(image);
	detectionLineLSD(image);
	imshow("image", image);
	imwrite("ipm_lines.png", image);

	Sobel(processImage, processImage, CV_8U, 1, 0, 1, 2);
	Sobel(processImage, processImage, CV_8U, 1, 0, 1, 2);
	//threshold(processImage, processImage, 0, 255, THRESH_OTSU);
	imshow("sobel-2", processImage);
	imwrite("sobel-2.png", processImage);

	blobProcess(processImage);

	//imshow("blobProcess", processImage);
	imwrite("lsd1.png", processImage);

	detectionLineLSD(processImage);
	imshow("lsd", processImage);
	imwrite("lsd2.png", processImage);
}