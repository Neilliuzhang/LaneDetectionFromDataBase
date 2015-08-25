#include "IPMImage.h"


InterfaceProcessIPMImage::InterfaceProcessIPMImage(){
	ipm = new InversePerspectiveMapping();
	showInfoVISO = false;
	showInfoGPS = false;
}


InterfaceProcessIPMImage::InterfaceProcessIPMImage(Calib_Data_Type calibData, double h, double rx){
	ipm = new InversePerspectiveMapping(calibData.S_rect_00[1], calibData.S_rect_00[0]);
	ipm->createModelForStandardAssumption(calibData.P_rect_00[0], calibData.P_rect_00[0],
		calibData.P_rect_00[2], calibData.P_rect_00[6], h, rx);
	width = calibData.S_rect_00[0];
	height = calibData.S_rect_00[1];
	ipmImage = Mat::zeros((IPM_Z_MAX - IPM_Z_MIN) * IPM_IMAGE_SIZE_SCALE,
		(IPM_X_MAX - IPM_X_MIN) * IPM_IMAGE_SIZE_SCALE, CV_8UC1);
	showInfoVISO = false;
	showInfoGPS = false;
}

InterfaceProcessIPMImage::InterfaceProcessIPMImage(Calib_Data_Type calibData, double h, const Mat &disp){
	ipm = new InversePerspectiveMapping(calibData.S_rect_00[1], calibData.S_rect_00[0]);
	ipm->createModelForStandardAssumption(calibData.P_rect_00[0], calibData.P_rect_00[0],
		calibData.P_rect_00[2], calibData.P_rect_00[6], h, disp);
	width = calibData.S_rect_00[0];
	height = calibData.S_rect_00[1];
	ipmImage = Mat::zeros((IPM_Z_MAX - IPM_Z_MIN) * IPM_IMAGE_SIZE_SCALE,
		(IPM_X_MAX - IPM_X_MIN) * IPM_IMAGE_SIZE_SCALE, CV_8UC1);
	showInfoVISO = false;
	showInfoGPS = false;
}

void InterfaceProcessIPMImage::showVehiclePosition(bool infoVISO, bool infoGPS)
{
	showInfoVISO = infoVISO;
	showInfoGPS = infoGPS;
}

void InterfaceProcessIPMImage::processIPM(const Mat &image, const Mat &pose, Mat &outIPMImage, 
	Oxts_Data_Type *gpsData)
{
	ipm->updateFrameUsingStandardAssumption(pose);

	Mat grayImage;
	if (image.channels() > 1)
		cvtColor(image, grayImage, CV_BGR2GRAY);
	else
		image.copyTo(grayImage);

	for (int i = 0; i < height; i++)
	{
		if (i < height / 2) continue;
		uchar* ptr_row_grayImage = grayImage.ptr<uchar>(i);
		for (int j = 0; j < width; j++)
		{
			double X, Z;
			X = ipm->remapX[i*width + j];
			Z = ipm->remapZ[i*width + j];
			if (Z < IPM_Z_MIN || Z >= IPM_Z_MAX) continue;
			int r = (Z - IPM_Z_MIN) * ((float)ipmImage.rows) / (IPM_Z_MAX - IPM_Z_MIN);
			r = ipmImage.rows - r - 1;

			uchar *ptr_row_ipm_image = ipmImage.ptr<uchar>(r);
			if (X < IPM_X_MIN || X >= IPM_X_MAX) continue;
			int c = (X - IPM_X_MIN) * ((float)ipmImage.cols) / (IPM_X_MAX - IPM_X_MIN);

			//if (r >= ipm_image.rows || c >= ipm_image.cols || r < 0 || c < 0)
			//	cout << "asdfasdfasfdasdfasdfas" << r << "," << c << endl;

			ptr_row_ipm_image[c] = ptr_row_grayImage[j];
		}
	}

	if (showInfoVISO)
		;

	if (showInfoGPS && (gpsData != NULL))
		drawVehiclePosition(gpsData);

	ipmImage.copyTo(outIPMImage);
}

void InterfaceProcessIPMImage::processIPM(const Mat &image, const Matrix &pose, Mat &IPMImage, 
	Oxts_Data_Type *gpsData)
{
	Mat mat_pose = Mat::eye(pose.m, pose.n, CV_64FC1);
	double* ptr_mat_pose = mat_pose.ptr<double>(0);
	for (int i = 0; i < mat_pose.rows; i++)
		for (int j = 0; j < mat_pose.cols; j++)
			ptr_mat_pose[i*mat_pose.cols + j] = pose.val[i][j];
	
	processIPM(image, mat_pose, IPMImage, gpsData);
}

void InterfaceProcessIPMImage::drawVehiclePosition()
{
}
