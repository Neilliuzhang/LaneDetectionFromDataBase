#define _CRT_SECURE_NO_WARNINGS

#include "KITTI_Data_Reader\KITTI_Data_Reader.h"
#include "RectifyImages\RectifyStereo.h"
#include "VisualOdometryStereo\InterfaceProcessVISO.h"
#include "LIBELAS\image.h"
#include "LIBELAS\ELAS_Disparity_Interface.h"
#include "IPMImage\IPMImage.h"
#include "LSD\LaneDetection.h"

#include <iostream>
using namespace std;



void parse(string &DataSetFolderName, string &calibFileName, int &rectified,
	int &frameInterval, float &h, int &methodeDisparity, int &noDisparity,
	int &elasSetting)
{
	ifstream in("config.txt");
	if (!in.is_open())
	{
		cout << "error to open file : config.txt. Use defalut values." << endl;
		return;
	}
	in >> DataSetFolderName;
	in >> calibFileName;
	in >> rectified;
	in >> frameInterval;
	in >> h;
	in >> methodeDisparity;//0:elas ; 1:sgbm
	in >> noDisparity;//0:computes disparity map every frame
	in >> elasSetting;
	in.close();
}


int main(){

	//default values
	string DataSetFolderName; 
	DataSetFolderName = "C:\\201506-201511MFE\\KITTI_data\\2011_09_26\\2011_09_26_drive_0029_sync";

	string calibFileName; 
	calibFileName = "C:\\201506-201511MFE\\KITTI_data\\2011_09_26\\calib_cam_to_cam.txt";

	//default settings:
	int frameInterval = 7;
	float h = 1.2;//m
	int rectified = 1; // 1 -- have already rectified; 0 -- need rectify
	int methodeDisparity = 0;
	int noDisparity = 0;
	int elasSetting = Elas::ROBOTICS;

	//read config.txt to settings:
	parse(DataSetFolderName, calibFileName, rectified, frameInterval, h, methodeDisparity, noDisparity, elasSetting);
	

	KITTI_Data_Reader reader(DataSetFolderName);
	RectifyStereo rectifyStereo(calibFileName);

	cout << "Image number in this dir : " << reader.getMaxIndex() << endl;

	if (!rectifyStereo.isLoadCameraParam)
	{
		cout << "Camera Param is not loaded! Promgram exits..." << endl;
		return 1;
	}


	Calib_Data_Type calibData = rectifyStereo.calibData;

	//instance for computing Visual Odometry (camera pose change)
	InterfaceProcessVISO *procVISO = new InterfaceProcessVISO(calibData);
	

	Elas::setting enumSetting = Elas::ROBOTICS;
	if (elasSetting == 1) // default 0
		enumSetting = Elas::MIDDLEBURY;

	Elas::parameters param(enumSetting);
	InterfaceProcessELAS procELAS(param);//instance for computing disparity map

	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 256, 11);//sgbm for computing dm

	
	reader.generateNextDataFileName(0);
	string left_img_file_name = reader.curImageFileName[0];
	string right_img_file_name = reader.curImageFileName[1];

	Mat L = imread(left_img_file_name, 0);
	Mat R = imread(right_img_file_name, 0);
	Mat rL, rR;

	if (rectified == 1)
	{
		L.copyTo(rL);
		R.copyTo(rR);
	}
	else
		rectifyStereo.rectifyImages(L, R, rL, rR);

	Mat disp;
	procELAS.computeDisparity(rL, rR, disp);

	InterfaceProcessIPMImage *interface_ipm = new InterfaceProcessIPMImage(calibData, h, disp);//initialize pitch angle with disparity map
	interface_ipm->showVehiclePosition(false, true);

	int frameNum = 0;
	Mat ipmImage;
	while (reader.generateNextDataFileName())
	{
		int64 t0 = getTickCount();
		string left_img_file_name = reader.curImageFileName[0];
		string right_img_file_name = reader.curImageFileName[1];

		Mat L = imread(left_img_file_name);
		Mat R = imread(right_img_file_name);
		Mat rL, rR;//rectified images : rL, rR
		
		if (rectified == 1)
		{
			L.copyTo(rL);
			R.copyTo(rR);
		}
		else
			rectifyStereo.rectifyImages(L, R, rL, rR);


		if (frameNum == frameInterval)
		{
			procELAS.computeDisparity(rL, rR, disp);
			procVISO->~InterfaceProcessVISO();
			procVISO = new InterfaceProcessVISO(calibData);
			interface_ipm->~InterfaceProcessIPMImage();
			interface_ipm = new InterfaceProcessIPMImage(calibData, h, disp);
			interface_ipm->showVehiclePosition(false, true);
			frameNum = 0;
		}

		procVISO->processVISO(rL, rR);
		//cout << "[";
		//cout << procVISO.pose << "]" << endl;
		//cout << endl;
		

		///----------------------Disparity Map---------------------------
		Mat LforDisp, RforDisp;//Images for computing disparity map : LforDisp, RforDisp

		rectifyStereo.getROI(rL, rR, LforDisp, RforDisp);

		Mat disp;
		if (!noDisparity)
		{
			if (methodeDisparity == 0)
				procELAS.computeDisparity(LforDisp, RforDisp, disp);
			else
			{
				sgbm->compute(LforDisp, RforDisp, disp);
				disp.convertTo(disp, CV_8U, 1.0 / 16);
			}
		}
			

		///---------------------------------------------------------------

		
		string oxts_file_name = reader.curOxtsFileName;
		ifstream oxts_file(oxts_file_name);
		Oxts_Data_Type *oxtsData;//gps data
		if (oxts_file.is_open())
		{
			oxtsData = new Oxts_Data_Type;
			oxts_file >> *oxtsData;
		}
		else
		{
			oxtsData = NULL;
		}

		interface_ipm->processIPM(rL, procVISO->pose, ipmImage, oxtsData);

		int64 t1 = getTickCount();
		cout << (t1 - t0) / getTickFrequency() * 1000 << " ms. " << endl;;

		//detection of lines or lanes in ipmImage
		LaneDetection lsd(ipmImage);
		lsd.run();

		if (rL.data && rR.data)
		{
			imshow("rL", rL);
			imwrite("rl.png", rL);
			imshow("rR", rR);
		}
		if (ipmImage.data)
			imshow("ipm", ipmImage);
		if (disp.data)
		{
			imshow("disp", disp);
			imwrite("disp.png", disp);
		}

		if (waitKey(10) > 0)
			waitKey();

		frameNum++;
	}
	return 1;
}