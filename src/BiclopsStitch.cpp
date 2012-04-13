//============================================================================
// Name        : BiclopsStitch.cpp
// Author      : Dennis Lui
// Version     :
// Copyright   : 
// Description : Crude program reading in Kinect images captured using Biclops and stitching the colored point clouds together
//============================================================================

#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>
#include <stdio.h>
#include "kinect.h"
#include "utilities/rgbd_utils.h"
#include "utilities/pointcloud_viewer.h"

using namespace std;
using namespace cv;

int main() {

	char calib_filepath[] = "calib/dennis.yml";
	kinect myKinect(calib_filepath, false);

	//main folder storing the images
	string mainFolder = "/media/Data/Bionic Eye/Experimental Datasets/Biclops Scan/Scan1";

	//tilt angles
	double tiltAngles[] = {0.0,-30.0, 30.0};
	//pan angles
	double panAngles[] = {-150.0, -120.0, -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0};

	//storing all the folder names
	string folders[sizeof(tiltAngles)/sizeof(double)][sizeof(panAngles)/sizeof(double)];

	//folder paths to read all the images from
	ostringstream path;
	for(int i=0;i<sizeof(tiltAngles)/sizeof(double);i++)
	{
		for(int j=0;j<sizeof(panAngles)/sizeof(double);j++)
		{
			path.str("");
			path << mainFolder << "/P" << panAngles[j] << "T" << tiltAngles[i];
			folders[i][j] = path.str();
		}
	}

	//convert depth and color image at each scan position to color point clouds
	//transform them into a common coordinate frame

	ostringstream depthPath, rgbPath;
	int frameNum = 5;
	vector<pointcloud> allPoints;

	namedWindow("color",CV_GUI_NORMAL | CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
	for(int i=0;i<sizeof(tiltAngles)/sizeof(double);i++)
	{
		for(int j=0;j<sizeof(panAngles)/sizeof(double);j++)
		{
			Mat depthMat(Size(640,480),CV_16UC1);
			Mat rgbMat(Size(640,480),CV_8UC1);
			depthPath.str("");
			rgbPath.str("");
			depthPath << folders[i][j] << "/depth/depth" << frameNum << ".png";
			rgbPath << folders[i][j] << "/rgb/rgb" << frameNum << ".png";

			//loading the image into the system
			depthMat = cv::imread(depthPath.str().c_str(), -1);
			rgbMat = cv::imread(rgbPath.str().c_str(), 1);

			//converting the images into coloured 3D point clouds
			vector<pointcloud> ptCloud;
			rgbd_utils::getPointCloud(rgbMat,depthMat,ptCloud,myKinect.param);

			//apply the matrix inverse to the corresponding 3D point clouds
			//transform3DPtClouds(ptCloud, transMatInv);

			//accumulate the 3D point clouds
			for(int k=0;k<ptCloud.size();k++)
			{
				pointcloud pt;
				pt = ptCloud[k];
				allPoints.push_back(pt);
			}

			//display the image
			imshow("color", rgbMat);
			waitKey(0);
		}
	}


	//viewing the point cloud in OpenCV
	//plot the 3D point cloud using OpenGL
	ptCloudViz viz("Kinect Point Cloud");
	viz.update(allPoints);
	char c;
	while(c!=27)
	{
		c = cvWaitKey(10);
	}

	return 0;
}
