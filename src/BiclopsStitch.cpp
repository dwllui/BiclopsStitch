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

//function to write pointcloud data into .ply file format
void write2ply(vector<pointcloud> &ptCloud, string filePath);

void transform3DPtClouds(vector<pointcloud> &ptCloud, Mat &transMat);
//some handy transformation matrices
Mat rotX(float angle);
Mat rotY(float angle);
Mat rotZ(float angle);
Mat transX(float disp);
Mat transY(float disp);
Mat transZ(float disp);

/****************************LOADING SCANNED IMAGES CAPTURED AT THE FOLLOWING ANGLES******************************/
//double tiltAngles[] = {0.0,-30.0, 30.0};
double tiltAngles[] = {-30.0, 30.0, 0.0};
//pan angles
double panAngles[] = {-150.0, -120.0, -90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0};
//double panAngles[] = {0.0, 30.0,60.0};
/*****************************************************************************************************************/
string mainFolder = "/media/Data/Bionic Eye/Experimental Datasets/Biclops Scan/Scan2"; 	//main folder where scanned images reside
string meshlab_filepath = "pointcloud4.ply"; //specify file path to store mesh lab file


int main() {

	char calib_filepath[] = "calib/pantilt.yml";
	kinect myKinect(calib_filepath, true);

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
	Mat forwardTransMat(4,4,CV_32F);
	float base2tilt = 0.12069; //in meters
	float tilt2plat = 0.03175; //in meters

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

			//undistorting the images first to get better Depth and RGB alignment
			myKinect.undistortColor(rgbMat);
			myKinect.undistortDepth(depthMat);

			//converting the images into coloured 3D point clouds
			vector<pointcloud> ptCloud;
			rgbd_utils::getPointCloud(rgbMat,depthMat,ptCloud,myKinect.param);

			//computing forward transformation matrix (base of pan tilt to the kinect)
			//TODO: have yet to include offset from the optical center of the depth camera relative to the biclops platform
			forwardTransMat = rotX(90.0)*transZ(tilt2plat)*rotZ(-90.0)*transZ(base2tilt)*rotZ(90.0)*rotX(-tiltAngles[i])*rotZ(-panAngles[j]);

			//finding the inverse of the matrix
			Mat transMatInv(4,4,CV_32F);
			invert(forwardTransMat, transMatInv, CV_SVD);

			//apply the matrix inverse to the corresponding 3D point clouds
			transform3DPtClouds(ptCloud, transMatInv);

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

	//USE THIS FUNCTION TO WRITE TO MESHLAB .PLY FILE
	//writing to .ply file for viewing in meshlab
	write2ply(allPoints, meshlab_filepath);

	//UNCOMMENT THE CODE BELOW TO VISUALIZE POINT CLOUD USING OPENCV with OPENGL
//	ptCloudViz viz("Kinect Point Cloud");
//	viz.update(allPoints);
//	char c;
//	while(c!=27)
//	{
//		c = cvWaitKey(20);
//	}

	return 0;
}

Mat rotX(float degrees)
{
	float radians = degrees/180.0*CV_PI;
	Mat rotMat = Mat::eye(4,4,CV_32F);
	rotMat.at<float>(1,1) = cos(radians);
	rotMat.at<float>(1,2) = -sin(radians);
	rotMat.at<float>(2,1) = sin(radians);
	rotMat.at<float>(2,2) = cos(radians);

	return rotMat;
}

Mat rotY(float degrees){

	float radians = degrees/180.0*CV_PI;
	Mat rotMat = Mat::eye(4,4,CV_32F);
	rotMat.at<float>(0,0) = cos(radians);
	rotMat.at<float>(0,2) = sin(radians);
	rotMat.at<float>(2,0) = -sin(radians);
	rotMat.at<float>(2,2) = cos(radians);

	return rotMat;
}

Mat rotZ(float degrees){

	float radians = degrees/180.0*CV_PI;
	Mat rotMat = Mat::eye(4,4,CV_32F);
	rotMat.at<float>(0,0) = cos(radians);
	rotMat.at<float>(0,1) = -sin(radians);
	rotMat.at<float>(1,0) = sin(radians);
	rotMat.at<float>(1,1) = cos(radians);

	return rotMat;
}

Mat transX(float disp){
	Mat transMat = Mat::eye(4,4,CV_32F);
	transMat.at<float>(0,3) = disp;

	return transMat;
}

Mat transY(float disp){
	Mat transMat = Mat::eye(4,4,CV_32F);
	transMat.at<float>(1,3) = disp;

	return transMat;
}

Mat transZ(float disp){
	Mat transMat = Mat::eye(4,4,CV_32F);
	transMat.at<float>(2,3) = disp;

	return transMat;
}

//transform 3D point clouds
void transform3DPtClouds(vector<pointcloud> &ptCloud, Mat &transMat)
{
	Mat coords = Mat::ones(4,1,CV_32F);
	Mat transCoords = Mat::ones(4,1,CV_32F);

	for(int i=0;i<ptCloud.size();i++)
	{
		//put 3D points into 4x1 homogeneous matrix
		coords.at<float>(0,0) = (float)ptCloud[i].x;
		coords.at<float>(1,0) = (float)ptCloud[i].y;
		coords.at<float>(2,0) = (float)ptCloud[i].z;
		coords.at<float>(3,0) = 1.0;

		//apply transMat to points
		transCoords = transMat*coords;

		//update x,y,z in ptCloud
		ptCloud[i].x = transCoords.at<float>(0,0);
		ptCloud[i].y = transCoords.at<float>(1,0);
		ptCloud[i].z = transCoords.at<float>(2,0);
	}
}

void write2ply(vector<pointcloud> &ptCloud, string filePath){
	FILE* rgbdFILE = fopen(filePath.c_str(), "w+");

	//total points in point cloud
	int count = ptCloud.size();

	//headers required for '.ply' file
	fprintf(rgbdFILE, "%s\n", "ply");
	fprintf(rgbdFILE, "%s\n", "format ascii 1.0");
	fprintf(rgbdFILE, "%s ", "element vertex");
	fprintf(rgbdFILE, "%d\n", count);
	fprintf(rgbdFILE, "%s\n", "property float x");
	fprintf(rgbdFILE, "%s\n", "property float y");
	fprintf(rgbdFILE, "%s\n", "property float z");
	fprintf(rgbdFILE, "%s\n", "property uchar red");
	fprintf(rgbdFILE, "%s\n", "property uchar green");
	fprintf(rgbdFILE, "%s\n", "property uchar blue");
	fprintf(rgbdFILE, "%s\n", "end_header");

	//writing to file
	for(int j=0;j<count;j++)
	{
		fprintf(rgbdFILE, "%f %f %f %u %u %u\n", ptCloud[j].x, ptCloud[j].y, ptCloud[j].z,
							ptCloud[j].red, ptCloud[j].green, ptCloud[j].blue);
	}

	fclose(rgbdFILE);
}
