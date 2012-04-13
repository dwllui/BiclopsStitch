/*
 * datatypes.h
 *
 *  Created on: 19/07/2011
 *      Author: dennislui
 */

#include "opencv2/opencv.hpp"

using namespace cv;

#ifndef DATATYPES_H_
#define DATATYPES_H_

//accelerometer data structure
struct accData{
	double x;
	double y;
	double z;
	double tilt;
};

//point cloud data structure
struct pointcloud{
	uchar red;
	uchar green;
	uchar blue;
	//3D coordinates
	double x;
	double y;
	double z;
	//2D image coordinates
	int u; //columns (x-direction)
	int v; //rows (y-direction)
};

//kinect parameter data structure
struct kinectParam{
	Mat rgb_intrinsics;
	Mat rgb_distortion;
	Mat depth_intrinsics;
	Mat depth_distortion;
	Mat R; //relative rotation between rgb and depth
	Mat T; //relative translation between rgb and depth
	Size rgb_size;
	Size depth_size;
	double baseline;
	double offset;
};

struct uvqPlane{
	double alpha;
	double beta;
	double gamma;
};

struct plane{
	double a;
	double b;
	double c;
	double d;
};

struct depthPoint{
	int u; //cols (x-direction)
	int v; //rows (y-direction)
	double depth; //can be pseudo or metric depth
};

#endif /* DATATYPES_H_ */
