/*
 * rgbd_utils.cpp
 *
 *  Created on: 19/07/2011
 *      Author: dennislui
 */

#include "rgbd_utils.h"
#include "depthModel.h"
#include "opencv_utils.h"

namespace rgbd_utils{

static depthModel depth_model;
static Mat P3D(3,1,CV_32F);
static Mat P3D2(3,1,CV_32F);

//returns qcloud of depth image (u,v,q)
void getQCloud(Mat &depthMat, Mat &qCloud, kinectParam &param)
{
	ushort* depthPtr;
	float* qCldPtr;
	float metricDepth, depthValue;
	int x;

	float fx_d = param.depth_intrinsics.at<float>(0,0);
	float fy_d = param.depth_intrinsics.at<float>(1,1);
	float cx_d = param.depth_intrinsics.at<float>(0,2);
	float cy_d = param.depth_intrinsics.at<float>(1,2);

	for(int i=0;i<depthMat.rows;i++)
	{
		depthPtr = (ushort*) depthMat.ptr<ushort>(i);
		qCldPtr = (float*) qCloud.ptr<float>(i);
		for(int j=0;j<depthMat.cols;j++)
		{
			x = 3*j;
			depthValue = (float)depthPtr[j];
			if(depthValue < 2047)
			{
				metricDepth = pseudo2MetricDepth(depthValue);
				qCldPtr[x+0] = ((float)j-cx_d)/fx_d; //u
				qCldPtr[x+1] = ((float)i-cy_d)/fy_d; //v
				qCldPtr[x+2] = 1.0/metricDepth; //q
			}
			else
			{
				qCldPtr[x+0] = -1000; //u
				qCldPtr[x+1] = -1000; //v
				qCldPtr[x+2] = -1000; //q
			}
		}
	}
}

//aligns RGB image in reference to depth image
void align(Mat &rgbMat, Mat &depthMat, Mat &aligned)
{

}

//produces colour point cloud
void getPointCloud(Mat &rgbMat, Mat &depthMat, vector<pointcloud> &ptCloud, kinectParam &param)
{
	ptCloud.clear();
	pointcloud point;
	Point2i rgbCoord;
	float metricDepth;
	int prev_y = -1;
	uchar* rgbPtr;
	ushort* depthPtr;

	float fx_rgb = param.rgb_intrinsics.at<float>(0,0);
	float fy_rgb = param.rgb_intrinsics.at<float>(1,1);
	float cx_rgb = param.rgb_intrinsics.at<float>(0,2);
	float cy_rgb = param.rgb_intrinsics.at<float>(1,2);

	float fx_d = param.depth_intrinsics.at<float>(0,0);
	float fy_d = param.depth_intrinsics.at<float>(1,1);
	float cx_d = param.depth_intrinsics.at<float>(0,2);
	float cy_d = param.depth_intrinsics.at<float>(1,2);

	for(int j=0;j<depthMat.rows;j++)
	{
		depthPtr = (ushort*) depthMat.ptr<ushort>(j);
		for(int k=0;k<depthMat.cols;k++)
		{
			float depthValue = (float)depthPtr[k];
			if(depthValue < 2047 && depthValue > 0)
			{
				metricDepth = pseudo2MetricDepth(depthValue);
				P3D.at<float>(0,0) = (((float)k - cx_d)*metricDepth/fx_d);
				P3D.at<float>(1,0) = (((float)j - cy_d)*metricDepth/fy_d);
				P3D.at<float>(2,0) = (metricDepth);

				cv::gemm(param.R,P3D,1.0,param.T,1.0,P3D2);
				rgbCoord.x = (P3D2.at<float>(0,0)*fx_rgb/P3D2.at<float>(2,0)) + cx_rgb;
				rgbCoord.y = (P3D2.at<float>(1,0)*fy_rgb/P3D2.at<float>(2,0)) + cy_rgb;

				if(rgbCoord.x < 640 && rgbCoord.y < 480)
				{
					if(prev_y!=rgbCoord.y)
						rgbPtr = rgbMat.ptr<uchar>(rgbCoord.y);

					point.blue = rgbPtr[3*rgbCoord.x+0]; //blue
					point.green = rgbPtr[3*rgbCoord.x+1]; //green
					point.red = rgbPtr[3*rgbCoord.x+2]; //red

					point.x = P3D.at<float>(0,0);
					point.y = P3D.at<float>(1,0);
					point.z = P3D.at<float>(2,0);

					point.u = k; //columns (x direction)
					point.v = j; //rows (y direction)

					ptCloud.push_back(point);

					prev_y = rgbCoord.y;
				}
			}
		}
	}
}

//converts depth pixel(imgrow, imgcol, pseudoDepth) to 3D coordinates (x,y,z)
inline void depthPix2xyz(int imgrow, int imgcol, int pseudoDepth, kinectParam &param, Mat &out)
{
	float metricDepth = pseudo2MetricDepth(pseudoDepth);
	out.at<double>(0,0) = (((double)imgcol - param.depth_intrinsics.at<float>(0,2))*metricDepth/param.depth_intrinsics.at<float>(0,0));
	out.at<double>(1,0) = (((double)imgrow - param.depth_intrinsics.at<float>(1,2))*metricDepth/param.depth_intrinsics.at<float>(1,1));
	out.at<double>(2,0) = (metricDepth);
}

//converts 3D coordinates of depth pixel to image coordinates on the RGB camera
inline void xyz2rgbPix(Mat &xyz, kinectParam &param, Point2i &pixLocation)
{
	Mat P3D2(3,1,CV_64F);
	cv::gemm(param.R,xyz,1.0,param.T,1.0,P3D2,0);
	pixLocation.x = (P3D2.at<float>(0,0)*param.rgb_intrinsics.at<float>(0,0)/P3D2.at<float>(2,0)) + param.rgb_intrinsics.at<float>(0,2);
	pixLocation.y = (P3D2.at<float>(1,0)*param.rgb_intrinsics.at<float>(1,1)/P3D2.at<float>(2,0)) + param.rgb_intrinsics.at<float>(1,2);
}

//converts pseudo-depth to metric depth values according to depth model
float pseudo2MetricDepth(int pseudo_depth)
{
	if (pseudo_depth < 2047)
		return depth_model.depthLUT.at<float>(0,pseudo_depth);
	return 0;
}

//converts metric depth values to pseudo-depth values
inline int metric2PseudoDepth(float metric_depth)
{
	int pseudo_depth = round( ((1.0/metric_depth) - 3.3309495161)/ (-0.0030711016) );
	return pseudo_depth;
}

}
