/*
 * qcloud_gen.cpp
 *
 *  Created on: 02/08/2011
 *      Author: dennislui
 */

#include "qcloud_gen.h"

qcloud_gen::qcloud_gen(kinectParam &param){

	float fx_d = param.depth_intrinsics.at<float>(0,0);
	float fy_d = param.depth_intrinsics.at<float>(1,1);
	float cx_d = param.depth_intrinsics.at<float>(0,2);
	float cy_d = param.depth_intrinsics.at<float>(1,2);

	uLUT = Mat(1,param.depth_size.width,CV_32F);
	//create the lookup tables for U and V values
	for(int i=0;i<param.depth_size.width;i++)
		uLUT.at<float>(0,i) = ((float)i-cx_d)/fx_d;

	vLUT = Mat(1,param.depth_size.height,CV_32F);
	for(int i=0;i<param.depth_size.height;i++)
		vLUT.at<float>(0,i) = ((float)i-cy_d)/fy_d;

	qLUT = Mat(1,depth_model.depthLUT.cols, CV_32F);
	for(int i=0;i<depth_model.depthLUT.cols;i++)
		qLUT.at<float>(0,i) = 1.0/depth_model.depthLUT.at<float>(0,i);
}

qcloud_gen::~qcloud_gen(){

}

void qcloud_gen::getQCloud(Mat &depthMat, Mat &qCloud){
	ushort* depthPtr;
	float* qCldPtr;
	//float metricDepth;
	ushort depthValue;
	int x;
	float *uLUTPtr, *vLUTPtr, *qLUTPtr;

	uLUTPtr = (float*)uLUT.ptr<float>(0);
	vLUTPtr = (float*)vLUT.ptr<float>(0);
	qLUTPtr = (float*)qLUT.ptr<float>(0);

	for(int i=0;i<depthMat.rows;i++)
	{
		depthPtr = (ushort*) depthMat.ptr<ushort>(i);
		qCldPtr = (float*) qCloud.ptr<float>(i);
		for(int j=0;j<depthMat.cols;j++)
		{
			x = 3*j;
			depthValue = depthPtr[j];
			if(depthValue < 2046)
			{
				//metricDepth = pseudo2MetricDepth(depthValue);
				qCldPtr[x+0] = uLUTPtr[j]; //u
				qCldPtr[x+1] = vLUTPtr[i]; //v
				qCldPtr[x+2] = qLUTPtr[depthValue]; //q
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
