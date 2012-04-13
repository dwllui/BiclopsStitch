/*
 * rgbd_utils.h
 *
 *  Created on: 19/07/2011
 *      Author: dennislui
 */

#include "datatypes.h"
#include "opencv2/opencv.hpp"

using namespace cv;

#ifndef RGBD_UTILS_H_
#define RGBD_UTILS_H_

namespace rgbd_utils{

//returns qcloud of depth image
void getQCloud(Mat &depthMat, Mat &qCloud, kinectParam &param);
//aligns RGB image in reference to depth image
void align(Mat &rgbMat, Mat &depthMat, Mat &aligned);
//produces colour point cloud
void getPointCloud(Mat &rgbMat, Mat &depthMat, vector<pointcloud> &ptCloud, kinectParam &param);

//converts depth pixel(imgrow, imgcol, pseudoDepth) to 3D coordinates (x,yz)
inline void depthPix2xyz(int imgrow, int imgcol, int pseudoDepth, kinectParam &param, Mat &out);
//converts 3D coordinates of depth pixel to image coordinates on the RGB camera
inline void xyz2rgbPix(Mat &xyz, kinectParam &param, Point2i &pixLocation);

//converts pseudo-depth to metric depth values according to depth model
float pseudo2MetricDepth(int pseudo_depth);
//converts metric depth values to pseudo-depth values
inline int metric2PseudoDepth(float metric_depth);

}

#endif /* RGBD_UTILS_H_ */
