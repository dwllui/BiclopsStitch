/*
 * kinect.cpp
 *
 *  Created on: 18/07/2011
 *      Author: dennislui
 */

#include "kinect.h"
#include "utilities/opencv_utils.h"
#include <stdio.h>

using namespace opencv_utils;

kinect::kinect(const char *filename, bool undistort)
{
	//reading the calibration parameters from file
	if(calib_fromfile(filename) && undistort){
		undistort_mapping();}
}

kinect::kinect(int devNum, const char *filename, bool undistort)
{
	//constructor
	deviceNum = devNum;
	device = &freenect.createDevice<MyFreenectDevice>(deviceNum);

	undistort_flag = false;
	//reading the calibration parameters from file
	if(calib_fromfile(filename) && undistort){
		undistort_mapping();}

//	printMatrix<float>(param.rgb_intrinsics, "rgb_intrinsics");
//	printMatrix<float>(param.rgb_distortion, "rgb_distortion");
//	printMatrix<float>(param.depth_intrinsics, "depth_intrinsics");
//	printMatrix<float>(param.depth_distortion, "depth_distortion");
//	printMatrix<float>(param.T, "T");
//	printMatrix<float>(param.R, "R");

	//initializing the matrices to store the images
	rgbMat = Mat(param.rgb_size,CV_8UC3,Scalar(0));
	depthMat = Mat(param.depth_size,CV_16UC1);
}

kinect::~kinect()
{
	//destructor
}

void kinect::start(){
	device->startVideo();
	device->startDepth();
}

void kinect::stop(){
	device->stopVideo();
	device->stopDepth();
}

void kinect::setLED(freenect_led_options _options){
	device->setLed(_options);
}

void kinect::getVideo(Mat &img){
	while(!device->getVideo(rgbMat)){
	}
	if(undistort_flag && !zero_rgb_distortion)
	{
		cv::remap(rgbMat,remapped_rgb,mapx_rgb, mapy_rgb, INTER_LINEAR);
		remapped_rgb.copyTo(rgbMat);
	}
	rgbMat.copyTo(img);
}

void kinect::getDepth(Mat &img){
	while(!device->getDepth(depthMat)){
	}
	if(undistort_flag && !zero_depth_distortion)
	{
		cv::remap(depthMat,remapped_d,mapx_d, mapy_d, INTER_NEAREST);
		remapped_d.copyTo(depthMat);
	}
	depthMat.copyTo(img);
}

void kinect::getAcc(accData &acc){
	device->updateState();
	device->getState().getAccelerometers(&acc.x, &acc.y, &acc.z);
	acc.tilt = device->getState().getTiltDegs();
}

bool kinect::calib_fromfile(const char *filename){

	FILE *calibFile = fopen(filename,"r");
	if(calibFile != NULL)
	{
		fclose(calibFile);

		cv::Mat1d intrinsics_r, distortion_r;
		cv::FileStorage calibration_file (filename, CV_STORAGE_READ);
		readMatrix(calibration_file, "rgb_intrinsics", intrinsics_r);
		intrinsics_r.convertTo(param.rgb_intrinsics,CV_32F);
		readMatrix(calibration_file, "rgb_distortion", distortion_r);
		distortion_r.convertTo(param.rgb_distortion,CV_32F);
		zero_rgb_distortion = distortion_r(0,0) < 1e-5 ? true : false;

		cv::Mat1d intrinsics, distortion;
		readMatrix(calibration_file, "depth_intrinsics", intrinsics);
		intrinsics.convertTo(param.depth_intrinsics,CV_32F);
		readMatrix(calibration_file, "depth_distortion", distortion);
		distortion.convertTo(param.depth_distortion,CV_32F);
		zero_depth_distortion = distortion(0,0) < 1e-5 ? true : false;

		cv::Mat1d tempR, tempT;
		readMatrix(calibration_file, "R", tempR);
		tempR.convertTo(param.R,CV_32F);
		param.R = param.R.t();
		readMatrix(calibration_file, "T", tempT);
		tempT.convertTo(param.T,CV_32F);
		param.T = -param.T;

		cv::Mat1i size_mat;
		readMatrix(calibration_file, "rgb_size", size_mat);
		param.rgb_size = cv::Size(size_mat(0,0), size_mat(0,1));
		readMatrix(calibration_file, "raw_rgb_size", size_mat);
		raw_rgb_size = cv::Size(size_mat(0,0), size_mat(0,1));
		cv::Mat1i rgb_size_mat;
		readMatrix(calibration_file, "depth_size", size_mat);
		param.depth_size = cv::Size(size_mat(0,0), size_mat(0,1));
		readMatrix(calibration_file, "raw_depth_size", size_mat);
		raw_depth_size = cv::Size(size_mat(0,0), size_mat(0,1));
		return true;
	}
	else
	{
		cerr << "Calibration file " << filename <<  " not found." << endl;
		return false;
	}
}

void kinect::undistort_mapping(){
	//calculating the mapping from the originally distorted image to the undistorted image
	remapped_rgb = Mat(480,640, CV_8UC3);
	remapped_d = Mat(480,640, CV_16UC1);
	if(!zero_rgb_distortion)
		initUndistortRectifyMap(param.rgb_intrinsics, param.rgb_distortion, Mat(), param.rgb_intrinsics, param.rgb_size, CV_16SC2, mapx_rgb, mapy_rgb);
	if(!zero_depth_distortion)
		initUndistortRectifyMap(param.depth_intrinsics, param.depth_distortion, Mat(), param.depth_intrinsics, param.depth_size, CV_16SC2, mapx_d, mapy_d);
	undistort_flag = true;
}
