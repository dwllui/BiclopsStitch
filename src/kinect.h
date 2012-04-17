/*
 * kinect.h
 *
 *  Created on: 18/07/2011
 *      Author: dennislui
 */

#include "freenect/myFreenect.h"
#include "libfreenect.h"
#include "opencv2/opencv.hpp"
#include "utilities/datatypes.h"

using namespace cv;

#ifndef KINECT_H_
#define KINECT_H_

class kinect{
	public:
		kinect(const char *filename, bool undistort);
		kinect(int devNum, const char *filename, bool undistort);
		~kinect();
		//starts the callback loops
		void start();
		//stops the callback loops
		void stop();
		//returns the RGB image
		void getVideo(Mat &img);
		//returns the Depth image
		void getDepth(Mat &img);
		//returns the accelerometer's data
		void getAcc(accData &acc);
		//setting the LED colour on the kienct
		void setLED(freenect_led_options _options);
		//undistort the RGB image
		void undistortColor(Mat &img);
		//undistort the depth image
		void undistortDepth(Mat &img);

		//acessbile variables
		int deviceNum; //specified device number for the kinect
		kinectParam param; //parameters of the kinect
		Size raw_rgb_size,raw_depth_size;

	private:
		//reads calibration data from file
		bool calib_fromfile(const char *filename);
		//compute map for distortion correction
		void undistort_mapping();

		Freenect::Freenect freenect;
		MyFreenectDevice* device;
		Mat rgbMat, depthMat, remapped_rgb, remapped_d;
		Mat mapx_rgb, mapy_rgb, mapx_d, mapy_d;
		bool undistort_flag, zero_depth_distortion, zero_rgb_distortion;
};

#endif /* KINECT_H_ */
