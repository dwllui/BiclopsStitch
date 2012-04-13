/*
 * qcloud_gen.h
 *
 *  Created on: 02/08/2011
 *      Author: dennislui
 */

#include "opencv2/opencv.hpp"
#include "depthModel.h"
#include "datatypes.h"

using namespace cv;

#ifndef QCLOUD_GEN_H_
#define QCLOUD_GEN_H_

class qcloud_gen{
	public:
		qcloud_gen(kinectParam &param);
		~qcloud_gen();
		void getQCloud(Mat &depthMat, Mat &qCloud);

	private:
		depthModel depth_model;
		Mat uLUT, vLUT, qLUT;
};


#endif /* QCLOUD_GEN_H_ */
