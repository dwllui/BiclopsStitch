/*
 * depthModel.cpp
 *
 *  Created on: 19/07/2011
 *      Author: dennislui
 */

#include "depthModel.h"
#include <math.h>

depthModel::depthModel(){
	//initializing lookup table to obtain metric depth values
	//createLUT();
	createLUT_tanh();
}

depthModel::~depthModel(){

}

void depthModel::createLUT(){
	depthLUT = Mat(1,2047,CV_32FC1);
	for(int i=0;i<2047;i++)
		depthLUT.at<float>(0,i) = 1.0 / ((double)i * (-0.0030711016) + 3.3309495161);
}

////depth model using tanh
void depthModel::createLUT_tanh()
{
	depthLUT = Mat(1,2047,CV_32FC1);
	const float k1 = 1.1863;
	const float k2 = 2842.5;
	const float k3 = 0.1236;

	for(int i=0;i<2047;i++)
			depthLUT.at<float>(0,i) = k3 * tan((double)i/k2 + k1);

}
