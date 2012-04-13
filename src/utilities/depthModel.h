/*
 * depthModel.h
 *
 *  Created on: 19/07/2011
 *      Author: dennislui
 */

#include "opencv2/opencv.hpp"

using namespace cv;

#ifndef DEPTHMODEL_H_
#define DEPTHMODEL_H_

class depthModel{
	public:
		depthModel();
		~depthModel();

		Mat depthLUT;

	private:
		void createLUT();
		void createLUT_tanh();
};

#endif /* DEPTHMODEL_H_ */
