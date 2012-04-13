/*
 * opencv_utils.h
 *
 *  Created on: 19/07/2011
 *      Author: dennislui
 */

#include <stdio.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

#ifndef OPENCV_UTILS_H_
#define OPENCV_UTILS_H_

namespace opencv_utils{

void writeMatrix(FileStorage& output_file, const std::string& name, const cv::Mat& matrix);
void readMatrix(FileStorage& input_file, const std::string& name, cv::Mat& matrix);

//prints matrix to screen
template<class T> void printMatrix(Mat matrix, string matName)
{
	cout << matName << endl;
	for(int i=0;i<matrix.rows;i++)
	{
		for(int j=0;j<matrix.cols;j++)
			cout << matrix.at<T>(i,j) << " ";

		cout << ";" << endl;
	}
}

void extractMotionParam(Mat &T, Mat &dof6param1, Mat &dof6param2);

}

#endif /* OPENCV_UTILS_H_ */
