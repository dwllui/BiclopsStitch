/*
 * opencv_utils.cpp
 *
 *  Created on: 19/07/2011
 *      Author: dennislui
 */
#include "opencv_utils.h"


namespace opencv_utils{

//writes matrix to yml file
void writeMatrix(FileStorage& output_file, const std::string& name, const cv::Mat& matrix)
{
	CvMat m = matrix;
	output_file.writeObj(name, &m);
}

//reads matrix from yml file
void readMatrix(FileStorage& input_file, const std::string& name, cv::Mat& matrix)
{
    FileNode node = input_file[name];
    CvMat* m;
    m = (CvMat*)node.readObj();
    if(!m)
    	cerr << "Could not read field " << name << " from yml file." << endl;
    matrix = m;
}

//given a transformation matrix T, extract the 6 motion parameters
void extractMotionParam(Mat &T, Mat &dof6param1, Mat &dof6param2){

	if(T.at<float>(2,0) != 1 && T.at<float>(2,0) != -1 )
	{
		//Ry
		dof6param1.at<float>(0,1) = atan2(T.at<float>(2,1), T.at<float>(2,2));
		//dof6param2.at<float>(0,1) = atan2(T.at<float>(2,1), T.at<float>(2,2));

		//Rx
		dof6param1.at<float>(0,0) = atan2(-cos(dof6param1.at<float>(0,1))*T.at<float>(2,0),T.at<float>(0,0) );
		//dof6param1.at<float>(0,0) = asin(T.at<float>(2,0));
		//dof6param2.at<float>(0,0) = CV_PI - dof6param1.at<float>(0,0);

		//Rz
		dof6param1.at<float>(0,2) = atan2(T.at<float>(1,0)/cos(dof6param1.at<float>(0,0)), T.at<float>(0,0)/cos(dof6param1.at<float>(0,0)));
		//dof6param2.at<float>(0,2) = atan2(T.at<float>(1,0), T.at<float>(0,0));
		//dof6param1.at<float>(0,2) = atan2(T.at<float>(1,0)/cos(-asin(T.at<float>(2,0))), T.at<float>(0,0)/cos(-asin(T.at<float>(2,0))));

	}
	else
	{
		/*
		dof6param1.at<float>(0,2) = 0;
		if(T.at<float>(2,0) != -1)
		{
			dof6param1.at<float>(0,0) = CV_PI/2.0;
			dof6param1.at<float>(0,1) = dof6param1.at<float>(0,2) + atan2(T.at<float>(0,1),T.at<float>(0,2));
		}
		else
		{
			dof6param1.at<float>(0,0) = -CV_PI/2.0;
			dof6param1.at<float>(0,1) = -dof6param1.at<float>(0,2) + atan2(-T.at<float>(0,1),-T.at<float>(0,2));
		}
		*/
	}

	double temp;
	dof6param1.at<float>(0,0) = dof6param1.at<float>(0,0)/CV_PI*180.0;
	dof6param1.at<float>(0,1) = dof6param1.at<float>(0,1)/CV_PI*180.0;
	dof6param1.at<float>(0,2) = dof6param1.at<float>(0,2)/CV_PI*180.0;
	temp = dof6param1.at<float>(0,0);
	dof6param1.at<float>(0,0) = dof6param1.at<float>(0,1);
	dof6param1.at<float>(0,1) = temp;

//	dof6param2.at<float>(0,0) = dof6param2.at<float>(0,0)/CV_PI*180.0;
//	dof6param2.at<float>(0,1) = dof6param2.at<float>(0,1)/CV_PI*180.0;
//	dof6param2.at<float>(0,2) = dof6param2.at<float>(0,2)/CV_PI*180.0;
//	temp = dof6param2.at<float>(0,0);
//	dof6param2.at<float>(0,0) = dof6param2.at<float>(0,1);
//	dof6param2.at<float>(0,1) = temp;

	dof6param1.at<float>(0,3) = T.at<float>(0,3);
	dof6param2.at<float>(0,3) = T.at<float>(0,3);
	dof6param1.at<float>(0,4) = T.at<float>(1,3);
	dof6param2.at<float>(0,4) = T.at<float>(1,3);
	dof6param1.at<float>(0,5) = T.at<float>(2,3);
	dof6param2.at<float>(0,5) = T.at<float>(2,3);
}

}
