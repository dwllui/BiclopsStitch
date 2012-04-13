/*
 * pointcloud_viewer.h
 *
 *  Created on: 20/07/2011
 *      Author: dennislui
 */

#include "opencv2/opencv.hpp"
#include "datatypes.h"

using namespace cv;

#ifndef POINTCLOUD_VIEWER_H_
#define POINTCLOUD_VIEWER_H_

class ptCloudViz{
	public:
		ptCloudViz(string winname);
		~ptCloudViz();
		//refreshing the display with new point clouds
		void update(vector<pointcloud> newPtCloud);

	private:
		static void on_mouse( int event, int x, int y, int flags, void* obj);
		static void on_opengl(void *obj);

		vector<pointcloud> ptCloud;

		int mx,my;        // Previous mouse coordinates
		int rotangles[2]; // Panning angles
		float panDist_x; //panning distances
		float panDist_y; //panning distances
		float zoom;        // zoom factor
		string visWinname; //window name for OpenGL visualisation of 3D point clouds
		Mat zero;
};

#endif /* POINTCLOUD_VIEWER_H_ */
