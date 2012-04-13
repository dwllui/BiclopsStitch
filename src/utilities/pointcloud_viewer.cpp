/*
 * pointcloud_viewer.cpp
 *
 *  Created on: 20/07/2011
 *      Author: dennislui
 */

#include "pointcloud_viewer.h"
#include <stdio.h>
#include <GL/gl.h>
#include <GL/glut.h>

ptCloudViz::ptCloudViz(string winname){
	visWinname = winname;
	//creating window for visualization of 3D data
	//namedWindow(visWinname.c_str(),CV_GUI_NORMAL | CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO | CV_WINDOW_NORMAL);
	namedWindow(visWinname.c_str(),CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL);
	cvCreateOpenGLCallback(visWinname.c_str(),on_opengl,this);
	cvSetMouseCallback(visWinname.c_str(),on_mouse,this);

	mx=-1;
	my=-1;        // Previous mouse coordinates
	rotangles[0] = 0; // Panning angles
	rotangles[1] = 0; // Panning angles
	panDist_x = -1.0;
	panDist_y = 0.0;
	zoom = 1.6;         // zoom factor

	//zero = Mat::zeros(Size(640,480), CV_8UC3);
	zero = Mat::zeros(Size(1280,960), CV_8UC3);
}

ptCloudViz::~ptCloudViz(){
	cvDestroyWindow(visWinname.c_str());
}

void ptCloudViz::update(vector<pointcloud> newPtCloud)
{
	ptCloud = newPtCloud;
	imshow(visWinname,zero);
}

//mouse callback function for visualization
void ptCloudViz::on_mouse(int event, int x, int y, int flags, void* obj){

	if(event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_SHIFTKEY))
	{
		ptCloudViz* myObj = (ptCloudViz*) obj; //recast
		myObj->panDist_x += ((float)x-(float)myObj->mx)/100.0;
		myObj->panDist_y += ((float)y-(float)myObj->my)/100.0;

		myObj->mx = x;
		myObj->my = y;
	}
	else if( event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON) )
	 {
		ptCloudViz* myObj = (ptCloudViz*) obj; //recast
		 if(myObj->mx>=0 && myObj->my>0){
			 myObj->rotangles[0] += y-myObj->my;
			 myObj->rotangles[1] += x-myObj->mx;
		 }
		 myObj->mx = x;
		 myObj->my = y;
	 }
	else if(event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_MBUTTON))
	{
		ptCloudViz* myObj = (ptCloudViz*) obj; //recast
		myObj->zoom += ((float)y-(float)myObj->my)/20.0;
		if(myObj->zoom < 0)
			myObj->zoom = 0.0;

		myObj->mx = x;
		myObj->my = y;
	}
	else if(event == CV_EVENT_MOUSEMOVE)
	{
		ptCloudViz* myObj = (ptCloudViz*) obj; //recast
		myObj->mx = x;
		myObj->my = y;
	}

}

//opengl callback function for visualization
void ptCloudViz::on_opengl(void *obj)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(255.0f, 255.0f, 255.0f, 1.0f);
	glLoadIdentity();

	ptCloudViz* myObj = (ptCloudViz*) obj; //recast

	glScalef(myObj->zoom,myObj->zoom,1);
	//glTranslatef(0,0,-3.5);
	glTranslatef(myObj->panDist_x,myObj->panDist_y,-8.5);
	//glRotatef(90, 0,1,0);
	glRotatef(myObj->rotangles[0], 1,0,0);
	glRotatef(myObj->rotangles[1], 0,1,0);
	glTranslatef(0,0,1.5);

	glBegin(GL_POINTS);
	glPointSize(1);

	for(int i=0;i<myObj->ptCloud.size();i++)
	{
		glColor3ub(myObj->ptCloud[i].red,myObj->ptCloud[i].green, myObj->ptCloud[i].blue);
		glVertex3f(myObj->ptCloud[i].x,-myObj->ptCloud[i].y,-myObj->ptCloud[i].z);
	}

	glEnd();

	/*
	glBegin(GL_POLYGON);
		glColor3ub(255,0,0);
		glVertex3f(kp.p1.x, kp.p1.y, kp.p1.z);
		glColor3ub(0,255,0);
		glVertex3f(kp.p2.x, kp.p2.y, kp.p2.z);
		glColor3ub(0,0,255);
		glVertex3f(kp.p3.x, kp.p3.y, kp.p3.z);
	glEnd();
	*/
}
