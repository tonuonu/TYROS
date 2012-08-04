/*
 *  Copyright (c) 2011, Tõnu Samuel
 *  All rights reserved.
 *
 *  This file is part of TYROS.
 *
 *  TYROS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  TYROS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with TYROS.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/legacy/compat.hpp"
#include "opencv2/objdetect/objdetect.hpp"


#include "libcam.h"
#include "image.h"
#include <stdio.h>


struct circles circles_st[11];


double find_distance_y(double pixel_y){
	double distance_y = 
	tan(CAMERA_ANGLE_Y_START+(PIXEL_IN_RADIANS*(IMAGE_HEIGHT-pixel_y)))*(CAMERA_HEIGHT-BALL_RADIUS*2);
       return distance_y;
}


double find_distance_x(double distance_y, double pixel_x){
  double angle = (pixel_x/IMAGE_WIDTH-1)*(CAMERA_X_ANGLE/2);
  double distance_x = tan(angle)*distance_y;
	return distance_x;
}


void find_circles(IplImage* input) {
	IplImage* gray;
	CvMemStorage* storage;
	gray = cvCloneImage(input);
	storage = cvCreateMemStorage(0);

	cvSmooth(gray, gray, CV_GAUSSIAN, 5, 5, 0, 0); 

	// cvHoughCircles(image, circleStorage, method, dp, minDist, param1, param2, minRadius, maxRadius)
	CvSeq* results = cvHoughCircles(gray, storage, CV_HOUGH_GRADIENT, 2, 10, 60, 40, 3, 133);
	//CvSeq* results = cvHoughCircles(gray, storage, CV_HOUGH_GRADIENT, 2, 40, 60, 40, 3, 33);
	int i;
	int ball = 0;
	for (i=0; i<results->total && i<10; i++) {
//printf("%s, %d \n",__FILE__,__LINE__);
		float* p = (float*)cvGetSeqElem(results, i);
		int step = gray->widthStep/sizeof(uchar);
		unsigned char* data = (unsigned char *)gray->imageData;

		circles_st[ball].x_in_picture = p[0];
		circles_st[ball].y_in_picture = p[1];

               if(data[(circles_st[ball].y_in_picture*step
                       +circles_st[ball].x_in_picture)]<120)
                       continue;
//printf("%s, %d \n",__FILE__,__LINE__);
		circles_st[ball].r_in_picture = p[2];
		circles_st[ball].y_from_robot = find_distance_y(p[1]);
		circles_st[ball].x_from_robot = find_distance_x(circles_st[ball].y_from_robot, p[0]);
		circles_st[ball].r_from_robot = find_distance_x(circles_st[ball].y_from_robot, p[0]+p[2]) - circles_st[ball].x_from_robot;
		/*
		if ((BALL_RADIUS-0.5)> circles_st[ball].r_from_robot || 
			circles_st[ball].r_from_robot > (BALL_RADIUS+0.5)) 
			continue;*/
		//rosmain((double)p[0],(double)p[1],(double)p[2]);
		ball++;
	}
	circles_st[ball].x_in_picture = -1; 
	assert(gray!=0);
	cvReleaseImage(&gray);
	if(results)
		cvClearSeq(results);
	cvReleaseMemStorage( &storage );
	return ;
}

