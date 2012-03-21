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

#include <stdio.h>
#include "libcam.h"
#include "image.h"

void draw_circles(IplImage* img) {
    int ball=0;
    while(circles_st[ball].x_in_picture != -1) {
/*
        circles_st[ball].x_in_picture = p[0];
        circles_st[ball].y_in_picture = p[1];

        if(data[(circles_st[ball].y_in_picture*step
            +circles_st[ball].x_in_picture)]<120)
            continue;

        circles_st[ball].r_in_picture = p[2];
        circles_st[ball].y_from_robot = find_distance_y(p[1]);
        circles_st[ball].x_from_robot = find_distance_x(circles_st[ball].y_from_robot, p[0]);
        circles_st[ball].r_from_robot = find_distance_x(circles_st[ball].y_from_robot, p[0]+p[2]) - circles_st[ball].x_from_robot;
*/
printf("%s, %d %d\n",__FILE__,__LINE__,ball);
	
        cvCircle(img, cvPoint (circles_st[ball].x_in_picture,circles_st[ball].y_in_picture), 10, cvScalarAll(0), 1, 8, 0);
        //void cvCircle(CvArr* img, CvPoint center, int radius, CvScalar color, int thickness=1, int lineType=8, int shift=0)
	ball++;
    }
}

