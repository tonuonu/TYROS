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

void *ProcessImages(void *thdptr) ;


// YUYV byte order
#define Y1 0
#define U  1
#define Y2 2
#define V  3

#define ROTATE90 1
#ifdef ROTATE90
#define IMAGE_WIDTH 240
#define IMAGE_HEIGHT 320
#else
#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240
#endif


#define PI 3.14159265358979323846264338327950288419716939937
#define CAMERA_HEIGHT 24.0
#define BORDER_HEIGHT 7.0
#define CAMERA_ANGLE_Y_START ((2.0*PI)/(360.0/24.0)) // ~0.4
#define CAMERA_Y_ANGLE ((2.0*PI)/(360.0/61.0)) // ~1.06
#define CAMERA_X_ANGLE ((2.0*PI)/(360.0/(61.0/4.0*3.0)))
#define PIXEL_IN_RADIANS (CAMERA_Y_ANGLE/IMAGE_HEIGHT) 
#define CROWN_RADIUS 9.0
#define BALL_RADIUS 2.13
#define CAMERA_CENTER_HEIGHT CAMERA_ANGLE_Y_START+(CAMERA_Y_ANGLE/2)

struct circles {
        int x_in_picture;
        int y_in_picture;
        int r_in_picture;
        double x_from_robot;
        double y_from_robot;
        double r_from_robot;
};

extern struct circles circles_st[11];


//extern pthread_mutex_t image_mutex ;
extern int            imageflag[2][4];

void process_image (const unsigned char *p,int devnum);

int read_frame (int fd);


void errno_exit(const char * s);
extern int fd[8];
void open_device (int fd);
void start_capturing (int fd);
void stop_capturing (int fd);

void find_circles(IplImage* input) ;
void draw_circles(IplImage* input) ;


#ifdef USE_SPINLOCK
extern pthread_spinlock_t spinlock;
#else
extern pthread_mutex_t mutex;
#endif


extern const	char *camdirection[8];

/* Used by main to communicate with parse_opt. */
struct arguments     {
        char *args[2];                /* arg1 & arg2 */
        int silent, verbose;
        int headless;
        char *weight;
        char *camera;
        char *image_name;
};
extern struct arguments *arguments;
