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

#include <string>

#include <ros/ros.h>
#include <tyros_camera/Objects.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include "opencv2/opencv.hpp"
#include "libcam.h"
#include "image.h"
#define DEBUG 1

CvRect box;
bool drawing_box = false;
bool setroi = false;

void draw_box( IplImage* img, CvRect rect ) {
    cvRectangle (img,
	cvPoint(box.x,box.y),
	cvPoint(box.x+box.width,box.y+box.height),
        cvScalar(0xff,0x00,0x00)/* red */);
}

void my_mouse_callback(int event, int x, int y, int flags, void* param) {
    IplImage* image = (IplImage*) param;
    switch( event ) {
        case CV_EVENT_MOUSEMOVE: {
            if( drawing_box ) {
                box.width = x-box.x;
                box.height = y-box.y;
            }
        }
        break;
        case CV_EVENT_LBUTTONDOWN: {
            drawing_box = true;
            setroi= false;
            box = cvRect(x, y, 0, 0);
        }
        break;
        case CV_EVENT_LBUTTONUP: {
            drawing_box = false;
            setroi = true;
            if(box.width<0) {
                box.x+=box.width;
                box.width *=-1;
            }
            if(box.height<0) {
                box.y+=box.height;
                box.height*=-1;
            }
            draw_box(image, box);
	    
        }
        break;
    }
}

int main(int argc, char **argv) {
	int height, width, input;
	std::string dev, frame_id, cinfo_url;
	sensor_msgs::Image image;
	sensor_msgs::CameraInfo cam_info;
        box = cvRect(-1,-1,0,0);

	ros::init(argc, argv, "tyros_cam");
	ros::NodeHandle nh("~");

	image_transport::ImageTransport it(nh);
	image_transport::CameraPublisher image_pub = it.advertiseCamera("image_raw", 1);

	nh.param<int>("width", width, IMAGE_WIDTH);
	nh.param<int>("height", height, IMAGE_HEIGHT);
	nh.param<int>("input", input, 0);
	if(argc<2) {
		nh.param<std::string>("device", dev, "/dev/video0");
	} else {
		nh.param<std::string>("device", dev, argv[1]);
		printf("opening %s\n",argv[1]);
	}
	nh.param<std::string>("frame_id", frame_id, "camera");

	ROS_INFO("Opening device : %s", dev.c_str());
	Camera cam(dev.c_str(), IMAGE_WIDTH, IMAGE_HEIGHT);

	cam.setInput(input);

	image.header.frame_id = frame_id;
	image.height = IMAGE_HEIGHT;
	image.width = IMAGE_WIDTH;
	image.encoding = sensor_msgs::image_encodings::MONO8;

        ros::init(argc, argv, "tyros_cam");
        ros::NodeHandle n;
        ros::Rate loop_rate(30);
        ros::Publisher pub;

        pub = n.advertise<tyros_camera::Objects>("vision_publisher", 5); /* Message queue length is 5 */

	IplImage* iply,*iplu,*iplv;
        iply= cvCreateImage(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT), 8, 1);
        iplu= cvCreateImage(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT), 8, 1);
        iplv= cvCreateImage(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT), 8, 1);

#ifdef DEBUG
	cvNamedWindow( "result Y", 0 );
	cvNamedWindow( "result U", 0 );
	cvNamedWindow( "result V", 0 );
        cvStartWindowThread();
	cvSetMouseCallback("result Y",my_mouse_callback,(void*) iply);
#endif
	while (ros::ok()) {
		unsigned char* ptr = cam.Update();
		int i,j;
	
		image.header.stamp = ros::Time::now();
		int image_size = IMAGE_HEIGHT*IMAGE_WIDTH;
		image.step = IMAGE_WIDTH;
		image.data.resize(image_size);
                tyros_camera::Objects msg;

		/* 
                 * For each input pixel we do have 2 bytes of data. But we read them in 
                 * groups of four because of YUYV format.
                 * i represent absolute number of pixel, j is byte.
                 */
#ifdef ROTATE90
               for(i=0,j=0;i<(IMAGE_WIDTH*IMAGE_HEIGHT*2) ; i+=2,j+=1) {
                       int tmp1=((j%IMAGE_HEIGHT)*IMAGE_WIDTH+IMAGE_WIDTH-1);
		       int tmp2=(j%(IMAGE_WIDTH*IMAGE_HEIGHT))/IMAGE_HEIGHT;
                       image.data[tmp1-(int)(tmp2)] = ptr[i+Y1];
                       image.data[tmp1-(int)(tmp2-1)] = ptr[i+Y2];
                       iply->imageData[tmp1-(int)(tmp2)] = ptr[i+Y1];
                       iply->imageData[tmp1-(int)(tmp2-1)] = ptr[i+Y2];
                       iplu->imageData[tmp1-(int)(tmp2)] = ptr[i+U];
                       iplu->imageData[tmp1-(int)(tmp2-1)] = ptr[i+U];
                       iplv->imageData[tmp1-(int)(tmp2)] = ptr[i+V];
                       iplv->imageData[tmp1-(int)(tmp2-1)] = ptr[i+V];
		}
#else
        	for(i=0,j=0;j<(IMAGE_WIDTH*IMAGE_HEIGHT*2) ; i+=2,j+=4) {
                	image.data     [i  ] = ptr[j+Y1];
	                image.data     [i+1] = ptr[j+Y2];

		        iply->imageData[i  ] = ptr[j+Y1];
                        iply->imageData[i+1] = ptr[j+Y2];
			/* U channel */
		        iplu->imageData[i  ] = ptr[j+U];
                        iplu->imageData[i+1] = ptr[j+U];
                        /* V channel */
		        iplv->imageData[i  ] = ptr[j+V];
                        iplv->imageData[i+1] = ptr[j+V];
		}
#endif
		find_circles(iply);

#ifdef DEBUG
                double hScale=0.7;
                double vScale=0.7;
                int    lineWidth=1;
                CvFont font;
                cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, hScale,vScale,0,lineWidth,0);
#endif
                for (i=0; circles_st[i].x_in_picture != -1; i++) {
#ifdef DEBUG
                    //char buf[256];
#endif
                    tyros_camera::Object o;

                    o.vector.x=circles_st[i].x_in_picture;
                    o.vector.y=circles_st[i].y_in_picture;
                    o.vector.z=i;
                    o.certainity=i;
                    o.type="Ball";
                    msg.object.push_back(o);

#ifdef DEBUG
#if 0
                    printf("Circle[%d] x:%d y:%d r:%d\n", i, circles_st[i].x_in_picture, circles_st[i].y_in_picture, circles_st[i].r_in_picture);
                    printf("Circle[%d] real: x:%lf y:%lf r:%lf\n", i, circles_st[i].x_from_robot, circles_st[i].y_from_robot, circles_st[i].r_from_robot);
//                  cvCircle(iply[devnum], cvPoint(cvRound(circles[i].x_in_picture), cvRound(circles[i].y_in_picture)), 3, CV_RGB(0,255,0), -1, 8, 0);
                    cvCircle(iply, cvPoint(cvRound(circles_st[i].x_in_picture), cvRound(circles_st[i].y_in_picture)), cvRound(circles_st[i].r_in_picture), CV_RGB(0,255,0), 3, 8, 0);

                    sprintf(buf, "%d x:%d y:%d r:%d", i, circles_st[i].x_in_picture, circles_st[i].y_in_picture, circles_st[i].r_in_picture);
                    cvPutText (iply,buf,cvPoint(circles_st[i].x_in_picture-110,circles_st[i].y_in_picture+circles_st[i].r_in_picture*2), &font, cvScalarAll(255));
                    sprintf(buf, "%d x: %.1lf y: %.1lf r: %.1lf", i, circles_st[i].x_from_robot, circles_st[i].y_from_robot, circles_st[i].r_from_robot);
                    cvPutText (iply,buf,cvPoint(circles_st[i].x_in_picture-110,circles_st[i].y_in_picture+circles_st[i].r_in_picture*2+20), &font, cvScalarAll(255));
#endif 
#endif
                }

                pub.publish(msg);
           //     msg.object.clear();
#ifdef DEBUG
	        if( drawing_box ) {
			draw_box( iply, box );
		}
                cvShowImage( "result Y", iply );
                cvShowImage( "result U", iplu );
                cvShowImage( "result V", iplv );
#endif
//		cam_info = cinfo.getCameraInfo();
  	        cam_info.header.frame_id = image.header.frame_id;
		cam_info.header.stamp = image.header.stamp;

		image_pub.publish(image, cam_info);

		ros::spinOnce();
	}
#ifdef DEBUG
        cvReleaseImage(&iply);
        cvReleaseImage(&iplu);
        cvReleaseImage(&iplv);
#endif
	return 0;
}
