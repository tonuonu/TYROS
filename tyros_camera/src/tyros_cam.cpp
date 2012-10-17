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

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <cvblob.h>
#include "libcam.h"
#include "image.h"

#define DEBUG 1

using namespace cvb;
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
	    if(x<=IMAGE_WIDTH && y<=IMAGE_HEIGHT) {
	        CvScalar pixel=cvGet2D(image,y,x);
	        printf("coord x:%3d y:%3d color Y:%3.0f :U%3.0f V:%3.0f\n",x,y,pixel.val[0],pixel.val[1],pixel.val[2]);
            }
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
        CvBlobs blobs;
        CvTracks tracks_o;
        CvTracks tracks_b;
        CvTracks tracks_y;
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
		nh.param<std::string>("device", dev, "/dev/video3");
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
	cvZero(iply);
	cvZero(iplu);
	cvZero(iplv);
        IplImage* imgOrange = cvCreateImage(cvGetSize(iply), 8, 1);
        IplImage* imgBlue   = cvCreateImage(cvGetSize(iply), 8, 1);
        IplImage* imgYellow = cvCreateImage(cvGetSize(iply), 8, 1);
#ifdef DEBUG
	cvNamedWindow( "result Y", 0 );
	cvNamedWindow( "result U", 0 );
	cvNamedWindow( "result V", 0 );
	cvNamedWindow( "result YUV", 0 );
	cvNamedWindow( "orange", 0 );
	cvNamedWindow( "blue", 0 );
	cvNamedWindow( "yellow", 0 );
        cvStartWindowThread(); 
	IplImage*imgYUV= cvCreateImage(cvGetSize(iply), 8, 3);
	cvSetMouseCallback("result YUV",my_mouse_callback,(void*) imgYUV);
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
               for(i=0,j=0;i<(IMAGE_WIDTH*IMAGE_HEIGHT*2) ; i+=4,j+=2) {
                       int tmp1=((j%IMAGE_HEIGHT)*IMAGE_WIDTH+IMAGE_WIDTH-1);
		       int tmp2=j/IMAGE_HEIGHT;
                       image.data[tmp1-(int)(tmp2)]             = ptr[i+Y1];
                       image.data[tmp1-(int)(tmp2-IMAGE_WIDTH)] = ptr[i+Y2];
                       iply->imageData[tmp1-(int)(tmp2)]   = ptr[i+Y1];
                       iply->imageData[tmp1-(int)(tmp2-IMAGE_WIDTH)] = ptr[i+Y2];
                       iplu->imageData[tmp1-(int)(tmp2)]   = ptr[i+U-4];
                       iplu->imageData[tmp1-(int)(tmp2-IMAGE_WIDTH)] = ptr[i+U];
                       iplv->imageData[tmp1-(int)(tmp2)]   = ptr[i+V-4];
                       iplv->imageData[tmp1-(int)(tmp2-IMAGE_WIDTH)] = ptr[i+V];
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
		//double minVal,maxVal;

		cvMerge(iply,iplu ,iplv , NULL, imgYUV);
                cvInRangeS(imgYUV, cvScalar( 55,  65, 152), cvScalar(203, 109, 199), imgOrange);
                cvInRangeS(imgYUV, cvScalar( 0, 100,  115), cvScalar(40, 133, 128), imgBlue  );
                cvInRangeS(imgYUV, cvScalar(101,  83, 123), cvScalar(155, 114, 142), imgYellow);
                IplImage *labelImg=cvCreateImage(cvGetSize(imgYUV), IPL_DEPTH_LABEL, 1);
#if 0
                unsigned int result;



// Orange
		result=cvLabel(imgOrange, labelImg, blobs);
                cvFilterByArea(blobs, 15, 1000000);
                CvLabel label=cvLargestBlob(blobs);
                if(label!=0) {
			// Delete all blobs except the largest
          		cvFilterByLabel(blobs, label);
			if(blobs.begin()->second->maxy - blobs.begin()->second->miny < 50) { // Cut off too high objects
	        		printf("largest orange blob at %.1f %.1f\n",blobs.begin()->second->centroid.x,blobs.begin()->second->centroid.y);
				//blobs.begin()->second->label="orange";
			}
                 }
                cvRenderBlobs(labelImg, blobs, imgYUV, imgYUV,CV_BLOB_RENDER_BOUNDING_BOX);
                cvUpdateTracks(blobs, tracks_o, 200., 5);
                cvRenderTracks(tracks_o, imgYUV, imgYUV, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);

// Blue
//		result=cvLabel(imgBlue, labelImg, blobs);

//assert(0);
                cvFilterByArea(blobs, 15, 1000000);
                label=cvLargestBlob(blobs);
                if(label!=0) {
			// Delete all blobs except the largest
			cvFilterByLabel(blobs, label);
			if(blobs.begin()->second->maxy - blobs.begin()->second->miny < 50) { // Cut off too high objects
		            printf("largest blue blob at %.1f %.1f\n",blobs.begin()->second->centroid.x,blobs.begin()->second->centroid.y);
			    //blobs.begin()->second->label="orange";
			}
                 }
                cvRenderBlobs(labelImg, blobs, imgYUV, imgYUV,CV_BLOB_RENDER_BOUNDING_BOX);
                cvUpdateTracks(blobs, tracks_b, 200., 5);
                cvRenderTracks(tracks_b, imgYUV, imgYUV, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);




		for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it) {
			//printf("res %d minx %d,miny %d centroid %.1fx%.1f\n",result,it->second->minx,it->second->miny,it->second->centroid.x,it->second->centroid.y);
        	}

		/*cvMinMaxLoc( const CvArr* A, double* minVal, double* maxVal,
                  CvPoint* minLoc, CvPoint* maxLoc, const CvArr* mask=0 ); */
		//find_circles(iply);

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

#endif
                pub.publish(msg);
           //     msg.object.clear();
#ifdef DEBUG
	        if( drawing_box ) {
			draw_box( iply, box );
		}
                cvShowImage( "result Y", iply );
                cvShowImage( "result U", iplu );
                cvShowImage( "result V", iplv );
                cvShowImage( "result YUV", imgYUV );
                cvShowImage( "orange", imgOrange);
                cvShowImage( "blue", imgBlue);
                cvShowImage( "yellow", imgYellow);
		cvWaitKey(10);
#endif
//		cam_info = cinfo.getCameraInfo();
  	        cam_info.header.frame_id = image.header.frame_id;
		cam_info.header.stamp = image.header.stamp;

		image_pub.publish(image, cam_info);

		ros::spinOnce();
		cvReleaseImage(&labelImg);
	}
#ifdef DEBUG
        cvReleaseImage(&iply);
        cvReleaseImage(&iplu);
        cvReleaseImage(&iplv);
#endif
	return 0;
}
