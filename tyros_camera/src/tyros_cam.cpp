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

#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/legacy/compat.hpp"
#include "opencv2/objdetect/objdetect.hpp"

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

	IplImage* imgy,*imgu,*imgv;
        imgy= cvCreateImage(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT), 8, 1);
        imgu= cvCreateImage(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT), 8, 1);
        imgv= cvCreateImage(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT), 8, 1);
        IplImage* planes[] = { imgy, imgu};

#ifdef DEBUG
	cvNamedWindow( "result Y", 0 );
	cvNamedWindow( "result U", 0 );
	cvNamedWindow( "result V", 0 );
        cvStartWindowThread();
	cvSetMouseCallback("result Y",my_mouse_callback,(void*) imgy);
#endif
	while (ros::ok()) {
		unsigned char* ptr = cam.Update();
		int i,j;
	
		image.header.stamp = ros::Time::now();
		int image_size = IMAGE_HEIGHT*IMAGE_WIDTH;
		image.step = IMAGE_WIDTH;
		image.data.resize(image_size);
                tyros_camera::Objects msg;

		// For each input pixel we do have 2 bytes of data. But we read them in groups of four because of YUYV format
        	for(i=0,j=0;i<(IMAGE_WIDTH*IMAGE_HEIGHT*2) ; i+=2,j+=1) {
                	int tmp1=((j%IMAGE_HEIGHT)*IMAGE_WIDTH+IMAGE_WIDTH-1);
                	image.data[
				tmp1-(int)((j%(IMAGE_WIDTH*IMAGE_HEIGHT))/IMAGE_HEIGHT)
                        	] = ptr[i+Y1];
	                image.data[
				tmp1-(int)((j%(IMAGE_WIDTH*IMAGE_HEIGHT))/IMAGE_HEIGHT-1)
                        	] = ptr[i+Y2];
#if 1
		        imgy->imageData[
                                tmp1-(int)((j%(IMAGE_WIDTH*IMAGE_HEIGHT))/IMAGE_HEIGHT)
                                ] = ptr[i+Y1];
                        imgy->imageData[
                                tmp1-(int)((j%(IMAGE_WIDTH*IMAGE_HEIGHT))/IMAGE_HEIGHT-1)
                                ] = ptr[i+Y2];
#endif
#if 1
		        imgu->imageData[
                                tmp1-(int)((j%(IMAGE_WIDTH*IMAGE_HEIGHT))/IMAGE_HEIGHT)
                                ] = ptr[i+U];
                        imgu->imageData[
                                tmp1-(int)((j%(IMAGE_WIDTH*IMAGE_HEIGHT))/IMAGE_HEIGHT-1)
                                ] = ptr[i+U];
#endif
#if 1
		        imgv->imageData[
                                tmp1-(int)((j%(IMAGE_WIDTH*IMAGE_HEIGHT))/IMAGE_HEIGHT)
                                ] = ptr[i+V];
                        imgv->imageData[
                                tmp1-(int)((j%(IMAGE_WIDTH*IMAGE_HEIGHT))/IMAGE_HEIGHT-1)
                                ] = ptr[i+V];
#endif


        	}
		find_circles(imgy);
	//	draw_circles(imgy);


        // Build the histogram and compute its contents.
        //
        int h_bins = 30, s_bins = 32; 
        CvHistogram* hist;
        {
          int    hist_size[] = { h_bins, s_bins };
          float  h_ranges[]  = { 0, 180 };          // hue is [0,180]
          float  s_ranges[]  = { 0, 255 }; 
          float* ranges[]    = { h_ranges, s_ranges };
          hist = cvCreateHist( 
            2, 
            hist_size, 
            CV_HIST_ARRAY, 
            ranges, 
            1 
          ); 
        }
	if(setroi) {
		cvSetImageROI( imgy, cvRect (box.x,box.y,box.width,box.height));
		cvSetImageROI( imgu, cvRect (box.x,box.y,box.width,box.height));
	}
        cvCalcHist( planes, hist, 0, 0 );

        FILE *f=fopen("histogram.raw","w");
        if(f==NULL) {
            printf("error: %s\n",strerror(errno));
	    exit(1);
        }
        if(fwrite(hist,sizeof(CvHistogram),1,f) != 1) {
            printf("error: %s\n",strerror(errno));
	    exit(1);
        }
        fclose(f);

	if(setroi) {
		cvResetImageROI( imgy);
		cvResetImageROI( imgu);
	}

        // Create an image to use to visualize our histogram.
        //
        int scale = 10;
        IplImage* hist_img = cvCreateImage(  
          cvSize( h_bins * scale, s_bins * scale ), 
          8, 
          3
        ); 
        cvZero( hist_img );

        // populate our visualization with little gray squares.
        //
        float max_value = 0;
        cvGetMinMaxHistValue( hist, 0, &max_value, 0, 0 );

        for( int h = 0; h < h_bins; h++ ) {
            for( int s = 0; s < s_bins; s++ ) {
                float bin_val = cvQueryHistValue_2D( hist, h, s );
                int intensity = cvRound( bin_val * 255 / max_value );
                cvRectangle( 
                  hist_img, 
                  cvPoint( h*scale, s*scale ),
                  cvPoint( (h+1)*scale - 1, (s+1)*scale - 1),
                  CV_RGB(intensity,intensity,intensity), 
                  CV_FILLED
                );
            }
        }

//        cvNamedWindow( "Source", 1 );
//        cvShowImage(   "Source", src );

        cvNamedWindow( "H-S Histogram", 1 );
        cvShowImage(   "H-S Histogram", hist_img );


//0 Correlation, 1 ChiSqr, 2 Intersect, 3 Bhattacharyya
        int type=1;
/*
#define patchx 61
#define patchy 61
                        int iwidth = imgy->width - patchx + 1;
                        int iheight = imgy->height - patchy + 1;

            IplImage *ftmp = cvCreateImage( cvSize(iwidth,iheight),32,1);
*/
 //           printf("Doing cvCalcBackProjectPatch() with type =%d\n",type);
 //           cvCalcBackProjectPatch(planes[1],ftmp,cvSize(61,61),hist,type,1.0);
 //           printf("ftmp count = %d\n",cvCountNonZero(ftmp));
 IplImage * dst = cvCreateImage( cvGetSize(imgy),8,1);

            cvCalcBackProject(planes,dst,hist);

        cvNamedWindow( "back", 1 );
        cvShowImage(   "back", dst);

#ifdef DEBUG
                double hScale=0.7;
                double vScale=0.7;
                int    lineWidth=1;
                CvFont font;
                cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, hScale,vScale,0,lineWidth,0);
#endif
                for (i=0; circles_st[i].x_in_picture != -1; i++) {
#ifdef DEBUG
                    char buf[256];
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
//                  cvCircle(imgy[devnum], cvPoint(cvRound(circles[i].x_in_picture), cvRound(circles[i].y_in_picture)), 3, CV_RGB(0,255,0), -1, 8, 0);
                    cvCircle(imgy, cvPoint(cvRound(circles_st[i].x_in_picture), cvRound(circles_st[i].y_in_picture)), cvRound(circles_st[i].r_in_picture), CV_RGB(0,255,0), 3, 8, 0);

                    sprintf(buf, "%d x:%d y:%d r:%d", i, circles_st[i].x_in_picture, circles_st[i].y_in_picture, circles_st[i].r_in_picture);
                    cvPutText (imgy,buf,cvPoint(circles_st[i].x_in_picture-110,circles_st[i].y_in_picture+circles_st[i].r_in_picture*2), &font, cvScalarAll(255));
                    sprintf(buf, "%d x: %.1lf y: %.1lf r: %.1lf", i, circles_st[i].x_from_robot, circles_st[i].y_from_robot, circles_st[i].r_from_robot);
                    cvPutText (imgy,buf,cvPoint(circles_st[i].x_in_picture-110,circles_st[i].y_in_picture+circles_st[i].r_in_picture*2+20), &font, cvScalarAll(255));
#endif 
#endif
                }

                pub.publish(msg);
           //     msg.object.clear();
#ifdef DEBUG
	        if( drawing_box ) {
			draw_box( imgy, box );
		}
                cvShowImage( "result Y", imgy );
                cvShowImage( "result U", imgu );
                cvShowImage( "result V", imgv );
#endif
//		cam_info = cinfo.getCameraInfo();
  	        cam_info.header.frame_id = image.header.frame_id;
		cam_info.header.stamp = image.header.stamp;

		image_pub.publish(image, cam_info);

		ros::spinOnce();
	}
#ifdef DEBUG
        cvReleaseImage(&imgy);
        cvReleaseImage(&imgu);
        cvReleaseImage(&imgv);
#endif
	return 0;
}
