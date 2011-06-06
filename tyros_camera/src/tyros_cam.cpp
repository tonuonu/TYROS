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

int main(int argc, char **argv) {
	int height, width, input;
	std::string dev, frame_id, cinfo_url;
	sensor_msgs::Image image;
	sensor_msgs::CameraInfo cam_info;

	ros::init(argc, argv, "tyros_cam");
	ros::NodeHandle nh("~");

	CameraInfoManager cinfo(nh);

	image_transport::ImageTransport it(nh);
	image_transport::CameraPublisher image_pub = it.advertiseCamera("image_raw", 1);

	nh.param<int>("width", width, 240);
	nh.param<int>("height", height, 320);
	nh.param<int>("input", input, 0);
	nh.param<std::string>("device", dev, "/dev/video0");
	nh.param<std::string>("frame_id", frame_id, "camera");
	nh.param<std::string>("camera_info_url", cinfo_url, "file:///home/panda/TYROS/tyros_camera/calib.yaml");
	cinfo.loadCameraInfo(cinfo_url);

	ROS_INFO("Opening device : %s", dev.c_str());
	Camera cam(dev.c_str(), 240, 320);

	cam.setInput(input);

	image.header.frame_id = frame_id;
	image.height = 320;
	image.width = 240;
	image.encoding = sensor_msgs::image_encodings::MONO8;

        ros::init(argc, argv, "TIChronos_teleop");
        ros::NodeHandle n;
        ros::Rate loop_rate(60);
        ros::Publisher pub;

        pub = n.advertise<tyros_camera::Objects>("vision_publisher", 5); /* Message queue length is 3 */

	IplImage* imgy;
        imgy= cvCreateImage(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT), 8, 1);
#ifdef DEBUG
	cvNamedWindow( "result", 1 );
        cvStartWindowThread();
#endif
	while (ros::ok()) {
		unsigned char* ptr = cam.Update();
		int i,j;
	
		image.header.stamp = ros::Time::now();
		int image_size = 320*240;
		image.step = 240;
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
		        imgy->imageData[
                                tmp1-(int)((j%(IMAGE_WIDTH*IMAGE_HEIGHT))/IMAGE_HEIGHT)
                                ] = ptr[i+Y1];
                        imgy->imageData[
                                tmp1-(int)((j%(IMAGE_WIDTH*IMAGE_HEIGHT))/IMAGE_HEIGHT-1)
                                ] = ptr[i+Y2];
        	}
		find_circles(imgy);
//		draw_circles(imgy)

#ifdef DEBUG
                double hScale=0.7;
                double vScale=0.7;
                int    lineWidth=1;
                CvFont font;
                cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, hScale,vScale,0,lineWidth,0);
#endif
                for (i=0; circles[i].x_in_picture != -1; i++) {
#ifdef DEBUG
                    char buf[256];
#endif
                    tyros_camera::Object o;

                    o.vector.x=circles[i].x_in_picture;
                    o.vector.y=circles[i].y_in_picture;
                    o.vector.z=i;
                    o.certainity=i;
                    o.type="Ball";
                    msg.object.push_back(o);


#ifdef DEBUG
                    printf("Circle[%d] x:%d y:%d r:%d\n", i, circles[i].x_in_picture, circles[i].y_in_picture, circles[i].r_in_picture);
                    printf("Circle[%d] real: x:%lf y:%lf r:%lf\n", i, circles[i].x_from_robot, circles[i].y_from_robot, circles[i].r_from_robot);
//                    cvCircle(imgy[devnum], cvPoint(cvRound(circles[i].x_in_picture), cvRound(circles[i].y_in_picture)), 3, CV_RGB(0,255,0), -1, 8, 0);
                    cvCircle(imgy, cvPoint(cvRound(circles[i].x_in_picture), cvRound(circles[i].y_in_picture)), cvRound(circles[i].r_in_picture), CV_RGB(0,255,0), 3, 8, 0);


                    sprintf(buf, "%d x:%d y:%d r:%d", i, circles[i].x_in_picture, circles[i].y_in_picture, circles[i].r_in_picture);
                    cvPutText (imgy,buf,cvPoint(circles[i].x_in_picture-110,circles[i].y_in_picture+circles[i].r_in_picture*2), &font, cvScalarAll(255));
                    sprintf(buf, "%d x: %.1lf y: %.1lf r: %.1lf", i, circles[i].x_from_robot, circles[i].y_from_robot, circles[i].r_from_robot);
                    cvPutText (imgy,buf,cvPoint(circles[i].x_in_picture-110,circles[i].y_in_picture+circles[i].r_in_picture*2+20), &font, cvScalarAll(255));
#endif
                }


                pub.publish(msg);
           //     msg.object.clear();
#ifdef DEBUG
                cvShowImage( "result", imgy );
#endif
//		cam_info = cinfo.getCameraInfo();
  	        cam_info.header.frame_id = image.header.frame_id;
		cam_info.header.stamp = image.header.stamp;

		image_pub.publish(image, cam_info);

		ros::spinOnce();
	}
#ifdef DEBUG
        cvReleaseImage(&imgy);
#endif
	return 0;
}
