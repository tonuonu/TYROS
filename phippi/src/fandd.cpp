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

#include "ros/ros.h"
#include "phippi/TwoInts.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "phippi/Velocity.h"

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

int fd=-1; 

int
open_port(void){
        struct termios tio;
        memset(&tio,0,sizeof(tio));
        tio.c_iflag=0;
        tio.c_oflag=0;
        tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
        tio.c_lflag=0;
        tio.c_cc[VMIN]=1;
        tio.c_cc[VTIME]=5;
	fd = open("/dev/ttyO2", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
	    ROS_ERROR("open_port: Unable to open /dev/ttyO2");
	} else {
	    ROS_INFO("open_port: opened /dev/ttyO2");
	    fcntl(fd, F_SETFL, 0);

            cfsetospeed(&tio,B115200);            // 115200 baud
            cfsetispeed(&tio,B115200);            // 115200 baud
	}
	return (fd);
}

void chatterCallback(const phippi::Velocity::ConstPtr& msg)
{
    char buf[256];
    int len,r;
    ROS_INFO("we got: [%.1f %.1f]", msg->angular,msg->linear);
    if(0 > fd) {
        ROS_WARN("/dev/ttyO2 is not yet open. Trying to fix this...");
        open_port();
    } 
    len=0;
    if(0 > fd)  {
        ROS_ERROR("/dev/ttyO2 is still not open?! Now I give up!");
    } else if(msg->angular > 0) {
	snprintf(buf,256,"pwm -50 500%c",0x0d);
        len=strlen(buf);
    } else if(msg->angular < 0) {
	snprintf(buf,256,"pwm 50 -50%c",0x0d);
        len=strlen(buf);
    } else if(msg->linear > 0) {
	snprintf(buf,256,"pwm 50 50%c",0x0d);
        len=strlen(buf);
    } else if(msg->linear < 0) {
	snprintf(buf,256,"pwm -50 -50%c",0x0d);
        len=strlen(buf);
    } else {
	snprintf(buf,256,"pwm 0 0%c",0x0d);
        len=strlen(buf);
    }

    if(len>0) {
        if ((r=write(fd,buf,len))==len) {
       	    ROS_INFO("successfully wrote: '%s'",buf);
        } else {
       	    ROS_INFO("failed to write: '%s' (wrote %d of %d)",buf,r,len);
        }
    }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "phippi_motors");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/turtle1/command_velocity", 1, chatterCallback);
double angular=0,linear=0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10.0);
  while(n.ok()){
    if(angular>0) angular--;
    if(angular<0) angular++;
    if(linear>0) linear--;
    if(linear<0) linear++;
    if(!linear && !angular) {
        snprintf(buf,256,"pwm 0 0\n");
        len=strlen(buf);
    }
    current_time = ros::Time::now();

     ros::spinOnce();
 //   double dt = (current_time - last_time).toSec();

    last_time = current_time;
    r.sleep();
  }
  return 0;
}

