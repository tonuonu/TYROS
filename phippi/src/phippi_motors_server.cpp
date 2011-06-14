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

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

int
open_port(void){
	int fd; 

	fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
	    ROS_INFO("open_port: Unable to open /dev/ttyACM0 ");
	} else {
	    ROS_INFO("open_port: opened /dev/ttyACM0 ");
	    fcntl(fd, F_SETFL, 0);
	}
	return (fd);
}



bool add(phippi::TwoInts::Request  &req,
         phippi::TwoInts::Response &res ) {
    char buf[256];

    ROS_INFO("request: x=%f, y=%f", req.a, req.b);
    if(req.a!=0.0) {
	snprintf(buf,256,"G%f\r\n",req.a);
	ROS_INFO("  GO: [%f]",req.a);
        res.result = "Go done" ;
    } else {
	snprintf(buf,256,"R%f\r\n",req.b);
	ROS_INFO("  ROTATE: [%f]",req.b);
        res.result = "Rotate done" ;
    }
    int fd=open_port();
    int n = write(fd, buf, strlen(buf));
    if (n < 0)
      ROS_INFO("write() failed!" );
    close(fd);

//  ROS_INFO("  sending back response: [%f]",res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "phippi_motors");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("phippi_motors", add);
  ros::spin();

  return 0;
}

