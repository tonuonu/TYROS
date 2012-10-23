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

#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <strings.h>

#include <vector>

#define BAUDRATE B115200
#define MODEMDEVICE "/dev/ttyACM0"

using namespace std;

char START_AP[] =      { 0xFF, 0x07, 0x03 };
char ASK_ACC_DATA[] =  { 0xFF, 0x08, 0x07, 0x00, 0x00, 0x00, 0x00 };

int main(int argc, char** argv)
{
 
    int             fd,
                    res;
    struct termios  newtio;
    char            buf[255];

    fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY);
    if (fd < 0) {
	perror(MODEMDEVICE);
	exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /*
     * set input mode (non-canonical, no echo,...) 
     */
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;	/* inter-character timer unused */
    newtio.c_cc[VMIN] = 3;	/* blocking read until 3 chars received */

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    if(write(fd, START_AP, sizeof(START_AP))==-1) {
	printf("write() failed\n");
	exit(-1);
    }

    res = read(fd, buf, 3);	
    if(res==-1) {
        printf("read() failed\n");
        exit(-1);
    }
    newtio.c_cc[VMIN] = 7;	/* blocking read until 7 chars received */
    tcsetattr(fd, TCSANOW, &newtio);

    ros::init(argc, argv, "TIChronos_teleop");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Publisher pub;
    sensor_msgs::Joy msg;

    pub = n.advertise<sensor_msgs::Joy>("joy", 1); /* Message queue length is just 1 */

    printf("Now it is time to set TI Chronos into ACC mode and press DOWN\n");

    while (ros::ok()) {
	if(write(fd, ASK_ACC_DATA, sizeof(ASK_ACC_DATA))==-1) {
	    printf("write() failed\n");
	    exit(-1);
	}
	res = read(fd, buf, 7);	
	msg.header.stamp=ros::Time::now();
        if(res==-1) {
            printf("read() failed\n");
            exit(-1);
        }
	if(buf[3]==1) {
            // printf("%d %d %d %d x:%d y:%d z:%d\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
            msg.axes.push_back(buf[5]*-1.0/127.0);
            msg.axes.push_back(buf[4]*-1.0/127.0);
            msg.axes.push_back(buf[6]*-1.0/127.0);
	    pub.publish(msg);
	    msg.axes.clear();
	}
	ros::spinOnce();
	loop_rate.sleep();
    }

    return(0);
}


