/*
 *  Copyright (c) 2011, Tonu Samuel
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
#include "std_msgs/String.h"

#include <sstream>


#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

int fd=-1; 

double angular=0.0, linear=0.0;

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
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
	    ROS_ERROR("open_port: Unable to open /dev/ttyUSB0");
	} else {
	    ROS_INFO("open_port: opened /dev/ttyUSB0");
	    fcntl(fd, F_SETFL, 0);
            cfsetospeed(&tio,B115200);            // 115200 baud
            cfsetispeed(&tio,B115200);            // 115200 baud
	}
   //     fcntl(fd,F_SETFL,FNDELAY); // Make read() call nonblocking
	return (fd);
}


const char *keywords[]={
	"Acceleration",
	"Gyroscope",
	"Left motor",
	"Right motor",
	"Battery",
	"Capacitor",
	"Ball",
	0
};

enum kwnum {
	KW_ACCELERATION=0,
	KW_GYROSCOPE=1,
	KW_LEFTMOTOR=2,
	KW_RIGHTMOTOR=3,
	KW_BATTERY=4,
	KW_CAPACITOR=5,
	KW_BALL=6,
};

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

  //  ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    if(0 > fd) {
        ROS_WARN("/dev/ttyUSB0 is not yet open. Trying to fix this...");
        open_port();
    } 
    int len=0;
    if(0 > fd)  {
        ROS_ERROR("/dev/ttyUSB0 is still not open?! Now I give up!");
    } else {
	char buf[256];
	int nbytes=read(fd,buf,sizeof(buf));
	char *p=strchr(buf,27);
	int offset=p-buf;
        if(offset<200 && p[1]=='[' && p[4]==';' && p[6]=='H') {
            char *colonptr=strchr(&p[7],':');
	    if(colonptr>0) {
	        *colonptr=0;
	        //printf("%s\n",&p[7]);
		const char **kw=keywords;
		int kwnum=0;
		while(*kw) {
		    if(strcmp(*kw,&p[7])==0) {
			    printf("->Matched '%s'\n",*kw);

                            float p,x,y,z;
                            int n;

                            errno = 0;
			    switch(kwnum) {
			        case KW_ACCELERATION:
                                    n = sscanf(&colonptr[1]," %f ", &p);
	                            break;
			        case KW_GYROSCOPE:
                                    n = sscanf(&colonptr[1]," x: %f y: %f z: %f", &x ,&y, &z);
	                            break;
			        case KW_BATTERY:
                                    n = sscanf(&colonptr[1]," %f V", &p);
	                            break;
			        case KW_RIGHTMOTOR:
                                    n = sscanf(&colonptr[1]," %f ", &p);
	                            break;
			        case KW_LEFTMOTOR:
                                    n = sscanf(&colonptr[1]," %f ", &p);
	                            break;
			        case KW_CAPACITOR:
                                    n = sscanf(&colonptr[1]," %f ", &p);
	                            break;
                                default:
				    printf("ERROR: unhandled kwnum %d\n",kwnum);
				    exit(0);
			    }
                            if (n == 1) {
                                printf("read: %f\n", p);
                            } else if (n == 3) {
                                printf("read: %f %f %f\n", x,y,z);
                            } else if (errno != 0) {
                                perror("scanf");
                            } else {
                                printf( "(%d) No matching characters in '%c'\n",n,colonptr[2]);
                            }

		    }
	            kw++;
		    kwnum++;
		}
	    }

	}
    }
 //   loop_rate.sleep();
    ++count;
  }

  return 0;
}
