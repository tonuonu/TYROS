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
	fd = open("/dev/ttyO2", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
	    ROS_ERROR("open_port: Unable to open /dev/ttyO2");
	} else {
	    ROS_INFO("open_port: opened /dev/ttyO2");
	    fcntl(fd, F_SETFL, 0);
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
	snprintf(buf,256,"pwm -50 50\n");
        len=strlen(buf);
    } else if(msg->angular < 0) {
	snprintf(buf,256,"pwm 50 -50\n");
        len=strlen(buf);
    } else if(msg->linear > 0) {
	snprintf(buf,256,"pwm 50 50\n");
        len=strlen(buf);
    } else if(msg->linear < 0) {
	snprintf(buf,256,"pwm -50 -50\n");
        len=strlen(buf);
    } else {
	snprintf(buf,256,"pwm 0 0\n");
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
#if 0
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;
#endif
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10.0);
//  ros::spin();
  while(n.ok()){
   if(angular>0) angular--;
   if(angular<0) angular++;
   if(linear>0) linear--;
   if(linear<0) linear++;
    current_time = ros::Time::now();

  ros::spinOnce();
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
#if 0
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "/odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "/odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);
 //   ros::spin();


#endif
    last_time = current_time;
    r.sleep();
  }
  return 0;
}

