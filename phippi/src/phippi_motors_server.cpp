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
#include "geometry_msgs/Twist.h"

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

int
open_port(void){
	int fd; 

	fd = open("/dev/ttyO2", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
	    ROS_INFO("open_port: Unable to open /dev/ttyO2");
	} else {
	    ROS_INFO("open_port: opened /dev/ttyO2");
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

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->angular.x);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "phippi_motors");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/cmd_vel", 10, chatterCallback);

  ros::ServiceServer service = n.advertiseService("phippi_motors", add);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(20.0);

  while(n.ok()){
 
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
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

    last_time = current_time;
    r.sleep();
  }


  return 0;
}

