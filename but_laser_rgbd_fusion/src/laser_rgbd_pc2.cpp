/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Alfredo Chavez Plascencia(plascencia@fit.vutbr.cz)
 * 
 * 
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <laser_rgbd_pc2.h>

using namespace std;

Comb_FakeXtion_Laser::Comb_FakeXtion_Laser() : nh_("~"){
   ROS_INFO("Entering run.");
   run();
}


void  Comb_FakeXtion_Laser::run(){
 Load_Global_Parameters();
 scan_xtion_sub_ = nh_.subscribe<sensor_msgs::LaserScan> ("/scan_xtion", 100, &Comb_FakeXtion_Laser::Scan_Xtion_Callback, this);  
 scan_laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_filter", 100, &Comb_FakeXtion_Laser::Scan_Laser_Callback, this);
 odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odometer_pose_pb", 100, &Comb_FakeXtion_Laser::Odom_Callback, this);
 laser_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/laser_coordinates", 2);
 xtion_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/xtion_coordinates", 2);
 PC2_laser_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/PC2_laser_coordinates", 2);
 PC2_xtion_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/PC2_xtion_coordinates", 2);
 
 ros::spin();
}

Comb_FakeXtion_Laser::~Comb_FakeXtion_Laser(){
 scan_xtion_sub_.shutdown();
 scan_laser_sub_.shutdown();
 laser_scan_pub_.shutdown();
 xtion_scan_pub_.shutdown();
 PC2_laser_scan_pub_.shutdown();
 PC2_xtion_scan_pub_.shutdown();
 odom_sub_.shutdown();
 free (robot_pose_);
}

void Comb_FakeXtion_Laser::Load_Global_Parameters(void){
 robot_pose_=NULL;
 robot_pose_receive_=false;
 nh_.param("R2D",R2D_,57.21);
 success = false;
}

// this creates and returns a pointer to a POSE struct
Comb_FakeXtion_Laser::POSE *Comb_FakeXtion_Laser::make_pose(double x, double y, double z){

  POSE* pose = (POSE*)calloc(1, sizeof(POSE));
  if(pose==NULL) 
    exit(1);
  pose->x = 0;
  pose->y = 0;
  pose->z = 0;
  return pose;
}


//Callback to get the position of the robot
void Comb_FakeXtion_Laser::Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg){
 if(robot_pose_ == NULL)
  robot_pose_ = make_pose(0.0,0.0,0.0);
  robot_pose_->x =  msg->pose.pose.position.x;
  robot_pose_->y =  msg->pose.pose.position.y;
  robot_pose_->z =  msg->pose.pose.orientation.w;
  robot_pose_receive_=true;
}


//converts the rgbd data readings to pc and pc2.
void Comb_FakeXtion_Laser::Scan_Xtion_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
 sensor_msgs::PointCloud2 PC2_xtion_msg;
 PC2_xtion_msg.header.frame_id = msg->header.frame_id;
 PC2_xtion_msg.header.stamp = ros::Time(0); 

 if(!xtion_listener_.waitForTransform(
        msg->header.frame_id,
        "/laser",
        msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),
        ros::Duration(1.0))){
  
     return;
  }


  sensor_msgs::PointCloud xtion_msg;
  xtion_projector_.transformLaserScanToPointCloud("/laser",*msg,xtion_msg,xtion_listener_);

 xtion_scan_pub_.publish(xtion_msg);
 sensor_msgs::convertPointCloudToPointCloud2(xtion_msg,PC2_xtion_msg);
  PC2_xtion_scan_pub_.publish(PC2_xtion_msg);
}


//converts the laser data readings to pc and pc2.
void Comb_FakeXtion_Laser::Scan_Laser_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
 sensor_msgs::PointCloud laser_msg;
 laser_msg.header.frame_id = msg->header.frame_id;
 laser_msg.header.stamp = msg->header.stamp;

 sensor_msgs::PointCloud2 PC2_laser_msg;
 PC2_laser_msg.header.frame_id = msg->header.frame_id;
 PC2_laser_msg.header.stamp = msg->header.stamp;

  try {
    laser_listener_.waitForTransform(
        msg->header.frame_id,"/laser", msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),ros::Duration(1.0));
   
  } catch (tf::ExtrapolationException e) {

}
  
 laser_projector_.transformLaserScanToPointCloud("/laser",*msg,
           laser_msg,laser_listener_);

 laser_scan_pub_.publish(laser_msg);
 int laser_length = laser_msg.points.size();

 sensor_msgs::convertPointCloudToPointCloud2(laser_msg,PC2_laser_msg);
 PC2_laser_scan_pub_.publish(PC2_laser_msg);
 
}




int main(int argc, char** argv){
  ros::init(argc, argv, "fakextion_laser");
   Comb_FakeXtion_Laser fake_xtionlaser;
   

  return (0);
}
