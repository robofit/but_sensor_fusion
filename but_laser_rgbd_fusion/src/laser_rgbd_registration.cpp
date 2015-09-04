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

#include <laser_rgbd_registration.h>

 using namespace sensor_msgs;
 using namespace message_filters;


 ros::Publisher pub;
 ros::Publisher Cloud_SYN_laser_scan_pub_;
 ros::Publisher Cloud_SYN_xtion_scan_pub_;
 ros::Publisher SYN_xtion_scan_pub_;
 ros::Publisher SYN_laser_scan_pub_;


//Callback function that makes sensor data readings alignment.
void callback(const sensor_msgs::PointCloud2ConstPtr& PC2_laser, const sensor_msgs::PointCloud2ConstPtr& PC2_xtion){

 sensor_msgs::PointCloud2 SYN_laser_msg;
 SYN_laser_msg.header.frame_id = PC2_laser->header.frame_id;
 SYN_laser_msg.header.stamp = PC2_laser->header.stamp;
 SYN_laser_scan_pub_.publish(PC2_laser);

 sensor_msgs::PointCloud2 SYN_xtion_msg;
 SYN_xtion_msg.header.frame_id = PC2_xtion->header.frame_id;
 SYN_xtion_msg.header.stamp = PC2_xtion->header.stamp;
  
 sensor_msgs::PointCloud Cloud_SYN_laser_msg;
 Cloud_SYN_laser_msg.header.frame_id = PC2_laser->header.frame_id;
 Cloud_SYN_laser_msg.header.stamp = PC2_laser->header.stamp;
 sensor_msgs::convertPointCloud2ToPointCloud(*PC2_laser,Cloud_SYN_laser_msg);
 Cloud_SYN_laser_scan_pub_.publish(Cloud_SYN_laser_msg);

 sensor_msgs::PointCloud Cloud_SYN_xtion_msg;
 Cloud_SYN_xtion_msg.header.frame_id = PC2_xtion->header.frame_id;
 Cloud_SYN_xtion_msg.header.stamp = PC2_xtion->header.stamp;

 pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ>);
 pcl::fromROSMsg (*PC2_laser, *laser_cloud);
 
 pcl::PointCloud<pcl::PointXYZ>::Ptr xtion_cloud(new pcl::PointCloud<pcl::PointXYZ>);
 pcl::fromROSMsg (*PC2_xtion, *xtion_cloud);

 pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;


 icp.setMaxCorrespondenceDistance (0.35);
 icp.setMaximumIterations (100);
 icp.setRANSACIterations (30); 	 
 icp.setTransformationEpsilon (1e-14);
 icp.setEuclideanFitnessEpsilon (3);
 icp.setInputSource(xtion_cloud);
 icp.setInputTarget(laser_cloud);
 pcl::PointCloud<pcl::PointXYZ> Final;
 icp.align(Final);
  
 pcl::toROSMsg(Final, SYN_xtion_msg);
 SYN_xtion_scan_pub_.publish(SYN_xtion_msg);
 
 sensor_msgs::convertPointCloud2ToPointCloud(SYN_xtion_msg,Cloud_SYN_xtion_msg);
 Cloud_SYN_xtion_scan_pub_.publish(Cloud_SYN_xtion_msg);
  
}

int main (int argc, char** argv){
  
 ros::init (argc, argv, "laser_rgbd_registration");
 ros::NodeHandle nh;

 message_filters::Subscriber<sensor_msgs::PointCloud2> laser_sub(nh, "PC2_laser_coordinates", 10);
 message_filters::Subscriber<sensor_msgs::PointCloud2> xtion_sub(nh,  "PC2_xtion_coordinates", 10);

 typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> MySyncPolicy;
 Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),  laser_sub, xtion_sub);
 sync.registerCallback(boost::bind(&callback, _1, _2));

 Cloud_SYN_laser_scan_pub_ = nh.advertise<sensor_msgs::PointCloud>("/Cloud_SYN_laser_msg", 2);
 Cloud_SYN_xtion_scan_pub_ = nh.advertise<sensor_msgs::PointCloud>("/Cloud_SYN_xtion_msg", 2);
 SYN_xtion_scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/SYN_xtion_msg", 2);
 SYN_laser_scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/SYN_laser_msg", 2);
  
    ros::Rate loop_rate(10);
       while(ros::ok()){	
          ros::spinOnce();
	  loop_rate.sleep();
      }

 return 0;
 
 Cloud_SYN_laser_scan_pub_.shutdown();
 Cloud_SYN_xtion_scan_pub_.shutdown();
 SYN_xtion_scan_pub_.shutdown();
 SYN_laser_scan_pub_.shutdown();
 
}
