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


#include <iostream>
#include <math.h>
#include <vector>
#include <list>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"


//#ifndef PI
  #define PI 3.1415926535897
//#endif


class Comb_FakeXtion_Laser{
 private:
  struct POSE{
    double x;
    double y;
    double z;
  };

  POSE* robot_pose_;
  ros::NodeHandle nh_;

  ros::Subscriber scan_xtion_sub_;
  ros::Subscriber scan_laser_sub_;
  ros::Subscriber odom_sub_;

  ros::Publisher laser_scan_pub_;
  ros::Publisher xtion_scan_pub_;
  ros::Publisher PC2_laser_scan_pub_;
  ros::Publisher PC2_xtion_scan_pub_;

  double R2D_;
  bool success;
  bool robot_pose_receive_;
  bool scan_xtion,scan_fake;

  int length_x,length_f,length_xtion,length_fake,total_length;
  double temp_xtion_ranges[], temp_fake_ranges[];
  
 
 public:
  Comb_FakeXtion_Laser();
  ~Comb_FakeXtion_Laser();
  void Scan_Xtion_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void Scan_Laser_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void Load_Global_Parameters(void);
  POSE* make_pose(double x, double y, double z);
  
  laser_geometry::LaserProjection xtion_projector_;
  laser_geometry::LaserProjection laser_projector_;
  tf::TransformListener xtion_listener_;
  tf::TransformListener laser_listener_;
  void run();
    
};
