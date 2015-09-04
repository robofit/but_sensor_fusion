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
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"

class Filter_Hokuyo{
 private:
  ros::NodeHandle nh_;
  ros::Subscriber laser_scan_sub_;
  ros::Publisher laser_scan_pub_;

  double temp_laser_ranges[];
  double minimum_range_;
  double maximum_range_;
  double value_;

 public:
  Filter_Hokuyo();
  ~Filter_Hokuyo();
  void Scan_Hokuyo_callback(const sensor_msgs::LaserScan::ConstPtr& msg); 
  void Load_Global_Parameters(void);
  void run();
};
