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


/* heder files goes here */
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <list>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include <nav_msgs/Odometry.h>
#include <fstream>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"
#include <visualization_msgs/Marker.h>

#define TEXTLEN 64

using namespace std;

class Fusion_Xtion_Laser{
 private:
  struct POINT;
  ros::NodeHandle nh_;

  ros::Subscriber scan_xtion_sub_;
  ros::Subscriber scan_laser_sub_;
  ros::Subscriber tf_pose_;
  ros::Subscriber laser_parameters_sub_;
 
  ros::Publisher  laser_map_changes_pub_;
  ros::Publisher  xtion_map_changes_pub_;
  ros::Publisher  xtion_laser_map_changes_pub_;
  ros::Publisher vis_pub_;
  ros::Publisher xtion_laser_scan_pub_; 

struct MAP{    
  double** cost_;    
  double height_;
  double width_;
  double resolution_;   
};

  MAP* laser_map_;
  MAP* xtion_map_;
  MAP* xtion_laser_map_;
  MAP* temp_laser_map_;
  MAP* temp_xtion_map_;

  double tfX_;
  double tfY_;
  double tfZ_;

  double length;
  double inc;
  double range_min;
  double range_max;
  double angle_min;
  double angle_max;
  double time_inc;

  int dataxl_;
  int datayl_;
  int dataxx_;
  int datayx_;
  double map_y_max_;    
  double map_x_max_;    
  double resolution_;   
  double map_resolution_;
  double X_OR_,Y_OR_;

  int GR_SIZE_;
  double height_; 
  double width_;  
  double prior_;  
  double emp;
  double occ; 
  double koef;
  int xx,yy,x0,y0, step;
  int laser_count, xtion_count, xtion_laser_count;
  double origin_map_x_,origin_map_y_;
  int max_gray;
  double value_;
  char outputFilename[TEXTLEN];

public:
  Fusion_Xtion_Laser();
  ~Fusion_Xtion_Laser();

  void Scan_Xtion_Callback(const sensor_msgs::PointCloud::ConstPtr& msg);
  void Scan_Laser_Callback(const sensor_msgs::PointCloud::ConstPtr& msg);
  void Laser_Parameters(const sensor_msgs::LaserScan::ConstPtr& msg);
  void Load_Global_Parameters(void);
  void tf_Pose(nav_msgs::Odometry msg);
  MAP* make_map(double height, double width, double resolution, double initial);
  void destroy_map(MAP* map);
  void Laser_Fusion(int c_x_, int c_y_, int dataxl_, int datayl_, int GR_SIZE);
  void Xtion_Fusion(int c_x_, int c_y_, int dataxl_, int datayl_, int GR_SIZE);
  void xtion_laser_fused(void);
  void publish_laser_map_changes();
  void publish_xtion_map_changes();
  void publish_xtion_laser_map_changes();
  void xtion_write();
  void laser_write();
  void run();

};

