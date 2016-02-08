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

#include <filter_hokuyo.h>

using namespace std;

Filter_Hokuyo::Filter_Hokuyo() : nh_("~"){

  run();   

}

void  Filter_Hokuyo::run(){
 Load_Global_Parameters();
 laser_scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &Filter_Hokuyo::Scan_Hokuyo_callback, this);  
 laser_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_filter", 100);
 ros::spin();
}

Filter_Hokuyo::~Filter_Hokuyo(){
 laser_scan_sub_.shutdown();
 laser_scan_pub_.shutdown();
}

void Filter_Hokuyo::Load_Global_Parameters(void){
  nh_.param("minimum_range",minimum_range_,0.25);
  nh_.param("maximum_range",maximum_range_,10.00);

}

//filters the sonar readings to a specified min and max range.
void Filter_Hokuyo::Scan_Hokuyo_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
 double laser_frequency = 40;
 sensor_msgs::LaserScan scan;
 scan.header.frame_id = msg->header.frame_id;
 scan.header.stamp = msg->header.stamp; 

 double length = msg->ranges.size();
 scan.angle_min =  msg->angle_min;
 scan.angle_max = msg->angle_max;
 scan.angle_increment =  msg->angle_increment;
 scan.time_increment = msg->time_increment;;
 scan.range_min = minimum_range_; 
 scan.range_max = msg->range_max;
 scan.range_max = maximum_range_; 
 scan.ranges.resize(length);
 
 for(int i = 0; i < length; i++){
   value_ = msg->ranges[i];
     if(minimum_range_ <= value_ && value_ <= maximum_range_){  
       scan.ranges[i] =  msg->ranges[i];
	 } else if(!isfinite(value_) && value_ < 0){
	   scan.ranges[i]= maximum_range_;
	 } else if(!isfinite(value_) && value_ > 0){
	   scan.ranges[i]= maximum_range_;
	 } else if(isnan(value_) ){
	   scan.ranges[i]= maximum_range_;
	 } else {
	   scan.ranges[i]= maximum_range_;
	 }
     }
 laser_scan_pub_.publish(scan);
}




int main(int argc, char** argv){
  ros::init(argc, argv, "filter_hokuyo");
  Filter_Hokuyo fake_xtionlaser;
   
  return (0);
}
