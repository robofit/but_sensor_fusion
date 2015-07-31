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


#ifndef PI
  #define PI 3.1415926535897
#endif

//#define forced_max_range // if defiend, this causes readings > max range to be reset to max range
using namespace std;

  

class Filter_Hokuyo{
private:


  ros::NodeHandle nh_;

// global ROS subscriber handles
  ros::Subscriber laser_scan_sub_;
  

// global ROS publisher handles
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


 Filter_Hokuyo::Filter_Hokuyo() : nh_("~"){

 
   
  ROS_INFO("Entering run.");
  run();   
}


void  Filter_Hokuyo::run(){

  Load_Global_Parameters();
  
  //set up subscribers
  laser_scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &Filter_Hokuyo::Scan_Hokuyo_callback, this);  

  

  // set up publishers
   laser_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_filter", 100);

    ros::spin();
   
}

Filter_Hokuyo::~Filter_Hokuyo(){

  // clean up subscribers and publishers
   laser_scan_sub_.shutdown();
   laser_scan_pub_.shutdown();

   ROS_INFO("Shutting down!!!!!!!");
 
}


void Filter_Hokuyo::Load_Global_Parameters(void){
  nh_.param("minimum_range",minimum_range_,0.25);
  nh_.param("maximum_range",maximum_range_,10.00);

}





void Filter_Hokuyo::Scan_Hokuyo_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  double laser_frequency = 40;

  sensor_msgs::LaserScan scan;
  scan.header.frame_id = msg->header.frame_id	;//"/laser";
  scan.header.stamp = msg->header.stamp; //scan_time;

  
 
//populate the LaserScan message
  double length = msg->ranges.size();
  scan.angle_min =  msg->angle_min;
  scan.angle_max = msg->angle_max;
  scan.angle_increment =  msg->angle_increment;//3.14 / length;
  scan.time_increment = msg->time_increment;;//(1 / laser_frequency) / (length);
  scan.range_min = minimum_range_; // msg->range_min;
  scan.range_max = msg->range_max;
  scan.range_max = maximum_range_; 

  //maximum_range_ = scan.range_max;
  scan.ranges.resize(length);
 
//ROS_INFO(" total %d ", total_length);
       for(int i = 0; i < length; i++){
	 value_ = msg->ranges[i];
	 if(minimum_range_ <= value_ && value_ <= maximum_range_){  
	   scan.ranges[i] =  msg->ranges[i];
	 } else if(!isfinite(value_) && value_ < 0){
	   // Object too close to measure.
	   scan.ranges[i]= maximum_range_;
	 } else if(!isfinite(value_) && value_ > 0){
	   // No objects detected in range.
	   scan.ranges[i]= maximum_range_;
	 } else if(isnan(value_) ){
	   // This is an erroneous, invalid, or missing measurement.
	   scan.ranges[i]= maximum_range_;
	 } else {
	   // The sensor reported these measurements as valid, but they are discarded per the limits defined by minimum_range and maximum_range.
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
