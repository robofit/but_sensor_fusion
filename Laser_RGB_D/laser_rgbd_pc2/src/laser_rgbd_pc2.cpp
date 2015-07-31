#include <iostream>
#include <math.h>
#include <vector>
#include <list>
//#include <Pose2d.hh>
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

//#define forced_max_range // if defiend, this causes readings > max range to be reset to max range
using namespace std;

  

class Comb_FakeXtion_Laser{
private:
  struct POSE{
    double x;
    double y;
    double z;
  };

  POSE* robot_pose_;
  ros::NodeHandle nh_;

// global ROS subscriber handles
  ros::Subscriber scan_xtion_sub_;
  ros::Subscriber scan_laser_sub_;
  ros::Subscriber odom_sub_;

// global ROS publisher handles
  ros::Publisher laser_scan_pub_;
  ros::Publisher xtion_scan_pub_;
  ros::Publisher PC2_laser_scan_pub_;
  ros::Publisher PC2_xtion_scan_pub_;

  double R2D_;
  bool success;
  bool robot_pose_receive_;
  bool scan_xtion,scan_fake;

  int length_x,length_f,length_xtion,length_fake,total_length;
  // double *temp_xtion_ranges, *temp_fake_ranges;
  double temp_xtion_ranges[], temp_fake_ranges[];
  //int x;
 
public:
  Comb_FakeXtion_Laser();
  ~Comb_FakeXtion_Laser();
  void Scan_Xtion_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void Scan_Laser_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void Load_Global_Parameters(void);
  POSE* make_pose(double x, double y, double z);
  //void broadcast_scanner_tf();

  laser_geometry::LaserProjection xtion_projector_;
  laser_geometry::LaserProjection laser_projector_;
  tf::TransformListener xtion_listener_;
  tf::TransformListener laser_listener_;
  void run();
    

};


 Comb_FakeXtion_Laser::Comb_FakeXtion_Laser() : nh_("~"){
   ROS_INFO("Entering run.");
   run();
}




void  Comb_FakeXtion_Laser::run(){

  Load_Global_Parameters();
  
  //set up subscribers
  scan_xtion_sub_ = nh_.subscribe<sensor_msgs::LaserScan> ("/scan_xtion", 100, &Comb_FakeXtion_Laser::Scan_Xtion_Callback, this);  
  scan_laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_filter", 100, &Comb_FakeXtion_Laser::Scan_Laser_Callback, this);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odometer_pose_pb", 100, &Comb_FakeXtion_Laser::Odom_Callback, this);

  // set up publishers
   laser_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/laser_coordinates", 2);
   xtion_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/xtion_coordinates", 2);
   PC2_laser_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/PC2_laser_coordinates", 2);
   PC2_xtion_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/PC2_xtion_coordinates", 2);

    ros::spin();
   
}

 Comb_FakeXtion_Laser::~Comb_FakeXtion_Laser(){

  // clean up subscribers and publishers
   scan_xtion_sub_.shutdown();
   scan_laser_sub_.shutdown();
   laser_scan_pub_.shutdown();
   xtion_scan_pub_.shutdown();
   PC2_laser_scan_pub_.shutdown();
   PC2_xtion_scan_pub_.shutdown();
   odom_sub_.shutdown();
   free (robot_pose_);
   ROS_INFO("Shutting down!!!!!!!");
 
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



void Comb_FakeXtion_Laser::Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg){

  // init pose
  if(robot_pose_ == NULL)
    robot_pose_ = make_pose(0.0,0.0,0.0);
    
  robot_pose_->x =  msg->pose.pose.position.x;
  robot_pose_->y =  msg->pose.pose.position.y;
  robot_pose_->z =  msg->pose.pose.orientation.w;//*G2R;

  robot_pose_receive_=true;

 
}



void Comb_FakeXtion_Laser::Scan_Xtion_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){

 ////////////-old-///////////////////////////////////////////////////////////////
 //// sensor_msgs::PointCloud xtion_msg;
 //// xtion_msg.header.frame_id = msg->header.frame_id;//"/camera_depth_frame";
  //ros::Time scan_time = ros::Time::now();
 //// xtion_msg.header.stamp = ros::Time(0); //msg->header.stamp;//scan_time;
 ////////////////-old-//////////////////////////////////////////////////////////////

  sensor_msgs::PointCloud2 PC2_xtion_msg;
  PC2_xtion_msg.header.frame_id = msg->header.frame_id;
  PC2_xtion_msg.header.stamp = ros::Time(0); //msg->header.stamp;


if(!xtion_listener_.waitForTransform(
        msg->header.frame_id,
        "/laser",
        msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),
        ros::Duration(1.0))){
  
     return;
  }


  sensor_msgs::PointCloud xtion_msg;
  xtion_projector_.transformLaserScanToPointCloud("/laser",*msg,
           xtion_msg,xtion_listener_);

 xtion_scan_pub_.publish(xtion_msg);
sensor_msgs::convertPointCloudToPointCloud2(xtion_msg,PC2_xtion_msg);
  PC2_xtion_scan_pub_.publish(PC2_xtion_msg);


}

void Comb_FakeXtion_Laser::Scan_Laser_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
 
  

  

  sensor_msgs::PointCloud laser_msg;
  laser_msg.header.frame_id = msg->header.frame_id;// "/laser";
  laser_msg.header.stamp = msg->header.stamp;//msg->scan_time;

  sensor_msgs::PointCloud2 PC2_laser_msg;
  PC2_laser_msg.header.frame_id = msg->header.frame_id;
  PC2_laser_msg.header.stamp = msg->header.stamp;


  
  try {
    laser_listener_.waitForTransform(
        msg->header.frame_id,"/laser", msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),ros::Duration(1.0));
   
  } catch (tf::ExtrapolationException e) {
  }
  

//sensor_msgs::PointCloud  cloud;
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
