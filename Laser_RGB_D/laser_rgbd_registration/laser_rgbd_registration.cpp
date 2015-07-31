#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/PointCloud.h"
// PCL specific includes
 #include <pcl_conversions/pcl_conversions.h>
 #include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/point_cloud_conversion.h>

//message filter libraries
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


using namespace sensor_msgs;
using namespace message_filters;


ros::Publisher pub;
ros::Publisher Cloud_SYN_laser_scan_pub_;
ros::Publisher Cloud_SYN_xtion_scan_pub_;
ros::Publisher SYN_xtion_scan_pub_;
ros::Publisher SYN_laser_scan_pub_;


//void  callback(const sensor_msgs::PointCloud2ConstPtr& PC2_laser)
  void callback(const sensor_msgs::PointCloud2ConstPtr& PC2_laser, const sensor_msgs::PointCloud2ConstPtr& PC2_xtion)
//void callback(const ImageConstPtr& image, const CameraInfoConstPtr& cam_info)
{



 
  sensor_msgs::PointCloud2 SYN_laser_msg;
  SYN_laser_msg.header.frame_id = PC2_laser->header.frame_id;
  SYN_laser_msg.header.stamp = PC2_laser->header.stamp;
  SYN_laser_scan_pub_.publish(PC2_laser);


  sensor_msgs::PointCloud2 SYN_xtion_msg;
  SYN_xtion_msg.header.frame_id = PC2_xtion->header.frame_id;
  SYN_xtion_msg.header.stamp = PC2_xtion->header.stamp;
  //SYN_xtion_scan_pub_.publish(PC2_xtion);


  sensor_msgs::PointCloud Cloud_SYN_laser_msg;
  Cloud_SYN_laser_msg.header.frame_id = PC2_laser->header.frame_id;
  Cloud_SYN_laser_msg.header.stamp = PC2_laser->header.stamp;
  sensor_msgs::convertPointCloud2ToPointCloud(*PC2_laser,Cloud_SYN_laser_msg);
  Cloud_SYN_laser_scan_pub_.publish(Cloud_SYN_laser_msg);

  sensor_msgs::PointCloud Cloud_SYN_xtion_msg;
  Cloud_SYN_xtion_msg.header.frame_id = PC2_xtion->header.frame_id;
  Cloud_SYN_xtion_msg.header.stamp = PC2_xtion->header.stamp;

  //SYN_xtion_scan_pub_.publish(PC2_xtion);



// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  //pcl::PointCloud<pcl::PointXYZ> laser_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*PC2_laser, *laser_cloud);

  //pcl::PointCloud<pcl::PointXYZ> xtion_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xtion_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*PC2_xtion, *xtion_cloud);



  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
icp.setMaxCorrespondenceDistance (0.35);
// Set the maximum number of iterations (criterion 1)
icp.setMaximumIterations (100);
 icp.setRANSACIterations (30); 	
// Set the transformation epsilon (criterion 2)
//Set the transformation epsilon (maximum allowable difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution. 
icp.setTransformationEpsilon (1e-14);
// Set the euclidean distance difference epsilon (criterion 3)
icp.setEuclideanFitnessEpsilon (3);


  icp.setInputSource(xtion_cloud);
  icp.setInputTarget(laser_cloud);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  //std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    // icp.getFitnessScore() << std::endl;
    //std::cout << icp.getFinalTransformation() << std::endl;

  pcl::toROSMsg(Final, SYN_xtion_msg);
  SYN_xtion_scan_pub_.publish(SYN_xtion_msg);
  //pcl::fromPCLPointCloud2(Final,  SYN_xtion_msg);
  sensor_msgs::convertPointCloud2ToPointCloud(SYN_xtion_msg,Cloud_SYN_xtion_msg);
  Cloud_SYN_xtion_scan_pub_.publish(Cloud_SYN_xtion_msg);
  // SYN_xtion_scan_pub_(SYN_xtion_msg);
  
  // Solve all of perception here...


 // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  //pcl::PointCloud<pcl::PointXYZ> cloud;
  //pcl::fromROSMsg (*input, cloud);


  /*
  pcl::PCLPointCloud2 cloud_filtered;	
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (input);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_filtered);

  // Publish the dataSize 
  pub.publish (cloud_filtered);
  */
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "laser_rgbd_registration");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("PC2_laser_coordinates", 1, callback);

  message_filters::Subscriber<sensor_msgs::PointCloud2> laser_sub(nh, "PC2_laser_coordinates", 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> xtion_sub(nh,  "PC2_xtion_coordinates", 10);


  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),  laser_sub, xtion_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));


  //message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> sync(laser_sub, xtion_sub, 10);
  //sync.registerCallback(boost::bind(&callback, _1, _2));


  

   /*
  message_filters::Subscriber<sensor_msgs::PointCloud2> laser_sub(nh, "PC2_laser_coordinates", 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> xtion_sub(nh,  "PC2_xtion_coordinates", 10);
  TimeSynchronizer<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> sync(laser_sub, xtion_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));
   */





  
  /*
 message_filters::Subscriber<Image> image_sub(nh, "image", 1);
  message_filters::Subscriber<CameraInfo> info_sub(nh, "camera_info", 1);
  TimeSynchronizer<Image, CameraInfo> sync(image_sub, info_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  */
  // Create a ROS publisher for the output point cloud
   Cloud_SYN_laser_scan_pub_ = nh.advertise<sensor_msgs::PointCloud>("/Cloud_SYN_laser_msg", 2);
   Cloud_SYN_xtion_scan_pub_ = nh.advertise<sensor_msgs::PointCloud>("/Cloud_SYN_xtion_msg", 2);
   SYN_xtion_scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/SYN_xtion_msg", 2);
   SYN_laser_scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/SYN_laser_msg", 2);
  // Spin


	ros::Rate loop_rate(10);
	while(ros::ok())
	{	
	ros::spinOnce();
	loop_rate.sleep();
	}
return 0;
 Cloud_SYN_laser_scan_pub_.shutdown();
 Cloud_SYN_xtion_scan_pub_.shutdown();
 SYN_xtion_scan_pub_.shutdown();
 SYN_laser_scan_pub_.shutdown();
 //ROS_INFO("Shutting down!!!!!!!");
 //return 0;
}
