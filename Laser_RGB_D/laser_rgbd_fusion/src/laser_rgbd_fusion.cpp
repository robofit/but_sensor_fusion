/* This is a template node class */


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
/*Macros*/

#define TEXTLEN 64


/*Standard library*/
using namespace std;


/*class definition goes here*/
class Fusion_Xtion_Laser{


private:
  struct POINT;

  ros::NodeHandle nh_;

   // global ROS subscriber handles
  ros::Subscriber scan_xtion_sub_;
  ros::Subscriber scan_laser_sub_;
  ros::Subscriber tf_pose_;
  ros::Subscriber laser_parameters_sub_;
 
  //variable type definitions go here 

// global ROS publisher handles
  ros::Publisher  laser_map_changes_pub_;
  ros::Publisher  xtion_map_changes_pub_;
  ros::Publisher  xtion_laser_map_changes_pub_;
  ros::Publisher vis_pub_;
  ros::Publisher xtion_laser_scan_pub_; 


/*---------------------- MAP --------------------------------------------*/
struct MAP{
    
  double** cost_;    //Define a pointer to pointer to double
  double height_;
  double width_;
  double resolution_;   
};

  MAP* laser_map_;
  MAP* xtion_map_;
  MAP* xtion_laser_map_;
  MAP* temp_laser_map_;
  MAP* temp_xtion_map_;

 //tf  pose variables
  double tfX_;
  double tfY_;
  double tfZ_;

  //laser parameters variables
  double length;
  double inc;
  double range_min;
  double range_max;
  double angle_min;
  double angle_max;
  double time_inc;

//Global variables
  int dataxl_;
  int datayl_;
  int dataxx_;
  int datayx_;
  double map_y_max_;    // (m), map goes from 0 to this in the y direction
  double map_x_max_;    // (m), map goes from 0 to this in the x direction
  double resolution_;   // (m), each map grid spans this much real-world distance
  double map_resolution_;
  double X_OR_,Y_OR_;

  int GR_SIZE_;
  double height_; //the height of the map in grids
  double width_;  // the width of the map in grids
  double prior_;  //Initial probability of the map 
  double emp;
  double occ; // min empty and max occupied cell values 
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

  /*Function definitions*/
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
  //void broadcast_scanner_tf();
  void run();

};

/*constructor goes here*/
Fusion_Xtion_Laser::Fusion_Xtion_Laser() : nh_("~"){

  /*Initialization of variables*/
  
  
  /*run call back functions*/
  run();   
}

void Fusion_Xtion_Laser::run(){

  /*run the global parameters*/
  Load_Global_Parameters();
 
  /*Example of subscriber*/
 //set up subscribers
  

  scan_xtion_sub_ = nh_.subscribe<sensor_msgs::PointCloud> ("/Cloud_SYN_xtion_msg", 100, &Fusion_Xtion_Laser::Scan_Xtion_Callback, this);  

  scan_laser_sub_ = nh_.subscribe<sensor_msgs::PointCloud>("/Cloud_SYN_laser_msg", 100, &Fusion_Xtion_Laser::Scan_Laser_Callback, this);
 
  tf_pose_ = nh_.subscribe<nav_msgs::Odometry>("/odometer_pose_pb", 100, &Fusion_Xtion_Laser::tf_Pose,this);
 
  laser_parameters_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 100, &::Fusion_Xtion_Laser::Laser_Parameters, this);

  // An example of setting up publishers
  laser_map_changes_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/laser_fused_map", 2);

 xtion_map_changes_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/xtion_fused_map", 2);

 xtion_laser_map_changes_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/xtion_laser_fused_map", 2);

vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "/visualization_marker", 0 );

 xtion_laser_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_xtion_laser", 100);

}

/*Run the destructor*/
 Fusion_Xtion_Laser::~ Fusion_Xtion_Laser(){

  // clean up subscribers and publishers
   scan_xtion_sub_.shutdown();
   scan_laser_sub_.shutdown();
   laser_parameters_sub_.shutdown();
   tf_pose_.shutdown();
   laser_map_changes_pub_.shutdown();
   xtion_map_changes_pub_.shutdown();
   xtion_laser_map_changes_pub_.shutdown();
   xtion_laser_scan_pub_.shutdown();
   vis_pub_.shutdown();

   free(laser_map_);
   free(xtion_map_);
   free(xtion_laser_map_);
   free(temp_laser_map_);
   free(temp_xtion_map_);

 
}


/*Global parameters*/
void Fusion_Xtion_Laser::Load_Global_Parameters(void){
 /*Initialization of variables*/
  nh_.param("resolution", resolution_, 5.0);
  nh_.param("map_resolution", map_resolution_, 0.05);
  nh_.param("tfX",tfX_, 0.0);
  nh_.param("tfY",tfY_, 0.0);
  nh_.param("tfZ",tfZ_, 0.0);
  nh_.param("map_x_size", map_x_max_, 300.0);
  nh_.param("map_y_size", map_y_max_, 300.0);
  nh_.param("height",  height_,  map_x_max_);
  nh_.param("with",  width_,  map_y_max_);
  nh_.param("prior",  prior_,0.5);
  nh_.param("emp", emp,0.4);
  nh_.param("occ", occ,0.65);

  origin_map_x_ = -(((width_)*(map_resolution_))/2);
  origin_map_y_ = -(((width_)*(map_resolution_))/2);
  Y_OR_ = abs((origin_map_x_)/(map_resolution_));
  X_OR_ = abs((origin_map_y_)/(map_resolution_));
 

  GR_SIZE_          =  300;
  laser_count       =  0; 
  xtion_count       =  0; 
  xtion_laser_count =  0;
  max_gray          =  255;

  laser_map_       =  NULL;
  xtion_map_       =  NULL;
  xtion_laser_map_ =  NULL;
  temp_laser_map_  =  NULL;
  temp_xtion_map_  =  NULL;

  destroy_map(laser_map_);
  destroy_map(xtion_map_);
  destroy_map(xtion_laser_map_);
  destroy_map(temp_xtion_map_);
  destroy_map(temp_laser_map_);

  laser_map_ = make_map(height_, width_,resolution_,prior_);
  xtion_map_ = make_map(height_, width_,resolution_,prior_);
  xtion_laser_map_ = make_map(height_, width_,resolution_,prior_);
  temp_laser_map_ = make_map(height_, width_,resolution_,prior_);
  temp_xtion_map_ = make_map(height_, width_,resolution_,prior_);
}

// this allocates all required memory for a map
void Fusion_Xtion_Laser::destroy_map(MAP* map){
  int y;
  if(map != NULL)
  {
    for (y = 0; y < map->height_; y++)  
    {
	  if(map->cost_[y] != NULL)
        free(map->cost_[y]);
    }

    if(map->cost_ != NULL)
      free(map->cost_);
                    
    map->height_ = 0;
    map->width_ = 0;
    free(map);
  }
}

// this creates and returns a pointer to a MAP struct
 Fusion_Xtion_Laser::MAP *Fusion_Xtion_Laser::make_map(double height, double width, double resolution,double initial){

  MAP* map = (MAP*)calloc(1, sizeof(MAP));
  map->height_ = height;
  map->width_  = width;
  map->resolution_ = resolution_;

  //Two-dimensional array with calloc

  //Allocate a one-dimensional array of pointers to double 
  map->cost_ = (double**)calloc(height, sizeof(double*));
  for (int y = 0; y < height; y++){
    //For each of the pointer to double, allocate an array of double, let the pointer pointing to it. 
    map->cost_[y] = (double*)calloc(width, sizeof(double)); 
    for(int x = 0; x < width; x++)
      //After this, cost can be used b as if it was declared as cost[x][y]  
      map->cost_[y][x] = initial;//(rand() % 10 +1)*0.1; //initial;
  }

  //ROS_INFO("h=%f, w=%f, r=%f",  map->height_,   map->width_,  map->resolution_);
  return map;
}


void Fusion_Xtion_Laser::tf_Pose(nav_msgs::Odometry msg){
  tfX_  = (int)round((X_OR_ + (int)((msg.pose.pose.position.x)*100)/resolution_));// ((msg.pose.pose.position.x)*100);
  tfY_  =  (int)round((Y_OR_ + (int)((msg.pose.pose.position.y)*100)/resolution_));// ((msg.pose.pose.position.y)*100);
  tfZ_  =  ((msg.pose.pose.position.z)*100);
  //ROS_INFO("Received tfX:[%f] :: tfY:[%f] :: tfZ:[%f] ", tfX_, tfY_, tfZ_);
  
}

void Fusion_Xtion_Laser::Laser_Parameters(const sensor_msgs::LaserScan::ConstPtr& msg){
   length = msg->ranges.size();
   //ROS_INFO(" length %f ", length);
   inc = msg->angle_increment;
   //ROS_INFO(" inc %f ", inc); 
  //first pass, see how many values are actually in range
   range_min = 0.25;//msg->range_min;
   //ROS_INFO("range_min  %f ", range_min);
   range_max =10.0;// msg->range_max;
   //ROS_INFO(" range_max %f ", range_max);
   angle_min = msg->angle_min;
   //ROS_INFO("angle_min %f ", angle_min);
   angle_max = msg->angle_max;
   //ROS_INFO(" angle_max %f ", angle_max);
   time_inc=msg->time_increment;
}

/*call back function*/
void Fusion_Xtion_Laser::Scan_Xtion_Callback(const sensor_msgs::PointCloud::ConstPtr& msg){
  int xtion_length=msg->points.size();

 for (int j = 0; j <  GR_SIZE_; j++)
   for(int k = 0; k <  GR_SIZE_; k++)
     xtion_map_->cost_[k][j] = 0.5;

 for(int i=0; i<xtion_length;i++){

   dataxx_= (int)round(((msg->points[i].x)*100)/resolution_) + X_OR_;
   datayx_= (int)round(((msg->points[i].y)*100)/resolution_) + Y_OR_;
    
     /*Compute the sensor fusion based laser readings*/



   Xtion_Fusion(X_OR_ /*tfX_*/, Y_OR_ /*tfY_*/, dataxx_, datayx_, GR_SIZE_ );
   
   }


  



 //publish laser fusion
 publish_xtion_map_changes();
 
}



void Fusion_Xtion_Laser::xtion_write(){

FILE* out;
  
 
 //char *outputFilename;
  //const char *outputFilename =  "/home/acp/albotino/figure/laser.pgm";
  //sprintf(outputFilename, "/home/acp/albotino/figure_laser/laser%d.pgm",laser_count);
  sprintf(outputFilename, "/home/acp/patrol_ws/sandbox/fusion_xtion_laser/max_files/xtion.pgm");
  out = fopen(outputFilename, "w");
  if (!out){
    ROS_ERROR("Couldn't open map file to %s", out);
    return;
  }
  //else 
  //cout << "The file laser.pgm is successfully opened " << endl;
  //  sprintf( "/home/acp/albotino/figure/laser%d.pgm",laser_count++);
  fprintf(out, "P5");
  fprintf(out, "%d %d ",(int)xtion_map_->height_ ,(int)xtion_map_->width_);
  fprintf(out, "%d ", max_gray);
  for (int i = 0; i < (int)xtion_map_->width_; ++i){
    for (int j = 0; j < (int)xtion_map_->height_; ++j){
      fputc((int)(xtion_map_->cost_[i][j]*255), out);
    }
  }
  fclose(out);



 
 /*Open file to store the [x,y,z] and the uncertainty of the descriptors in .log file to be open in matlab

 /*Open file   data to store the fusion data */

  
       if((out=fopen("/home/acp/patrol_ws/sandbox/fusion_xtion_laser/max_files/xtion.log","wb"))==NULL){
	printf("\nCant open file\n");
	exit (0);
	      }
       //printf("landmark size %d,,%d\n",landmarks.size(),landmarks.size()*4);
       // for(j=0;j<27;j++){
       //fprintf(ptr,"%d\n",length);
       //fprintf(ptr, "P5");
       //fprintf(ptr, "%d %d ",(int)xtion_laser_map_->height_ ,(int)xtion_laser_map_->width_);
  
 for (int j = 0; j < (int)xtion_map_->width_; ++j){
    for (int i = 0; i < (int)xtion_map_->height_; ++i){
       fprintf(out," %f",xtion_map_->cost_[i][j]);
      //fwrite(&xtion_laser_map_, sizeof(double), sizeof(xtion_laser_map_), ptr);
    }
    fprintf(out,"\n");
  }
  
       //fwrite(&xtion_laser_map_, sizeof(double), sizeof(xtion_laser_map_), ptr);
 // fprintf(ptr,"\n");
	 // }
 
	fclose(out);

}

void Fusion_Xtion_Laser::Scan_Laser_Callback(const sensor_msgs::PointCloud::ConstPtr& msg){

  //GR_SIZE_ = (int)laser_map_->width_;
  int laser_length=msg->points.size();



 for (int j = 0; j <  GR_SIZE_; j++)
   for(int k = 0; k <  GR_SIZE_; k++)
     laser_map_->cost_[k][j] = 0.5;

 for(int i=0; i<laser_length;i++){

   dataxl_= (int)round(((msg->points[i].x)*100)/resolution_) + X_OR_;
   datayl_= (int)round(((msg->points[i].y)*100)/resolution_) + Y_OR_;
    



     /*Compute the sensor fusion based laser readings*/
   Laser_Fusion(X_OR_ /*tfX_*/, Y_OR_ /*tfY_*/, dataxl_, datayl_, GR_SIZE_ );
   
   }



 //publish laser fusion
 publish_laser_map_changes();
 //xtion_laser_fused();


 
}

void Fusion_Xtion_Laser::laser_write(){

FILE* out;
  
 
 //char *outputFilename;
  //const char *outputFilename =  "/home/acp/albotino/figure/laser.pgm";
  //sprintf(outputFilename, "/home/acp/albotino/figure_laser/laser%d.pgm",laser_count);
  sprintf(outputFilename, "/home/acp/patrol_ws/sandbox/fusion_xtion_laser/max_files/laser.pgm");
  out = fopen(outputFilename, "w");
  if (!out){
    ROS_ERROR("Couldn't open map file to %s", out);
    return;
  }
  //else 
  //cout << "The file laser.pgm is successfully opened " << endl;
  //  sprintf( "/home/acp/albotino/figure/laser%d.pgm",laser_count++);
  fprintf(out, "P5");
  fprintf(out, "%d %d ",(int)laser_map_->height_ ,(int)laser_map_->width_);
  fprintf(out, "%d ", max_gray);
  for (int i = 0; i < (int)laser_map_->width_; ++i){
    for (int j = 0; j < (int)laser_map_->height_; ++j){
      fputc((int)(laser_map_->cost_[i][j]*255), out);
    }
  }
  fclose(out);



 
 /*Open file to store the [x,y,z] and the uncertainty of the descriptors in .log file to be open in matlab

 /*Open file   data to store the fusion data */

  
       if((out=fopen("/home/acp/patrol_ws/sandbox/fusion_xtion_laser/max_files/laser.log","wb"))==NULL){
	printf("\nCant open file\n");
	exit (0);
	      }
       //printf("landmark size %d,,%d\n",landmarks.size(),landmarks.size()*4);
       // for(j=0;j<27;j++){
       //fprintf(ptr,"%d\n",length);
       //fprintf(ptr, "P5");
       //fprintf(ptr, "%d %d ",(int)xtion_laser_map_->height_ ,(int)xtion_laser_map_->width_);
  
 for (int j = 0; j < (int)laser_map_->width_; ++j){
    for (int i = 0; i < (int)laser_map_->height_; ++i){
       fprintf(out," %f",laser_map_->cost_[i][j]);
      //fwrite(&xtion_laser_map_, sizeof(double), sizeof(xtion_laser_map_), ptr);
    }
    fprintf(out,"\n");
  }
  
       //fwrite(&xtion_laser_map_, sizeof(double), sizeof(xtion_laser_map_), ptr);
 // fprintf(ptr,"\n");
	 // }
 
	fclose(out);


}

void Fusion_Xtion_Laser::xtion_laser_fused(void){



  
   for (int y = 0; y < laser_map_->height_; y++){
     for(int x = 0; x < laser_map_->width_; x++){
    
  	temp_laser_map_->cost_[y][x] = laser_map_->cost_[y][x];
     	temp_xtion_map_->cost_[y][x] = xtion_map_->cost_[y][x];
     }
   }

   
  for (int y = 0; y < laser_map_->height_; y++){
     for(int x = 0; x < laser_map_->width_; x++){
       if(laser_map_->cost_[y][x]>0.5)
	 temp_laser_map_->cost_[y][x] = 1;//(2*(temp_laser_map_->cost_[y][x]))-0.5;
       else if(temp_laser_map_->cost_[y][x]>0.95)
        temp_laser_map_->cost_[y][x] = 1;
       else if(temp_laser_map_->cost_[y][x]<0.5)
 	temp_laser_map_->cost_[y][x] = 0.05;
       //ROS_INFO(" z = %f", temp_laser_map_->cost_[y][x]);
     }
  }

for (int y = 0; y < xtion_map_->height_; y++){
     for(int x = 0; x < xtion_map_->width_; x++){
       if(xtion_map_->cost_[y][x]>0.5)
	 temp_xtion_map_->cost_[y][x] = 1;//(2*(temp_xtion_map_->cost_[y][x]))-0.5;
       else if(temp_xtion_map_->cost_[y][x]>0.95)
        temp_xtion_map_->cost_[y][x] = 1;
       else if(temp_xtion_map_->cost_[y][x]<0.5)
 	temp_xtion_map_->cost_[y][x] = 0.05;
       //ROS_INFO(" z = %f", temp_laser_map_->cost_[y][x]);
     }
  }

 


 for(int i=0;i<1;i++){


for (int y = 0; y < xtion_laser_map_->height_; y++){
  for(int x = 0; x < xtion_laser_map_->width_; x++){
  
    xtion_laser_map_->cost_[y][x]=(temp_laser_map_->cost_[y][x]*temp_xtion_map_->cost_[y][x])/((temp_laser_map_->cost_[y][x]*temp_xtion_map_->cost_[y][x])+((1-temp_laser_map_->cost_[y][x])*(1-temp_xtion_map_->cost_[y][x])));

    //ROS_INFO(" z = %f", xtion_laser_map_->cost_[x][y]); 
  }
 }
}


 
 double laser_inc=0.0;
 double slope=0.0;
 double r;
 double orientationvector[2]={0.0,0.0};
 double Xpart_x=150.0;
 double Xpart_y=150.0;
 double beamCoor[2]={Xpart_x,Xpart_y};
 int wallHit=0;
 int beamCoorInt[2]={0,0};
 int xx;
 int yy;
 
 double laser_frequency = 40;
 sensor_msgs::LaserScan scan_msg;
 scan_msg.header.frame_id = "/laser";
 //ros::Time scan_time = ros::Time::now();
  scan_msg.header.stamp = ros::Time::now(); //scan_time;

 
  scan_msg.angle_min =  - angle_min;
  scan_msg.angle_max =  angle_max;


  

  scan_msg.angle_increment = -inc;
  scan_msg.time_increment =  time_inc;//(1 / laser_frequency) / (length);
  scan_msg.range_min = range_min; // msg->range_min;
  scan_msg.range_max = range_max;
  scan_msg.ranges.resize(length);



 //xtion_laser_map_->cost_[0][150]=1.0;
 //ROS_INFO("start process");


 for(int i=1;i<(length-3);i++){
 
  
   laser_inc =  i*inc;//angle_min;
   orientationvector[0] =  -sin(-laser_inc);
   orientationvector[1] =  cos(-laser_inc);
  
    while(wallHit==0){  
   
    beamCoorInt[0] = (int)round(beamCoor[0]);
    beamCoorInt[1] = (int)round(beamCoor[1]);

    
   
   
    
      if((beamCoorInt[0] >= 299)  || (beamCoorInt[1] >= 299))
	wallHit = 1;

     
      
    if((xtion_laser_map_->cost_[beamCoorInt[0]][beamCoorInt[1]]) > 0.9){
      
	    wallHit = 1;
	    
	    double distance = sqrt(pow((150-beamCoorInt[0]),2) + pow((150-beamCoorInt[1]),2)); //why 0.25 and not 0.2 ????


   
	 value_ = (distance/5)*0.25;
	 if(range_min <= value_ && value_ <=  range_max)
	   scan_msg.ranges[i] =  value_;
	







	 

	   
    }
   

    //ROS_INFO("% f",sqrt(pow(xx,2) + pow(yy,2)));	  
   beamCoor[0] = beamCoor[0] + orientationvector[0];
   beamCoor[1] = beamCoor[1] + orientationvector[1]; 

  



     
     
    }//while(wallHit==0);
    wallHit = 0;
   beamCoor[0]=Xpart_x;
   beamCoor[1]=Xpart_y;
 
   
 }

 xtion_laser_scan_pub_.publish(scan_msg);
 


 //printf("\n\n ");

 publish_xtion_laser_map_changes();
 
  
 if(xtion_laser_map_ == NULL)
     return;
  int length =  xtion_laser_map_->height_ *  xtion_laser_map_->width_;
  
  
 
}

/*------------------ ROS Publisher's Xtion-Laser Function ----------------*/
void Fusion_Xtion_Laser::publish_xtion_laser_map_changes(){

 nav_msgs::OccupancyGrid  msg;
  msg.header.frame_id = "/laser";
  msg.header.stamp = ros::Time::now();

  msg.info.width  = GR_SIZE_;       //witdth of the map
  msg.info.height = GR_SIZE_;      //height of the map
  int mapsize   =  msg.info.width * msg.info.height; //size of the global map
  msg.info.resolution = map_resolution_; //The map resolution [m/cell]
  /*-----The origin of the map [m, m, rad].  This is the real-world pose of the
         cell (0,0) in the map-----*/
  msg.info.origin.position.x = origin_map_x_;  
  msg.info.origin.position.y = origin_map_y_ ;
 msg.data.resize(GR_SIZE_ * GR_SIZE_);
   
 //ROS_INFO("x = %f, y = %f",origin_map_x_,origin_map_y_); 
    
 int k=0;
  //int k=0;
 for (int j = 0; j <  GR_SIZE_; j++)
   for(int i = 0; i <  GR_SIZE_; i++){
     if(xtion_laser_map_->cost_[i][j] == 0.5)
       msg.data[k] = -1;

     else  if(xtion_laser_map_->cost_[i][j] > 0.5){ 
       msg.data[k] = 100-((xtion_laser_map_->cost_[i][j])*100);
     
     }
     else
       msg.data[k] = 100-((xtion_laser_map_->cost_[i][j])*100);
     k++;  
   }
  
  //printf("\n \n \n"); 
  xtion_laser_map_changes_pub_.publish(msg);

}

/*---------------------- ROS Publisher's Laser Function ------------------------*/
void Fusion_Xtion_Laser::publish_laser_map_changes(){

  nav_msgs::OccupancyGrid  msg;
  msg.header.frame_id = "/laser";
  msg.header.stamp = ros::Time::now();

  msg.info.width  = GR_SIZE_;       //witdth of the map
  msg.info.height = GR_SIZE_;      //height of the map
  int mapsize   =  msg.info.width * msg.info.height; //size of the global map
  msg.info.resolution = map_resolution_; //The map resolution [m/cell]
  /*-----The origin of the map [m, m, rad].  This is the real-world pose of the
         cell (0,0) in the map-----*/
  msg.info.origin.position.x = origin_map_x_;  
  msg.info.origin.position.y = origin_map_y_ ;
 msg.data.resize(GR_SIZE_ * GR_SIZE_);
   
 //ROS_INFO("x = %f, y = %f",origin_map_x_,origin_map_y_); 

 int k=0;
  //int k=0;
 for (int j = 0; j <  GR_SIZE_; j++)
   for(int i = 0; i <  GR_SIZE_; i++){
     if(laser_map_->cost_[i][j] == 0.5)
       msg.data[k] = -1;

     else  msg.data[k] = 100-((laser_map_->cost_[i][j])*100);
       
     k++;  
   }
  
  //printf("\n \n \n"); 
  laser_map_changes_pub_.publish(msg);
}


/*---------------------- ROS Publisher's Laser Function ------------------------*/
void Fusion_Xtion_Laser::publish_xtion_map_changes(){

  nav_msgs::OccupancyGrid  msg;
  msg.header.frame_id = "/laser";
  msg.header.stamp = ros::Time::now();

  msg.info.width  = GR_SIZE_;       //witdth of the map
  msg.info.height = GR_SIZE_;      //height of the map
  int mapsize   =  msg.info.width * msg.info.height; //size of the global map
  msg.info.resolution = map_resolution_; //The map resolution [m/cell]
  /*-----The origin of the map [m, m, rad].  This is the real-world pose of the
         cell (0,0) in the map-----*/
  msg.info.origin.position.x = origin_map_x_;  
  msg.info.origin.position.y = origin_map_y_;
 msg.data.resize(GR_SIZE_ * GR_SIZE_);


 int k=0;
  //int k=0;
 for (int j = 0; j <  GR_SIZE_; j++)
   for(int i = 0; i <  GR_SIZE_; i++){
     if(xtion_map_->cost_[i][j] == 0.5)
       msg.data[k] = -1;

     
     else
       msg.data[k]=100-((xtion_map_->cost_[i][j])*100);
       
     k++;  
   }
  
  //printf("\n \n \n"); 
  xtion_map_changes_pub_.publish(msg);
}
///////////////////////////-End publish function-/////////////////////////////

/*---------------------- ROS  Laser's Fusion Function ------------------------*/
void  Fusion_Xtion_Laser::Laser_Fusion(int c_x_, int c_y_, int dataxl_, int datayl_, int GR_SIZE){
  //  ROS_INFO("c_x_ = %d, c_y_ = %d, dataxl_ = %d, datayl_ = %d,  GR_SIZE = %d", c_x_, c_y_, dataxl_, datayl_, GR_SIZE);


 

  if(abs(dataxl_-c_x_)>abs(datayl_-c_y_)){
       
       if(dataxl_>c_x_){
	 
       step = -1;
       koef = (double)(datayl_-c_y_)/(dataxl_-c_x_);
       
       for(xx=((dataxl_-c_x_)+2*step); xx>-1;--xx){
	  
	 yy = c_y_+(int)round(xx*koef);
	 x0 = (int)(round(xx+c_x_));
	 
	 if ((xx+c_x_>0 && xx+c_x_<=(GR_SIZE-1)) && (yy>0 && yy<=(GR_SIZE-1))){
	    
            
	   laser_map_->cost_[x0][yy]=(emp*laser_map_->cost_[x0][yy])/(emp*(laser_map_->cost_[x0][yy])+(1-emp)*(1-laser_map_->cost_[x0][yy]));
	  }
	 }

	 for (xx=(dataxl_-c_x_-step); xx>(dataxl_-c_x_+0.5*step);--xx){
	   yy = (int)(round(c_y_+xx*koef));
	   yy = c_y_+(int)(round(xx*koef));
	 x0 = (int)(round(xx+c_x_));
	 if (x0>0 && x0<=(GR_SIZE-1) && yy>0 && yy<=(GR_SIZE-1)){
	  

	   laser_map_->cost_[x0][yy]= (occ*laser_map_->cost_[x0][yy])/(occ*(laser_map_->cost_[x0][yy])+(1-occ)*(1-laser_map_->cost_[x0][yy]));
	 }
	 }
       }
       else{
	 
       step = 1;
   
       koef = (double)(datayl_-c_y_)/(dataxl_-c_x_);
      
       for(xx=((dataxl_-c_x_)+2*step); xx<1;++xx){
	 
	   yy = (int)(round(c_y_+xx*koef));
	   x0 = (int)(round(xx+c_x_));
        if (xx+c_x_>0 && xx+c_x_<=(GR_SIZE-1) && yy>0 && yy<=(GR_SIZE-1))
	 

	  laser_map_->cost_[x0][yy] = (emp*laser_map_->cost_[x0][yy])/(emp*(laser_map_->cost_[x0][yy])+(1-emp)*(1-laser_map_->cost_[x0][yy]));
       }

       for (xx=((dataxl_-c_x_)-step); xx<((dataxl_-c_x_)+0.5*step);++xx){
	   yy = (int)(round(c_y_+xx*koef));
	 x0 = (int)(round(xx+c_x_));
	 if (x0>0 && x0<=(GR_SIZE-1) && yy>0 && yy<=(GR_SIZE-1)){
	   


	   laser_map_->cost_[x0][yy]= (occ*laser_map_->cost_[x0][yy])/(occ*(laser_map_->cost_[x0][yy])+(1-occ)*(1-laser_map_->cost_[x0][yy]));

	 }
	 }
       }

     }

      else{
	
  if(datayl_>c_y_){
   
       step = -1;
       koef = (double)(dataxl_-c_x_)/(datayl_-c_y_);
       
       for(yy=((datayl_-c_y_)+2*step); yy>-1;--yy){
	 
	   xx = (int)(round(c_x_+yy*koef));
	   y0 = (int)(round(yy+c_y_));
        if (xx>0 && xx<=(GR_SIZE-1) && y0>0 && y0<=(GR_SIZE-1))
	  

	  laser_map_->cost_[xx][y0] = (emp*laser_map_->cost_[xx][y0])/(emp*(laser_map_->cost_[xx][y0])+(1-emp)*(1-laser_map_->cost_[xx][y0]));

	 }

       for (yy=((datayl_-c_y_)-step); yy>((datayl_-c_y_)+0.5*step);--yy){
	 
	   xx = (int)(round(c_x_+yy*koef));
	 y0 = (int)(round(yy+c_y_));
	 if (xx>0 && xx<=(GR_SIZE-1) && y0>0 && y0<=(GR_SIZE-1)){
	   

	   laser_map_->cost_[xx][y0] = (occ*laser_map_->cost_[xx][y0])/(occ*(laser_map_->cost_[xx][y0])+(1-occ)*(1-laser_map_->cost_[xx][y0]));

	 }
       }
  }
       else{
	 
       step = 1;
   
       koef =(double)(dataxl_-c_x_)/(datayl_-c_y_);
       
       for(yy=((datayl_-c_y_)+2*step); yy<-1;++yy){
	
	   xx = (int)(round(c_x_+yy*koef));
	   y0 = (int)(round(yy+c_y_));
        if (xx>0 && xx<=(GR_SIZE-1) && y0>0 && y0<=(GR_SIZE-1))
	  
	 
	  laser_map_->cost_[xx][y0] = (emp*laser_map_->cost_[xx][y0])/(emp*(laser_map_->cost_[xx][y0])+(1-emp)*(1-laser_map_->cost_[xx][y0]));
       }

       for (yy=(datayl_-c_y_-step); yy<((datayl_-c_y_)+0.5*step);++yy){
	   
	   xx = (int)(round(c_x_+yy*koef));
	 y0 = (int)(round(yy+c_y_));
	 if (xx>0 && xx<=(GR_SIZE-1) && y0>0 && y0<=(GR_SIZE-1)){
	   
	   
	   laser_map_->cost_[xx][y0] = (occ*laser_map_->cost_[xx][y0])/(occ*(laser_map_->cost_[xx][y0])+(1-occ)*(1-laser_map_->cost_[xx][y0]));
	 }
	 }
       }
      }  

  //kinect_laser_fused();
}


//////////////////////-End Laser Fusion-///////////////////////////////


/*---------------------- ROS  Xtion's Fusion Function ------------------------*/
void Fusion_Xtion_Laser::Xtion_Fusion(int c_x_, int c_y_, int dataxl_, int datayl_, int GR_SIZE){
  // ROS_INFO("c_x_ = %d, c_y_ = %d, dataxl_ = %d, datayl_ = %d,  GR_SIZE = %d", c_x_, c_y_, dataxl_, datayl_, GR_SIZE);


 

  if(abs(dataxl_-c_x_)>abs(datayl_-c_y_)){
       
       if(dataxl_>c_x_){
	
       step = -1;
       koef = (double)(datayl_-c_y_)/(dataxl_-c_x_);
       
       for(xx=((dataxl_-c_x_)+2*step); xx>-1;--xx){
	  
	 yy = c_y_+(int)round(xx*koef);
	 x0 = (int)(round(xx+c_x_));
	 
	 if ((xx+c_x_>0 && xx+c_x_<=(GR_SIZE-1)) & (yy>0 && yy<=(GR_SIZE-1)))
	    
 
	   xtion_map_->cost_[x0][yy]=(emp*xtion_map_->cost_[x0][yy])/(emp*(xtion_map_->cost_[x0][yy])+(1-emp)*(1-xtion_map_->cost_[x0][yy]));
	  
	 }

	 for (xx=(dataxl_-c_x_-step); xx>(dataxl_-c_x_+0.5*step);--xx){
	   yy = (int)(round(c_y_+xx*koef));
	   yy = c_y_+(int)(round(xx*koef));
	 x0 = (int)(round(xx+c_x_));
	 if (x0>0 && x0<=(GR_SIZE-1) && yy>0 && yy<=(GR_SIZE-1)){
	   

	   xtion_map_->cost_[x0][yy]=(occ*xtion_map_->cost_[x0][yy])/(occ*(xtion_map_->cost_[x0][yy])+(1-occ)*(1-xtion_map_->cost_[x0][yy]));
	 }
	 }
       }
       else{
	 
       step = 1;
   
       koef = (double)(datayl_-c_y_)/(dataxl_-c_x_);
       
       for(xx=((dataxl_-c_x_)+2*step); xx<1;++xx){
	 
	   yy = (int)(round(c_y_+xx*koef));
	   x0 = (int)(round(xx+c_x_));
        if (xx+c_x_>0 && xx+c_x_<=(GR_SIZE-1) && yy>0 && yy<=(GR_SIZE-1))
	  

	xtion_map_->cost_[x0][yy]=(emp*xtion_map_->cost_[x0][yy])/(emp*(xtion_map_->cost_[x0][yy])+(1-emp)*(1-xtion_map_->cost_[x0][yy]));
       }

       for (xx=((dataxl_-c_x_)-step); xx<((dataxl_-c_x_)+0.5*step);++xx){
	   yy = (int)(round(c_y_+xx*koef));
	 x0 = (int)(round(xx+c_x_));
	 if (x0>0 && x0<=(GR_SIZE-1) && yy>0 && yy<=(GR_SIZE-1)){
	  


	   xtion_map_->cost_[x0][yy]=(occ*xtion_map_->cost_[x0][yy])/(occ*(xtion_map_->cost_[x0][yy])+(1-occ)*(1-xtion_map_->cost_[x0][yy]));

	 }
	 }
       }

     }

      else{
	
  if(datayl_>c_y_){
    
       step = -1;
       koef = (double)(dataxl_-c_x_)/(datayl_-c_y_);
      

       for(yy=((datayl_-c_y_)+2*step); yy>-1;--yy){
	 
	   xx = (int)(round(c_x_+yy*koef));
	   y0 = (int)(round(yy+c_y_));
        if (xx>0 && xx<=(GR_SIZE-1) && y0>0 && y0<=(GR_SIZE-1))
	  

	xtion_map_->cost_[xx][y0]=(emp*xtion_map_->cost_[xx][y0])/(emp*(xtion_map_->cost_[xx][y0])+(1-emp)*(1-xtion_map_->cost_[xx][y0]));

	 }

       for (yy=((datayl_-c_y_)-step); yy>((datayl_-c_y_)+0.5*step);--yy){
	 
	   xx = (int)(round(c_x_+yy*koef));
	 y0 = (int)(round(yy+c_y_));
	 if (xx>0 && xx<=(GR_SIZE-1) && y0>0 && y0<=(GR_SIZE-1)){
	   

	   xtion_map_->cost_[xx][y0]=(occ*xtion_map_->cost_[xx][y0])/(occ*(xtion_map_->cost_[xx][y0])+(1-occ)*(1-xtion_map_->cost_[xx][y0]));

	 }
       }
  }
       else{
	 
       step = 1;
   
       koef =(double)(dataxl_-c_x_)/(datayl_-c_y_);
       
       for(yy=((datayl_-c_y_)+2*step); yy<-1;++yy){
	 
	   xx = (int)(round(c_x_+yy*koef));
	   y0 = (int)(round(yy+c_y_));
        if (xx>0 && xx<=(GR_SIZE-1) && y0>0 && y0<=(GR_SIZE-1))
	  

	xtion_map_->cost_[xx][y0]=(emp*xtion_map_->cost_[xx][y0])/(emp*(xtion_map_->cost_[xx][y0])+(1-emp)*(1-xtion_map_->cost_[xx][y0]));
       }

       for (yy=(datayl_-c_y_-step); yy<((datayl_-c_y_)+0.5*step);++yy){
	   
	   xx = (int)(round(c_x_+yy*koef));
	 y0 = (int)(round(yy+c_y_));
	 if (xx>0 && xx<=(GR_SIZE-1) && y0>0 && y0<=(GR_SIZE-1)){
	   
	   xtion_map_->cost_[xx][y0]=(occ*xtion_map_->cost_[xx][y0])/(occ*(xtion_map_->cost_[xx][y0])+(1-occ)*(1-xtion_map_->cost_[xx][y0]));
	 }
	 }
       }
      }  
}

///////////////////////////////-End Xtion function-////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_rgbd_fusion");
  Fusion_Xtion_Laser f_xtion_laser;



ros::Rate loop_rate(10);
 
  
   while (ros::ok()){

     //if(mapper.using_tf_)
     //ROS_INFO("kinect_laser\n");    
     //f_xtion_laser.xtion_write();  
     //f_xtion_laser.laser_write();  
     f_xtion_laser.xtion_laser_fused();    
      ros::spinOnce();
    loop_rate.sleep();
    }
    
 

  
  return (0);
}
