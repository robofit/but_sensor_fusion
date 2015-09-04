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
 * This sensor fusion code was inspired by the matlab code implemeted by Petr Štěpán, 
 * Czech Technical University in Prague (CVUT).
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



#include <laser_rgbd_fusion.h>

Fusion_Xtion_Laser::Fusion_Xtion_Laser() : nh_("~"){
  run();   
}

void Fusion_Xtion_Laser::run(){

 /*run the global parameters*/
 Load_Global_Parameters();
 
 scan_xtion_sub_ = nh_.subscribe<sensor_msgs::PointCloud> ("/Cloud_SYN_xtion_msg", 100, &Fusion_Xtion_Laser::Scan_Xtion_Callback, this);  
 scan_laser_sub_ = nh_.subscribe<sensor_msgs::PointCloud>("/Cloud_SYN_laser_msg", 100, &Fusion_Xtion_Laser::Scan_Laser_Callback, this);
 tf_pose_ = nh_.subscribe<nav_msgs::Odometry>("/odometer_pose_pb", 100, &Fusion_Xtion_Laser::tf_Pose,this);
 laser_parameters_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 100, &::Fusion_Xtion_Laser::Laser_Parameters, this);

 laser_map_changes_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/laser_fused_map", 2);
 xtion_map_changes_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/xtion_fused_map", 2);
 xtion_laser_map_changes_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/xtion_laser_fused_map", 2);
 vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "/visualization_marker", 0 );
 xtion_laser_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan_xtion_laser", 100);
}

/*Run the destructor*/
Fusion_Xtion_Laser::~ Fusion_Xtion_Laser(){
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

// this creates and returns a pointer to a MAP structure
 Fusion_Xtion_Laser::MAP *Fusion_Xtion_Laser::make_map(double height, double width, double resolution,double initial){

  MAP* map = (MAP*)calloc(1, sizeof(MAP));
  map->height_ = height;
  map->width_  = width;
  map->resolution_ = resolution_;

  map->cost_ = (double**)calloc(height, sizeof(double*));
  for (int y = 0; y < height; y++){ 
    map->cost_[y] = (double*)calloc(width, sizeof(double)); 
     for(int x = 0; x < width; x++)
      map->cost_[y][x] = initial;
  }

  return map;
}


void Fusion_Xtion_Laser::tf_Pose(nav_msgs::Odometry msg){
 tfX_  =  (int)round((X_OR_ + (int)((msg.pose.pose.position.x)*100)/resolution_));
 tfY_  =  (int)round((Y_OR_ + (int)((msg.pose.pose.position.y)*100)/resolution_));
 tfZ_  =  ((msg.pose.pose.position.z)*100);
}

void Fusion_Xtion_Laser::Laser_Parameters(const sensor_msgs::LaserScan::ConstPtr& msg){
 length = msg->ranges.size();
 inc = msg->angle_increment;

 range_min = 0.25;
 range_max = 10.0;
 angle_min = msg->angle_min;
 angle_max = msg->angle_max;
 time_inc=msg->time_increment;
}

/*rgbd callback function*/
void Fusion_Xtion_Laser::Scan_Xtion_Callback(const sensor_msgs::PointCloud::ConstPtr& msg){
  int xtion_length=msg->points.size();

 for (int j = 0; j <  GR_SIZE_; j++)
   for(int k = 0; k <  GR_SIZE_; k++)
     xtion_map_->cost_[k][j] = 0.5;

 for(int i=0; i<xtion_length;i++){
   dataxx_= (int)round(((msg->points[i].x)*100)/resolution_) + X_OR_;
   datayx_= (int)round(((msg->points[i].y)*100)/resolution_) + Y_OR_;
    
     /*Compute the sensor fusion based on rgbd readings*/
     Xtion_Fusion(X_OR_, Y_OR_, dataxx_, datayx_, GR_SIZE_ );
   }

 //publish rgbd fusion
 publish_xtion_map_changes();
 
}

//write into a file the fused rgbd figure and the fused data
void Fusion_Xtion_Laser::xtion_write(){

 FILE* out;
 sprintf(outputFilename, "/home/acp/patrol_ws/sandbox/fusion_xtion_laser/max_files/xtion.pgm");
 out = fopen(outputFilename, "w");
  if (!out){
    ROS_ERROR("Couldn't open map file to %s", out);
    return;
  }
  
  fprintf(out, "P5");
  fprintf(out, "%d %d ",(int)xtion_map_->height_ ,(int)xtion_map_->width_);
  fprintf(out, "%d ", max_gray);
  for (int i = 0; i < (int)xtion_map_->width_; ++i){
    for (int j = 0; j < (int)xtion_map_->height_; ++j){
      fputc((int)(xtion_map_->cost_[i][j]*255), out);
    }
  }
  fclose(out);

  if((out=fopen("/home/acp/patrol_ws/sandbox/fusion_xtion_laser/max_files/xtion.log","wb"))==NULL){
	printf("\nCant open file\n");
	exit (0);
	      }
       
  for (int j = 0; j < (int)xtion_map_->width_; ++j){
    for (int i = 0; i < (int)xtion_map_->height_; ++i){
       fprintf(out," %f",xtion_map_->cost_[i][j]);
    }
    fprintf(out,"\n");
  }
  fclose(out);
}

/*laser callback function*/
void Fusion_Xtion_Laser::Scan_Laser_Callback(const sensor_msgs::PointCloud::ConstPtr& msg){

 int laser_length=msg->points.size();

 for (int j = 0; j <  GR_SIZE_; j++)
  for(int k = 0; k <  GR_SIZE_; k++)
   laser_map_->cost_[k][j] = 0.5;

 for(int i=0; i<laser_length;i++){
  dataxl_= (int)round(((msg->points[i].x)*100)/resolution_) + X_OR_;
  datayl_= (int)round(((msg->points[i].y)*100)/resolution_) + Y_OR_;
    

  /*Compute the sensor fusion based laser readings*/
  Laser_Fusion(X_OR_, Y_OR_, dataxl_, datayl_, GR_SIZE_ );
   
 }

 //publish laser fusion
 publish_laser_map_changes();
 //xtion_laser_fused();
 
}


//write into a file the fused laser figure and the fused data
void Fusion_Xtion_Laser::laser_write(){

 FILE* out;
 sprintf(outputFilename, "/home/acp/patrol_ws/sandbox/fusion_xtion_laser/max_files/laser.pgm");
 out = fopen(outputFilename, "w");
 
 if (!out){
    ROS_ERROR("Couldn't open map file to %s", out);
    return;
  }
  
 fprintf(out, "P5");
 fprintf(out, "%d %d ",(int)laser_map_->height_ ,(int)laser_map_->width_);
 fprintf(out, "%d ", max_gray);
  for (int i = 0; i < (int)laser_map_->width_; ++i){
   for (int j = 0; j < (int)laser_map_->height_; ++j){
    fputc((int)(laser_map_->cost_[i][j]*255), out);
   }
 }
 fclose(out);
 
 if((out=fopen("/home/acp/patrol_ws/sandbox/fusion_xtion_laser/max_files/laser.log","wb"))==NULL){
  printf("\nCant open file\n");
  exit (0);
  }
       
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


/*rgbd-laser callback function*/
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
    temp_laser_map_->cost_[y][x] = 1;
   else if(temp_laser_map_->cost_[y][x]>0.95)
    temp_laser_map_->cost_[y][x] = 1;
   else if(temp_laser_map_->cost_[y][x]<0.5)
    temp_laser_map_->cost_[y][x] = 0.05;
  }
 }

 for (int y = 0; y < xtion_map_->height_; y++){
  for(int x = 0; x < xtion_map_->width_; x++){
   if(xtion_map_->cost_[y][x]>0.5)
    temp_xtion_map_->cost_[y][x] = 1;
   else if(temp_xtion_map_->cost_[y][x]>0.95)
    temp_xtion_map_->cost_[y][x] = 1;
   else if(temp_xtion_map_->cost_[y][x]<0.5)
    temp_xtion_map_->cost_[y][x] = 0.05;
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
 scan_msg.header.stamp = ros::Time::now(); 

 scan_msg.angle_min =  - angle_min;
 scan_msg.angle_max =  angle_max;
 scan_msg.angle_increment = -inc;
 scan_msg.time_increment =  time_inc;
 scan_msg.range_min = range_min; 
 scan_msg.range_max = range_max;
 scan_msg.ranges.resize(length);

 for(int i=1;i<(length-3);i++){
  laser_inc =  i*inc;
  orientationvector[0] = -sin(-laser_inc);
  orientationvector[1] =  cos(-laser_inc);
   while(wallHit==0){  
    beamCoorInt[0] = (int)round(beamCoor[0]);
    beamCoorInt[1] = (int)round(beamCoor[1]);
   
     if((beamCoorInt[0] >= 299)  || (beamCoorInt[1] >= 299))
      wallHit = 1;
     if((xtion_laser_map_->cost_[beamCoorInt[0]][beamCoorInt[1]]) > 0.9){
      wallHit = 1;
      double distance = sqrt(pow((150-beamCoorInt[0]),2) + pow((150-beamCoorInt[1]),2)); 
      value_ = (distance/5)*0.25;
       if(range_min <= value_ && value_ <=  range_max)
        scan_msg.ranges[i] =  value_;
     }  
 beamCoor[0] = beamCoor[0] + orientationvector[0];
 beamCoor[1] = beamCoor[1] + orientationvector[1]; 
 }
    
   wallHit = 0;
   beamCoor[0]=Xpart_x;
   beamCoor[1]=Xpart_y;
}

 xtion_laser_scan_pub_.publish(scan_msg);
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

 msg.info.width  = GR_SIZE_;      
 msg.info.height = GR_SIZE_;      
 int mapsize   =  msg.info.width * msg.info.height; 
 msg.info.resolution = map_resolution_; 
 msg.info.origin.position.x = origin_map_x_;  
 msg.info.origin.position.y = origin_map_y_ ;
 msg.data.resize(GR_SIZE_ * GR_SIZE_);
   
 int k=0; 
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
  
 xtion_laser_map_changes_pub_.publish(msg);

}

/*---------------------- ROS Publisher's Laser Function ------------------------*/
void Fusion_Xtion_Laser::publish_laser_map_changes(){
 
 nav_msgs::OccupancyGrid  msg;
 msg.header.frame_id = "/laser";
 msg.header.stamp = ros::Time::now();
 msg.info.width  = GR_SIZE_;       
 msg.info.height = GR_SIZE_;     
 int mapsize   =  msg.info.width * msg.info.height; 
 msg.info.resolution = map_resolution_; 
 msg.info.origin.position.x = origin_map_x_;  
 msg.info.origin.position.y = origin_map_y_ ;
 msg.data.resize(GR_SIZE_ * GR_SIZE_);
 
 int k=0;
 for (int j = 0; j <  GR_SIZE_; j++)
   for(int i = 0; i <  GR_SIZE_; i++){
     if(laser_map_->cost_[i][j] == 0.5)
       msg.data[k] = -1;
     else  msg.data[k] = 100-((laser_map_->cost_[i][j])*100);  
      k++;  
   }

  laser_map_changes_pub_.publish(msg);
}


/*---------------------- ROS Publisher's Laser Function ------------------------*/
void Fusion_Xtion_Laser::publish_xtion_map_changes(){
 nav_msgs::OccupancyGrid  msg;
 msg.header.frame_id = "/laser";
 msg.header.stamp = ros::Time::now();

 msg.info.width  = GR_SIZE_;       
 msg.info.height = GR_SIZE_;      
 int mapsize   =  msg.info.width * msg.info.height; 
 msg.info.resolution = map_resolution_; 
 msg.info.origin.position.x = origin_map_x_;  
 msg.info.origin.position.y = origin_map_y_;
 msg.data.resize(GR_SIZE_ * GR_SIZE_);

 int k=0;
 for (int j = 0; j <  GR_SIZE_; j++)
  for(int i = 0; i <  GR_SIZE_; i++){
   if(xtion_map_->cost_[i][j] == 0.5)
    msg.data[k] = -1;
   else
    msg.data[k]=100-((xtion_map_->cost_[i][j])*100);   
    k++;  
   }

xtion_map_changes_pub_.publish(msg);

}




/*---------------------- ROS laser's Fusion Function ------------------------*/
void  Fusion_Xtion_Laser::Laser_Fusion(int c_x_, int c_y_, int dataxl_, int datayl_, int GR_SIZE){
 
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





/*---------------------- ROS  Xtion's Fusion Function ------------------------*/
void Fusion_Xtion_Laser::Xtion_Fusion(int c_x_, int c_y_, int dataxl_, int datayl_, int GR_SIZE){

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



int main(int argc, char** argv){
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
