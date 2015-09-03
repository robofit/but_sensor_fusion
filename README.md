# but_sensor_fusion


##Package description:

<ul>
  <li> The but_laser_rgbd_fusion package nodes description are: </li>
    <ul>
      <li> filter_hokuyo filters the sonar readings to a specified min and max range. </li>
      <li> laser_rgbd_pc2 converts the data readings to pc and pc2 formats. </li>
      <li> laser_rgbd_registration makes sensor data readings alignment. </li>
      <li> laser_rgbd_fusion fuses both sensor data readings. </li>
    </ul>
  </li>
  <li> It fuses data from the environment which is obtained  by a Hokuyo laser and a RGB-D camera. </li>
  <li> It is tested with an UTM-30LX  and an Asus Live Xtion Pro RGB-D camera. </li>
  <li> The fused readings can be depicted under RVIZ under "Global Options/FixedFrame/laser" option, whrere the fused data readings topic is; "\laser_xtion_sensor" </li>
</ul>


##Package parameters:
<ul>
  <li> The main parameters used in the package are listed below, the package has not been used with different parameters. </li>
  <li> filter_hokuyo </li>
     <ul>
        <li> minimum_range_ specifies the minimun desired laser range, the default value is: 0.25 [m] </li>
        <li> maximum_range_ specifies the maximum desired laser range, the default value is: 10.0 [m] </li>
     </ul>
  <li> laser_rgbd_fusion </li>
     <ul>
        <li> map_resolution_ specifies the resolution of the map, the default value is: 0.05 [m/cell] </li>
        <li>map_x_max_ defines the height of the map, the default value is: 300 [cells] </li>
         <li>  map_y_max_ defines the witdth of the map, the default value is: 300 [cells] </li>
         <li> emp specifies prior probabilistic value of a cell being empty, the default value is: [0.4]  </li>
         <li> occ  specifies prior probabilistic value of a cell being occupied, the default value is: [0.65] </li>
     </ul>
</ul>


##Instalation instructions:
<ul>
  <li> cd ~/ros/catkin_ws/src </li>
  <li> git clone https://github.com/robofit/but_sensor_fusion.git </li>
  <li> cd ../ </li>
  <li> catkin_make </li>
</ul>

##Run simulation:
<ul>
  <li> roslaunch but_laser_rgbd_fusion laser_rgbd.launch
</ul>








