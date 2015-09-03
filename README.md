# but_sensor_fusion


##Package description:

<ul>
  <li> It fuses data from the environment which is obtained  by a Hokuyo laser and a RGB-D camera. </li>
  <li> It is tested with an UTM-30LX  and an Asus Live Xtion Pro RGB-D camera. </li>
  <li> The fused readings can be depicted under RVIZ in "Global Options/FixedFrame/laser", whrere the fused data readings topic is "\laser_xtion_sensor" </li>
  <li> Nodes package description </li>
    <ul>
      <li> filter_hokuyo filters the sonar readings to a specified min and max range. </li>
      <li> laser_rgbd_pc2 converts the data readings to pc and pc2. </li>
      <li> laser_rgbd_registration makes sensor data readings alignment. </li>
      <li> laser_rgbd_fusion fuses both sensor data readings. </li>
    </ul>
  </li>
  <li>third item</li>
</ul>

##Instalation instructions:

-cd ~/ros/catkin_ws/src

-git clone https://github.com/robofit/but_sensor_fusion.git

-cd ../

-catkin_make


##run simulation:

-roslaunch lunch laser_rgbd.launch









