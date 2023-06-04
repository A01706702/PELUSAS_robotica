<?xml version="1.0" encoding="UTF-8"?>
<launch>
 
  <!-- Set up env variable so plugin and textures are found -->
  <env name="GAZEBO_PLUGIN_PATH" value="$(find puzzlebot_world)/plugins"/> 
  <env name="GAZEBO_MODEL_PATH" value="$(find puzzlebot_world)/models"/>
  <env name="GAZEBO_RESOURCE_PATH" value="$(find puzzlebot_world)/models" />

 <!-- these are the arguments you can pass this launch file, for example paused:=true -->
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find puzzlebot_sim)/worlds/final_challenge_world_arucos4.world"/>
    <!-- <arg name="world_name" value="$(find puzzlebot_world)/worlds/final_challenge_world.world"/>  -->
    <!-- <arg name="world_name" value="$(find puzzlebot_world)/worlds/obstacle.world"/> -->
    <arg name="debug" value="$(arg debug)" />
    <arg name="gui" value="$(arg gui)" />
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="headless" value="$(arg headless)"/>
  </include>
  
  <!--load robot description -->
   <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find puzzlebot_gazebo)/urdf/puzzlebot.xacro'" />

    <arg name="x" default="0.00"/>
    <arg name="y" default="0.00"/>
    <!--! SPAWN ROBOT ABOVE GROUND -->
    <arg name="z" default="0.10"/> <!--? Value to modify, originally 0.0 -->
    <arg name="Y" default="-1.57"/>
    
  <node name="puzzlebot_spawn" pkg="gazebo_ros" type="spawn_model"  output="screen"
          args="-urdf -param robot_description -model puzzlebot -x $(arg x) -y $(arg y) -z $(arg z) -Y $(arg Y)" />

  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen">
  </node>

  <node name="joint_state_publisher"  pkg="joint_state_publisher" type="joint_state_publisher"> 
  </node>

  <node name="puzzlebot_controller" pkg="puzzlebot_control" type="puzzlebot_control_node"/>

  <node name="transform_lidar_data" pkg="puzzlebot_sim" type="transform_lidar_data.py"/>

  <!-- <node name="camera_aruco_final" pkg="aruco_markers_tec" type="camera_aruco_final.py"/> -->

  <node name="rqt_image_view" pkg="rqt_image_view" type="rqt_image_view"/>

<include file="$(find aruco_markers_tec)/launch/aruco_detect_sim.launch"/>"

<include file="$(find puzzlebot_control)/launch/puzzlebot_control.launch"/>"

</launch>

  
