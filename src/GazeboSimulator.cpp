#include <ros/ros.h>

#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SpawnModel.h>

#include <iostream>
#include <string>
#include <fstream>

#include "GazeboDriver.h"

#include <memory>

#include <geometry_msgs/Pose.h>


//~

int main (int argc, char** argv)
{
  ros::init(argc,argv,"gazebo_simulator");
  ros::NodeHandle n;
  
  GazeboDriver l_gazebo_driver;
  
  geometry_msgs::Pose::Ptr l_initial = boost::make_shared<geometry_msgs::Pose>();
  
  double theta = 0.3;
  
  l_initial->position.x = 2 ;
  l_initial->orientation.z = sin(theta/2);
  l_initial->orientation.w = cos(theta/2);
  
  l_gazebo_driver.addGuard("test", l_initial);
 
  theta = 0.5;
  l_initial->position.y = -1;
  l_initial->orientation.z = sin(theta/2);
  l_initial->orientation.w = cos(theta/2);
    
  l_gazebo_driver.addThief("test2", l_initial);
  
  theta = -0.8;
  l_initial->position.x = 1;
  l_initial->position.y = 1;
  
  l_initial->orientation.z = sin(theta/2);
  l_initial->orientation.w = cos(theta/2);
  
  l_gazebo_driver.addGuard("test3", l_initial);
  
  
  l_gazebo_driver.remove("test");
  l_gazebo_driver.remove("test2");
  l_gazebo_driver.remove("test3");
  
  ros::spin();
  
  //l_gazebo_driver.remove("test2");
 
}