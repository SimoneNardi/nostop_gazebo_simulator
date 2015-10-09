#include <ros/ros.h>

#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SpawnModel.h>

#include <iostream>
#include <string>
#include <fstream>

#include "GazeboDriver.h"

#include <memory>

//~

int main (int argc, char** argv)
{
  ros::init(argc,argv,"gazebo_simulator");
  ros::NodeHandle n;
  
  std::shared_ptr<GazeboDriver> l_gazebo_driver;
  l_gazebo_driver->start();
  
  ros::spin();
 
}