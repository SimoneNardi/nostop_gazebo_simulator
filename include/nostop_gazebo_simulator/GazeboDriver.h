#ifndef GAZEBO_DRIVER_H
#define GAZEBO_DRIVER_H
#pragma once

#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>

#include <geometry_msgs/Pose.h>

#include <memory>

#include "GazeboSubscriber.h"

#include <map>
#include <vector>

class GazeboDriver
{
protected:
  std::vector<std::string> m_model_name;
  std::vector<GazeboSubscriber> m_updater;
  std::map<std::string, size_t> m_hash;
  
  ros::NodeHandle m_node;
  ros::ServiceClient m_clientSpawn;
  ros::ServiceClient m_clientDelete;
  
  gazebo_msgs::SpawnModel m_spawnModel;
  
public:
  GazeboDriver(std::string urdf_model_filename = "/home/simone/catkin_ws/src/nostop/nostop_robot/description/urdf/model.urdf");
  
  ~GazeboDriver();
   
  bool add(std::string model_name, const geometry_msgs::Pose::ConstPtr &pose);
  
  bool remove(std::string model_name);
};


#endif