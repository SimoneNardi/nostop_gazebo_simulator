#ifndef GAZEBO_DRIVER_H
#define GAZEBO_DRIVER_H
#pragma once

#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>

#include <geometry_msgs/Pose.h>

#include <memory>

#include <map>
#include <vector>

#include "GazeboSubscriber.h"

class GazeboDriver
{
public:
  std::vector<std::string> m_model_name;
  std::vector<GazeboSubscriberPtr> m_updater;
  std::map<std::string, size_t> m_hash;
  
  ros::NodeHandle m_node;
  ros::ServiceClient m_clientSpawn;
  ros::ServiceClient m_clientDelete;
  
  gazebo_msgs::SpawnModel m_spawnModel;
  std::string m_urdf_pathname;
  
public:
  GazeboDriver(std::string urdf_model_filename = "/home/simone/catkin_ws/src/nostop/nostop_gazebo_simulator/model/");
  
  ~GazeboDriver();
   
  bool addGuard(std::string const& model_name, const geometry_msgs::Pose::ConstPtr &pose, int const& type = 2);
  bool addThief(std::string const& model_name, const geometry_msgs::Pose::ConstPtr &pose);
  
  bool remove(std::string const& model_name);
};


#endif