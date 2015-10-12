#include "GazeboDriver.h"

#include <ros/ros.h>

#include <iostream>
#include <string>
#include <fstream>

#include <gazebo_msgs/DeleteModel.h>

///
GazeboDriver::~GazeboDriver()
{
    for(size_t i = 0; i < m_model_name.size(); ++i)
    {
      this->remove(m_model_name[i]);
    }
}

///
GazeboDriver::GazeboDriver(std::string urdf_model_filename) 
: m_node()
{
  m_clientSpawn = m_node.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  
  std::ifstream inFile (urdf_model_filename.c_str());
  if (inFile.is_open())
  {
    std::stringstream buffer;
    buffer << inFile.rdbuf();
    std::string str = buffer.str();
    m_spawnModel.request.model_xml = str.c_str();
    ROS_DEBUG("%s",str.c_str());
  }
  
  m_clientDelete = m_node.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
}

///
bool GazeboDriver::add(std::string model_name_, const geometry_msgs::Pose::ConstPtr &pose_ )
{
    m_spawnModel.request.model_name = model_name_.c_str();
    m_spawnModel.request.initial_pose.orientation = pose_->orientation;
    m_spawnModel.request.initial_pose.position = pose_->position;
      
    if (m_clientSpawn.call(m_spawnModel))
    {
      if( m_spawnModel.response.success )
      {
	ROS_DEBUG("Added %s to simulation", model_name_.c_str());

	m_hash.insert( make_pair( model_name_, m_model_name.size() ) );
	m_model_name.push_back(model_name_);
	m_updater.push_back( GazeboSubscriber(model_name_) );
		
	return true;
      }
      else
      {
	ROS_DEBUG("Unable to add %s to simulation!", model_name_.c_str());
	return false;
      }
    }
    else
    {
      ROS_ERROR("Failed to call service!");
      return false;
    }
}
  
///
bool GazeboDriver::remove(std::string model_name_)
{
    gazebo_msgs::DeleteModel l_model;
    l_model.request.model_name = model_name_;

    if (m_clientDelete.call(l_model))
    {
      if( l_model.response.success )
      {
	ROS_DEBUG("Removed %s from simulation", model_name_.c_str());

	m_model_name.erase( m_model_name.begin() + m_hash[model_name_] );
	m_updater.erase( m_updater.begin() + m_hash[model_name_] );
	m_hash.erase(model_name_);
	
	return true;
      }
      else
      {
	ROS_DEBUG("Unable to remove %s from simulation!", model_name_.c_str());
	return false;
      }
    }
    else
    {
      ROS_ERROR("Failed to call service!");
      return false;
    }
}