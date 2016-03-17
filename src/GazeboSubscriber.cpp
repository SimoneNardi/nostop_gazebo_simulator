#include "GazeboSubscriber.h"
#include "GazeboDriver.h"

#include <gazebo_msgs/SetModelState.h>

#include <iostream>
#include <string>
#include <fstream>

#include <condition_variable>

///
GazeboSubscriber::GazeboSubscriber(std::string model_name_)
 : m_node()
{
  m_model_name = model_name_;
  
  std::string l_twistTopic = "/";
  l_twistTopic += model_name_;
  l_twistTopic += "/cmd_vel";
  //m_updateTwist = m_node.subscribe<geometry_msgs::Twist>(l_twistTopic.c_str(), 1, &GazeboSubscriber::updateTwist, this);
  
  std::string l_poseTopic = "/";
  l_poseTopic += model_name_;
  l_poseTopic += "/pose";
  m_updatePose = m_node.subscribe<geometry_msgs::Pose>(l_poseTopic.c_str(), 10, &GazeboSubscriber::updatePose, this);
  
  m_modelstate.model_name = m_model_name;
  m_modelstate.reference_frame = "map";
  m_setmodelstate.request.model_state = m_modelstate;
  
  m_clientSet = m_node.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); 
  if (!m_clientSet.exists())
  {
      m_clientSet.waitForExistence(ros::Duration(-1));
  }
}

///
void GazeboSubscriber::updateTwist(const geometry_msgs::Twist::ConstPtr &twist_)
{
  std::unique_lock<std::mutex> l_lock(m_mutex);
  
  m_setmodelstate.request.model_state.twist.angular = twist_->angular;
  m_setmodelstate.request.model_state.twist.linear = twist_->linear;
  
  if ( m_clientSet.exists() && m_clientSet.call(m_setmodelstate) )
  {
    if (m_setmodelstate.response.success)
    {
      ROS_DEBUG("%s: Twist State updated!", m_model_name.c_str());
      return;
    }
    else
    {
      ROS_DEBUG("%s: Twist State UNABLE to Update!", m_model_name.c_str());
      return;
    }
  }
  else
  {
    ROS_ERROR("Failed to call service: GazeboSubscriber::updateTwist!");
    return;
  }
}

///
void GazeboSubscriber::updatePose(const geometry_msgs::Pose::ConstPtr &pose_)
{
  std::unique_lock<std::mutex> l_lock(m_mutex);
  
  m_setmodelstate.request.model_state.pose.orientation = pose_->orientation;
  m_setmodelstate.request.model_state.pose.position = pose_->position;
  
  if (m_clientSet.exists() && m_clientSet.call(m_setmodelstate))
  {
    if (m_setmodelstate.response.success)
    {
      ROS_DEBUG("%s: Pose State updated!", m_model_name.c_str());
      return;
    }
    else
    {
      ROS_DEBUG("%s: Pose State UNABLE to Update!", m_model_name.c_str());
      return;
    }
  }
  else
  {
    ROS_ERROR("Failed to call service: GazeboSubscriber::updatePose!");
    return;
  }
}