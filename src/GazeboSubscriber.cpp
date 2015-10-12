#include "GazeboSubscriber.h"

#include <gazebo_msgs/SetModelState.h>

#include <iostream>
#include <string>
#include <fstream>

///
GazeboSubscriber::GazeboSubscriber(std::string model_name_)
 : m_node()
{
  m_model_name = model_name_;
  
  std::string l_twistTopic = "MotorControl_";
  l_twistTopic += model_name_;
  
  m_updateTwist = m_node.subscribe<geometry_msgs::Twist>(l_twistTopic.c_str(), 10, &GazeboSubscriber::updateTwist, this);
  
  std::string l_poseTopic = "Pose_";
  l_poseTopic += model_name_;
  m_updatePose = m_node.subscribe<geometry_msgs::Pose>(l_poseTopic.c_str(), 10, &GazeboSubscriber::updatePose, this);
  
  gazebo_msgs::ModelState l_modelstate;
  l_modelstate.model_name = m_model_name;
  m_setmodelstate.request.model_state = l_modelstate;
  
  m_clientSet = m_node.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); 
}

///
void GazeboSubscriber::updateTwist(const geometry_msgs::Twist::ConstPtr &twist_)
{
  m_setmodelstate.request.model_state.twist.angular = twist_->angular;
  m_setmodelstate.request.model_state.twist.linear = twist_->linear;
  
  if (m_clientSet.call(m_setmodelstate))
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
    ROS_ERROR("Failed to call service!");
    return;
  }
}

///
void GazeboSubscriber::updatePose(const geometry_msgs::Pose::ConstPtr &pose_)
{
  m_setmodelstate.request.model_state.pose.orientation = pose_->orientation;
  m_setmodelstate.request.model_state.pose.position = pose_->position;
  
  if (m_clientSet.call(m_setmodelstate))
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
    ROS_ERROR("Failed to call service!");
    return;
  }
}