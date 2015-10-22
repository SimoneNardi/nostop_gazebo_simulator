#ifndef GAZEBO_SUBSCRIBER_H
#define GAZEBO_SUBSCRIBER_H
#pragma once

#include <ros/ros.h>
#include <memory>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <gazebo_msgs/SetModelState.h>

#include <mutex>

class GazeboSubscriber
{
protected:
  std::string m_model_name;
  
  ros::Subscriber m_updateTwist;
  ros::Subscriber m_updatePose;
    
  gazebo_msgs::SetModelState m_setmodelstate;
  gazebo_msgs::ModelState m_modelstate;
    
  ros::NodeHandle m_node;
  
  ros::ServiceClient m_clientSet;
  
  std::mutex m_mutex;
      
public:
  GazeboSubscriber(std::string model_name_);
 
  void updateTwist(const geometry_msgs::Twist::ConstPtr & twist_);
  
  void updatePose(const geometry_msgs::Pose::ConstPtr &pose_);
};

typedef std::shared_ptr<GazeboSubscriber> GazeboSubscriberPtr;


#endif