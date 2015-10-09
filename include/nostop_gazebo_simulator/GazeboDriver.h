#ifndef GAZEBO_DRIVER_H
#define GAZEBO_DRIVER_H
#pragma once

#include "ThreadBase.h"

#include <memory>

class GazeboDriver : public Robotics::GameTheory::ThreadBase
{
protected:
  std::vector<std::string> m_call;
  
public:
  GazeboDriver();
  
  virtual void run() = 0;
  
  bool add(std::string model_name);
};


#endif