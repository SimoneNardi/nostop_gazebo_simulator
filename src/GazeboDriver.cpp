#include "GazeboDriver.h"

#include <ros/ros.h>

GazeboDriver::GazeboDriver()
{
  
}
  
void GazeboDriver::run()
{
  ros::Rate loop_rate(5);
  
  int count = 0;
  while (ros::ok())
  {
      ros::spinOnce();
      
      loop_rate.sleep();
      ++count;
  }
  
}

bool add(std::string model_name)
  