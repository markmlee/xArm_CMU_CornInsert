#include "ros/ros.h"
#include "std_msgs/Float64.h"
// #include "amiga_xarm/Patch.h"
#include "amiga_xarm_msgs/Patch.h"
#include "geometry_msgs/Point.h"
#include <xarm_api/xarm_driver.h>

#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>

#include <signal.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

""" 
#######################################################
# Helper class for xArm interface with SDK 

# input: none
# output: none

# author: Dominic Guri  (@andrew.cmu.edu)
# version: 1.0 (05/2023)
#######################################################
""" 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cornstalk_insertion");
  const std::string robot_ip = "192.168.1.213";
  ScrapperController scrapper(robot_ip, FREQUENCY);
  ros::spin();
}