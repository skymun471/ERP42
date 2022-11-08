/*
 * yolo_object_detector_node.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>
#include <darknet_ros/YoloObjectDetector.hpp>
#include "std_msgs/Int16.h"

std_msgs::Int16 go;

int main(int argc, char** argv) {
  ros::init(argc, argv, "darknet_ros");
  ros::NodeHandle nodeHandle("~");
  darknet_ros::YoloObjectDetector yoloObjectDetector(nodeHandle);
  ros::Publisher numberPublisher_= nodeHandle.advertise<std_msgs::Int16>("/darknet_result", 1);

  ros::spin();
  return 0;
}
