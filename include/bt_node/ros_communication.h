#ifndef ROS_COMMUNICATION_H
#define ROS_COMMUNICATION_H

#include "general.h"
#include "drive_ros_custom_behavior_trees/TrajectoryMessage.h"

void setup_ros_communication(ros::NodeHandle *nh);
void publish_trajectory_metadata(drive_ros_custom_behavior_trees::TrajectoryMessage msg);

#endif