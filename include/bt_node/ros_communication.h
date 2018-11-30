#ifndef ROS_COMMUNICATION_H
#define ROS_COMMUNICATION_H

#define STANDARD 0
#define STRAIGHT_FORWARD 1
#define TURN_LEFT 2
#define TURN_RIGHT 3
#define SWITCH_TO_LEFT_LANE 4
#define SWITCH_TO_RIGHT_LANE 5

#include "general.h"
#include "drive_ros_custom_behavior_trees/TrajectoryMessage.h"

void setup_ros_communication(ros::NodeHandle *nh);
void publish_trajectory_metadata(drive_ros_custom_behavior_trees::TrajectoryMessage msg);

#endif