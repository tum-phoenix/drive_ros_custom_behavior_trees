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

<<<<<<< HEAD
void setup_ros_communication(ros::NodeHandle *nh);
void publish_trajectory_metadata(drive_ros_custom_behavior_trees::TrajectoryMessage msg);
=======
#include "dynamic_reconfigure/server.h"
#include "drive_ros_custom_behavior_trees/BehaviorTreeConfig.h"
>>>>>>> 28a26d79e654a5a4857df649dd8f738bd725383a

class RosInterface {
public:
    RosInterface(ros::NodeHandle &nh);
private:
    dynamic_reconfigure::Server<drive_ros_custom_behavior_trees::BehaviorTreeConfig> dr_server_;
    ros::AsyncSpinner spinner_;
};

#endif
