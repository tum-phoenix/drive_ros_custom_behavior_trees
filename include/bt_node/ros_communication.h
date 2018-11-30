#ifndef ROS_COMMUNICATION_H
#define ROS_COMMUNICATION_H

#include "general.h"

#include "dynamic_reconfigure/server.h"
#include "drive_ros_custom_behavior_trees/BehaviorTreeConfig.h"

class RosInterface {
public:
    RosInterface(ros::NodeHandle &nh);
private:
    dynamic_reconfigure::Server<drive_ros_custom_behavior_trees::BehaviorTreeConfig> dr_server_;
    ros::AsyncSpinner spinner_;
};

#endif
