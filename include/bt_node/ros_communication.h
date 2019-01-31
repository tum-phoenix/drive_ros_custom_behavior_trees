#ifndef ROS_COMMUNICATION_H
#define ROS_COMMUNICATION_H


#include "dynamic_reconfigure/server.h"
#include "drive_ros_custom_behavior_trees/BehaviorTreeConfig.h"

#include "general.h"
#include "drive_ros_msgs/TrajectoryMetaInput.h"

void setup_ros_communication(ros::NodeHandle *nh);
void publish_trajectory_metadata(drive_ros_msgs::TrajectoryMetaInput msg);
std::string get_driving_mode();

class RosInterface {
public:
    RosInterface(ros::NodeHandle &nh);
private:
    dynamic_reconfigure::Server<drive_ros_custom_behavior_trees::BehaviorTreeConfig> dr_server_;
    ros::AsyncSpinner spinner_;
};

#endif
