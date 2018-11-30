#include "bt_node/ros_communication.h"

extern std::string mode;

void dynamic_reconfigure_callback(drive_ros_custom_behavior_trees::BehaviorTreeConfig &config, uint32_t level) {
    mode = config.mode;
}

RosInterface::RosInterface(ros::NodeHandle &nh) : dr_server_(), spinner_(1) {
    dynamic_reconfigure::Server<drive_ros_custom_behavior_trees::BehaviorTreeConfig>::CallbackType dr_callback;
    dr_callback = boost::bind(dynamic_reconfigure_callback, _1, _2);
    dr_server_.setCallback(dr_callback);
    spinner_.start();
}
