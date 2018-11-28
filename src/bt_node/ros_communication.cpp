#include "bt_node/ros_communication.h"

#include "dynamic_reconfigure/server.h"
#include "drive_ros_custom_behavior_trees/BehaviorTreeConfig.h"
#include "drive_ros_custom_behavior_trees/EnvModelMessage.h"
#include "drive_ros_custom_behavior_trees/TrajectoryMessage.h"

extern std::string mode;

void dynamic_reconfigure_callback(drive_ros_custom_behavior_trees::BehaviorTreeConfig &config, uint32_t level) {
    mode = config.mode;
}

void env_model_callback(const drive_ros_custom_behavior_trees::EnvModelMessage &msg) {
    ROS_INFO("Received envmodel message");
}

ros::Subscriber environment_model_subscriber;
ros::Publisher trajectory_publisher;
void setup_ros_communication(ros::NodeHandle *nh) {
    /* Dynamic Reconfigure Setup */
    dynamic_reconfigure::Server<drive_ros_custom_behavior_trees::BehaviorTreeConfig> dr_server;
    dynamic_reconfigure::Server<drive_ros_custom_behavior_trees::BehaviorTreeConfig>::CallbackType dr_callback;
    dr_callback = boost::bind(dynamic_reconfigure_callback, _1, _2);
    dr_server.setCallback(dr_callback);

    /* Topic Subscribers Setup */
    environment_model_subscriber = nh->subscribe("env_model_topic", 4, &env_model_callback);

    /* Topic Publishers Setup */
    trajectory_publisher = nh->advertise<drive_ros_custom_behavior_trees::TrajectoryMessage>("trajectory_metadata", 64);
}

void publish_trajectory_metadata(drive_ros_custom_behavior_trees::TrajectoryMessage msg) {
    trajectory_publisher.publish(msg);
}