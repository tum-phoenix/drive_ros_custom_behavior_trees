#include "bt_node/ros_communication.h"

#include "dynamic_reconfigure/server.h"
#include "drive_ros_custom_behavior_trees/BehaviorTreeConfig.h"
#include "drive_ros_msgs/TrajectoryMetaInput.h"
#include "bt_node/environment_model.h"

extern float min_sign_react_distance;
extern float max_sign_react_distance;
extern std::string mode;

extern float current_velocity;

void dynamic_reconfigure_callback(drive_ros_custom_behavior_trees::BehaviorTreeConfig &config, uint32_t level) {
    mode = config.mode;
}

void car_data_callback(/* const drive_ros_msgs::Message &msg */) {
    //current_velocity = msg.velocity;
}

ros::Subscriber environment_model_subscriber;
ros::Subscriber car_data_subscriber;
ros::Publisher trajectory_publisher;
void setup_ros_communication(ros::NodeHandle *nh) {
    /* Dynamic Reconfigure Setup */
    dynamic_reconfigure::Server<drive_ros_custom_behavior_trees::BehaviorTreeConfig> dr_server;
    dynamic_reconfigure::Server<drive_ros_custom_behavior_trees::BehaviorTreeConfig>::CallbackType dr_callback;
    dr_callback = boost::bind(dynamic_reconfigure_callback, _1, _2);
    dr_server.setCallback(dr_callback);

    /* Topic Subscribers Setup */
    environment_model_subscriber = nh->subscribe("env_out", 4, &EnvModel::subscriber_callback);
    //car_data_subscriber = nh->subscribe("", 4, &car_data_callback); //TODO car data topic name

    /* Topic Publishers Setup */
    trajectory_publisher = nh->advertise<drive_ros_msgs::TrajectoryMetaInput>("trajectory_metadata", 64);
}

void publish_trajectory_metadata(drive_ros_msgs::TrajectoryMetaInput msg) {
    trajectory_publisher.publish(msg);
}


RosInterface::RosInterface(ros::NodeHandle &nh) : dr_server_(), spinner_(1) {
    dynamic_reconfigure::Server<drive_ros_custom_behavior_trees::BehaviorTreeConfig>::CallbackType dr_callback;
    dr_callback = boost::bind(dynamic_reconfigure_callback, _1, _2);
    dr_server_.setCallback(dr_callback);
    spinner_.start();
}
