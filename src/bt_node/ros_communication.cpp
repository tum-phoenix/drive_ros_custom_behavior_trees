#include "bt_lib/tree.h"

#include "bt_node/ros_communication.h"
#include "bt_node/environment_model.h"
#include "bt_node/node_reset.h"

#include "dynamic_reconfigure/server.h"
#include "drive_ros_custom_behavior_trees/BehaviorTreeConfig.h"
#include "drive_ros_msgs/TrajectoryMetaInput.h"
#include "drive_ros_msgs/ParkingInProgress.h"
#include "drive_ros_uavcan/phoenix_msgs__DriveState.h"
#include "drive_ros_uavcan/phoenix_msgs__UserButtons.h"
#include "drive_ros_uavcan/phoenix_msgs__StartParking.h"

extern std::string mode;
extern bool dynamic_reconfigure_overwrite_runtime_vals;
extern int tick_freq_ms;
extern int intersection_turn_duration;
extern float general_max_speed;
extern float general_max_speed_cautious;
extern float max_bridge_speed;
extern float parking_spot_search_speed;
extern float max_lane_switch_speed;
extern float sharp_turn_speed;
extern float very_sharp_turn_speed;
extern float overtake_distance;
extern float object_following_break_factor;
extern float universal_break_factor;
extern float barred_area_react_distance;
extern float oncoming_traffic_clearance;
extern float max_start_box_distance;
extern float intersection_turn_speed;
extern float break_distance_safety_factor;
extern float intersection_max_obj_distance;
extern float speed_zero_tolerance;

extern bool overtaking_forbidden_zone;
extern bool express_way;
extern bool priority_road;
extern bool force_stop;
extern bool on_bridge;
extern bool give_way;
extern int speed_limit;
extern int successful_parking_count;
extern int intersection_turn_indication;
extern float current_velocity;

extern BT::Tree *tree;

ros::Subscriber environment_model_subscriber;
ros::Subscriber car_data_subscriber;
ros::Subscriber button_subscriber;
ros::Publisher trajectory_publisher;
ros::Publisher parking_publisher;
ros::ServiceClient parking_finished_client;

void dynamic_reconfigure_callback(drive_ros_custom_behavior_trees::BehaviorTreeConfig &config, uint32_t level) {
    if(!config.mode.compare("NONE")) mode = config.mode;
    if(config.tick_freq_ms != -1000) tick_freq_ms = config.tick_freq_ms;
    if(config.general_max_speed != -1000) general_max_speed = config.general_max_speed;
    if(config.general_max_speed_cautious != -1000) general_max_speed_cautious = config.general_max_speed_cautious;
    if(config.max_bridge_speed != -1000) max_bridge_speed = config.max_bridge_speed;
    if(config.parking_spot_search_speed != -1000) parking_spot_search_speed = config.parking_spot_search_speed;
    if(config.max_lane_switch_speed != -1000) max_lane_switch_speed = config.max_lane_switch_speed;
    if(config.sharp_turn_speed != -1000) sharp_turn_speed = config.sharp_turn_speed;
    if(config.very_sharp_turn_speed != -1000) very_sharp_turn_speed = config.very_sharp_turn_speed;
    if(config.overtake_distance != -1000) overtake_distance = config.overtake_distance;
    if(config.object_following_break_factor != -1000) object_following_break_factor = config.object_following_break_factor;
    if(config.universal_break_factor != -1000) universal_break_factor = config.universal_break_factor;
    if(config.barred_area_react_distance != -1000) barred_area_react_distance = config.barred_area_react_distance;
    if(config.oncoming_traffic_clearance != -1000) oncoming_traffic_clearance = config.oncoming_traffic_clearance;
    if(config.max_start_box_distance != -1000) max_start_box_distance = config.max_start_box_distance;
    if(config.intersection_turn_speed != -1000) intersection_turn_speed = config.intersection_turn_speed;
    if(config.break_distance_safety_factor != -1000) break_distance_safety_factor = config.break_distance_safety_factor;
    if(config.intersection_max_obj_distance != -1000) intersection_max_obj_distance = config.intersection_max_obj_distance;
    if(config.intersection_turn_duration != 0) intersection_turn_duration = config.intersection_turn_duration;
    if(config.speed_zero_tolerance != -1000) speed_zero_tolerance = config.speed_zero_tolerance; 

    if(dynamic_reconfigure_overwrite_runtime_vals) {
        overtaking_forbidden_zone = config.overtaking_forbidden_zone;
        express_way = config.express_way;
        priority_road = config.priority_road;
        force_stop = config.force_stop;
        on_bridge = config.on_bridge;
        give_way = config.give_way;
        if(config.successful_parking_count != -1000) successful_parking_count = config.successful_parking_count;
        if(config.intersection_turn_indication != -1000) intersection_turn_indication = config.intersection_turn_indication;
        if(config.speed_limit != -1000) speed_limit = config.speed_limit;
    }
}

bool previously_armed = false;
void car_data_callback(const drive_ros_uavcan::phoenix_msgs__DriveState &msg) {
    current_velocity = msg.v;
    if(previously_armed && !msg.arm) reset_tree_state(tree);
    previously_armed = msg.arm;
}

int user_button_state = 0;
void user_button_callback(const drive_ros_uavcan::phoenix_msgs__UserButtons &msg) {
    user_button_state = msg.bit_but;
}

drive_ros_msgs::ParkingInProgress pip_msg;
bool parking_finished() {
    if(parking_finished_client.call(pip_msg)) {
        return !pip_msg.response.parking_in_progress;
    }
}

std::string get_driving_mode() {
    ros::Rate r(10);
    for(;;) {
        drive_ros_msgs::TrajectoryMetaInput msg;
        msg.control_metadata = drive_ros_msgs::TrajectoryMetaInput::STRAIGHT_FORWARD;
        msg.max_speed = 0;
        publish_trajectory_metadata(msg);
        //The user buttons are one-hot-encoded; least significant bit being the leftmost button.
        if(user_button_state & 1) return "OBSTACLES";
        if(user_button_state & 2) return "PARKING";
        r.sleep();
    }
}

void setup_ros_communication(ros::NodeHandle *nh) {
    /* Dynamic Reconfigure Setup */
    dynamic_reconfigure::Server<drive_ros_custom_behavior_trees::BehaviorTreeConfig> dr_server;
    dynamic_reconfigure::Server<drive_ros_custom_behavior_trees::BehaviorTreeConfig>::CallbackType dr_callback;
    dr_callback = boost::bind(dynamic_reconfigure_callback, _1, _2);
    dr_server.setCallback(dr_callback);

    /* Topic Subscribers Setup */
    environment_model_subscriber = nh->subscribe("env_in", 4, &EnvModel::subscriber_callback);
    car_data_subscriber = nh->subscribe("drive_state_in", 4, &car_data_callback);
    button_subscriber = nh->subscribe("buttons", 1, &user_button_callback);

    /* Topic Publishers Setup */
    trajectory_publisher = nh->advertise<drive_ros_msgs::TrajectoryMetaInput>("trajectory_metadata", 64);
    parking_publisher = nh->advertise<drive_ros_uavcan::phoenix_msgs__StartParking>("start_parking", 10);

    /* Services setup */
    parking_finished_client = nh->serviceClient<drive_ros_msgs::ParkingInProgress>("parking_finished");
}

void publish_trajectory_metadata(drive_ros_msgs::TrajectoryMetaInput msg) {
    trajectory_publisher.publish(msg);
}

void publish_parking() {
    drive_ros_uavcan::phoenix_msgs__StartParking msg;
    msg.start_flag = 1;
    parking_publisher.publish(msg);
}


RosInterface::RosInterface(ros::NodeHandle &nh) : dr_server_(), spinner_(1) {
    dynamic_reconfigure::Server<drive_ros_custom_behavior_trees::BehaviorTreeConfig>::CallbackType dr_callback;
    dr_callback = boost::bind(dynamic_reconfigure_callback, _1, _2);
    dr_server_.setCallback(dr_callback);
    spinner_.start();
}
