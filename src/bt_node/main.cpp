#include "general.h"
#include <vector>
#include <boost/algorithm/string.hpp>

#include "bt_node/ros_communication.h"
#include "bt_node/environment_model.h"
#include "bt_lib/tree.h"
#include "bt_lib/sequence_node.h"
#include "bt_lib/parallel_node.h"
#include "bt_lib/action_node.h"

#include "bt_node/nodes.h"


/* ---------- GLOBAL DATA ---------- */
//Fixed parameters
std::string mode;
bool output_show_computation_time;
bool dynamic_reconfigure_overwrite_runtime_vals;
int tick_frequency;
int tick_freq_ms;
int intersection_turn_duration;
float general_max_speed;
float general_max_speed_cautious;
float min_sign_react_distance;
float max_sign_react_distance;
float max_bridge_speed;
float parking_spot_search_speed;
float max_lane_switch_speed;
float sharp_turn_speed;
float very_sharp_turn_speed;
float overtake_distance;
float object_following_break_factor;
float universal_break_factor;
float barred_area_react_distance;
float oncoming_traffic_clearance;
float max_start_box_distance;
float intersection_turn_speed;
float break_distance_safety_factor;
float intersection_max_obj_distance;
float speed_zero_tolerance;

//Dynamic values
bool overtaking_forbidden_zone;
bool express_way;
bool priority_road;
bool force_stop;
bool on_bridge;
bool give_way;
int successful_parking_count;
int intersection_turn_indication;
float speed_limit;
float current_velocity = 0;

//Miscellaneous
bool clean_output;
bool started_by_button = false;
BT::Tree *tree;

/* ---------- END OF GLOBAL DATA ---------- */

std::set<std::string> *initial_states;

void read_launch_file(ros::NodeHandle *nh);
void check_parameter_values();
void construct_parkingmode_tree(BT::SequenceNode *head);
void construct_obstaclesmode_tree(BT::SequenceNode *head);


int main(int argc, char **argv) {
    ros::init(argc, argv, "BehaviorTree");
    ros::NodeHandle nh;
    setup_ros_communication(&nh);
    read_launch_file(&nh);
    check_parameter_values();
    RosInterface ros_interface(nh);
    
    if(mode.length() == 0) {
        ROS_INFO("Waiting for button input specifying the driving mode");
        mode = get_driving_mode();
        started_by_button = true;
    }

    //Try to construct the tree
    BT::SequenceNode *head = new BT::SequenceNode("CaroloCup2019", false, true);
    ROS_INFO("Creating Behavior Tree for %s mode", mode.c_str());
    if(!mode.compare("PARKING")) construct_parkingmode_tree(head);
    else if(!mode.compare("OBSTACLES")) construct_obstaclesmode_tree(head);
    else {
        ROS_ERROR("Driving mode not properly declared. Please check launch file");
        return -1;
    }

    //Create the tree instance
    tree = new BT::Tree(head, tick_freq_ms, clean_output);
    //If start states have been set in the launch file, apply them
    if(!started_by_button && initial_states->size() > 0) {
        ROS_INFO_STREAM("" << initial_states->size() << " start state(s) manually set, applying...");
        tree->reset_state(initial_states);
    }
    //Fire up the tree
    tree->execute();
}

void read_launch_file(ros::NodeHandle *nh) {
    nh->getParam("behavior_tree/mode", mode);
    nh->getParam("behavior_tree/clean_output", clean_output);
    nh->getParam("behavior_tree/output_show_computation_time", output_show_computation_time);
    nh->getParam("behavior_tree/dynamic_reconfigure_overwrite_runtime_vals", dynamic_reconfigure_overwrite_runtime_vals);
    nh->getParam("behavior_tree/break_distance_safety_factor", break_distance_safety_factor);
    nh->getParam("behavior_tree/tick_frequency", tick_frequency);
    nh->getParam("behavior_tree/object_following_break_factor", object_following_break_factor);
    nh->getParam("behavior_tree/intersection_turn_duration", intersection_turn_duration);

    nh->getParam("behavior_tree/general_max_speed", general_max_speed);
    nh->getParam("behavior_tree/general_max_speed_cautious", general_max_speed_cautious);
    nh->getParam("behavior_tree/intersection_turn_speed", intersection_turn_speed);
    nh->getParam("behavior_tree/max_bridge_speed", max_bridge_speed);
    nh->getParam("behavior_tree/parking_spot_search_speed", parking_spot_search_speed);
    nh->getParam("behavior_tree/max_lane_switch_speed", max_lane_switch_speed);
    nh->getParam("behavior_tree/sharp_turn_speed", sharp_turn_speed);
    nh->getParam("behavior_tree/very_sharp_turn_speed", very_sharp_turn_speed);
    nh->getParam("behavior_tree/speed_zero_tolerance", speed_zero_tolerance);


    nh->getParam("behavior_tree/min_sign_react_distance", min_sign_react_distance);
    nh->getParam("behavior_tree/max_sign_react_distance", max_sign_react_distance);
    nh->getParam("behavior_tree/overtake_distance", overtake_distance);
    nh->getParam("behaviro_tree/universal_break_factor", universal_break_factor);
    nh->getParam("behavior_tree/barred_area_react_distance", barred_area_react_distance);
    nh->getParam("behavior_tree/oncoming_traffic_clearance", oncoming_traffic_clearance);
    nh->getParam("behavior_tree/max_start_box_distance", max_start_box_distance);
    nh->getParam("behavior_tree/intersection_max_obj_distance", intersection_max_obj_distance);
 
    nh->getParam("behavior_tree/start_value__overtaking_forbidden_zone", overtaking_forbidden_zone);
    nh->getParam("behavior_tree/start_value__express_way", express_way);
    nh->getParam("behavior_tree/start_value__priority_road", priority_road);
    nh->getParam("behavior_tree/start_value__force_stop", force_stop);
    nh->getParam("behavior_tree/start_value__on_bridge", on_bridge);
    nh->getParam("behavior_tree/start_value__speed_limit", speed_limit);
    nh->getParam("behavior_tree/start_value__successful_parking_coung", successful_parking_count);
    nh->getParam("behavior_tree/start_value__give_way", give_way);

    //Convert tick frequency to waiting-milliseconds 
    if(tick_frequency == 0) tick_freq_ms = 0;
    else tick_freq_ms = (1 / tick_frequency) * 1000;
    //Read start states
    std::string states;
    nh->getParam("behavior_tree/initial_states", states);
    std::vector<std::string> initial_states_vector;
    boost::split(initial_states_vector, states, [](char c){return c == '|';});
    initial_states = new std::set<std::string>(initial_states_vector.begin(), initial_states_vector.end());
    while(initial_states->find("") != initial_states->end()) {initial_states->erase("");}
}

void check_parameter_values() {
    if(speed_limit == 0) ROS_WARN("WARNING: speed_limit is set to 0. Check launch file if you'd like to change it.");
    if(general_max_speed == 0) ROS_WARN("WARNING: general_max_speed is set to 0. Check launch file if you'd like to change it.");
    if(general_max_speed_cautious == 0) ROS_WARN("WARNING: general_max_speed_cautious is set to 0. Check launch file if you'd like to change it.");
}


void construct_parkingmode_tree(BT::SequenceNode *head) {
    //Create nodes
    NODES::WaitForStart *node_waitForStart = new NODES::WaitForStart("Waiting for gate");
    NODES::InitialDriving *node_initialDriving = new NODES::InitialDriving("Initial Driving");
    BT::SequenceNode *node_doCourse = new BT::SequenceNode("Course loop", true, true);

    BT::SequenceNode *node_parkingPending = new BT::SequenceNode("Parking");
    BT::SequenceNode *node_driving = new BT::SequenceNode("Driving", true, false);

    NODES::ParkingSpotSearch *node_parkingSpotSearch = new NODES::ParkingSpotSearch("Parking Spot Search");
    NODES::ParkingBreaking *node_parkingBreaking = new NODES::ParkingBreaking("Breaking (parking)");
    NODES::ParkingInProgress *node_parkingInProgress = new NODES::ParkingInProgress("Parking in progress");
    NODES::ParkingReverse *node_parkingReverse = new NODES::ParkingReverse("Parking reverse");

    NODES::FreeDrive *node_freeDrive = new NODES::FreeDrive("Free Drive");
    NODES::FreeDriveIntersectionWait *node_freeDriveIntersectionWait = new NODES::FreeDriveIntersectionWait("Waiting at intersection");
    NODES::IntersectionDrive *node_freeDriveIntersectionDrive = new NODES::IntersectionDrive("Crossing intersection");

    //Link them in a meaningful way
    head->addChild(node_waitForStart);
    head->addChild(node_initialDriving);
    head->addChild(node_doCourse);

    node_doCourse->addChild(node_parkingPending);
    node_doCourse->addChild(node_driving);

    node_parkingPending->addChild(node_parkingSpotSearch);
    node_parkingPending->addChild(node_parkingBreaking);
    node_parkingPending->addChild(node_parkingInProgress);
    node_parkingPending->addChild(node_parkingReverse);

    node_driving->addChild(node_freeDrive);
    node_driving->addChild(node_freeDriveIntersectionWait);
    node_driving->addChild(node_freeDriveIntersectionDrive);
}

void construct_obstaclesmode_tree(BT::SequenceNode *head) {
    //Create nodes
    NODES::WaitForStart *node_waitForStart = new NODES::WaitForStart("Waiting for gate");
    NODES::InitialDriving *node_initialDriving = new NODES::InitialDriving("Initial Driving");
    BT::ParallelNode *node_trackProperty = new BT::ParallelNode("Track property", &NODES::trackPropertyCallback, true);

    BT::SequenceNode *node_objectAvoiding = new BT::SequenceNode("Handling object");
    BT::SequenceNode *node_barredArea = new BT::SequenceNode("Handling barred area");
    BT::SequenceNode *node_crosswalk = new BT::SequenceNode("Handling crosswalk");
    BT::SequenceNode *node_intersection = new BT::SequenceNode("Handling intersection");

    NODES::FollowingObject *node_followingObject = new NODES::FollowingObject("Following object");
    NODES::SwitchToLeftLane *node_objectSwitchToLeft = new NODES::SwitchToLeftLane("Switching to left lane (object)");
    NODES::LeftLaneDrive *node_overtakeObject = new NODES::LeftLaneDrive("Overtake object");
    NODES::SwitchToRightLane *node_objectSwitchToRight = new NODES::SwitchToRightLane("Switching to right lane (object)");

    NODES::BarredAreaAnticipate *node_barredAreaAnticipate = new NODES::BarredAreaAnticipate("Anticipating barred area");
    NODES::SwitchToLeftLane *node_barredAreaSwitchToLeft = new NODES::SwitchToLeftLane("Switching to left lane (barred area)");
    NODES::LeftLaneDrive *node_barredAreaPass = new NODES::LeftLaneDrive("Passing barred area");
    NODES::SwitchToRightLane *node_barredAreaSwitchToRight = new NODES::SwitchToRightLane("Switching to right lane (barred area)");

    NODES::CrosswalkBreak *node_crosswalkBreak = new NODES::CrosswalkBreak("Breaking (crosswalk)");
    NODES::CrosswalkWait *node_crosswalkWait = new NODES::CrosswalkWait("Waiting at crosswalk");

    NODES::IntersectionWait *node_intersectionWait = new NODES::IntersectionWait("Waiting at intersection");
    NODES::IntersectionDrive *node_intersectionDrive = new NODES::IntersectionDrive("Crossing intersection");

    //Link them in a meaningful way
    head->addChild(node_waitForStart);
    head->addChild(node_initialDriving);
    head->addChild(node_trackProperty);
    node_trackProperty->addChild(node_objectAvoiding);
    node_trackProperty->addChild(node_barredArea);
    node_trackProperty->addChild(node_crosswalk);
    node_trackProperty->addChild(node_intersection);
    node_objectAvoiding->addChild(node_followingObject);
    node_objectAvoiding->addChild(node_objectSwitchToLeft);
    node_objectAvoiding->addChild(node_overtakeObject);
    node_objectAvoiding->addChild(node_objectSwitchToRight);
    node_barredArea->addChild(node_barredAreaAnticipate);
    node_barredArea->addChild(node_barredAreaSwitchToLeft);
    node_barredArea->addChild(node_barredAreaPass);
    node_barredArea->addChild(node_barredAreaSwitchToRight);
    node_crosswalk->addChild(node_crosswalkBreak);
    node_crosswalk->addChild(node_crosswalkWait);
    node_intersection->addChild(node_intersectionWait);
    node_intersection->addChild(node_intersectionDrive);
}