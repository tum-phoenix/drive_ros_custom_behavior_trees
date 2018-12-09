#include "bt_node/nodes.h"
#include "bt_node/environment_model.h"
#include "bt_lib/definitions.h"

extern float general_max_speed;
extern float general_max_speed_cautious;
extern float max_bridge_speed;
extern float parking_spot_search_speed;
extern float max_lane_switch_speed;
extern float overtake_distance;
extern float object_following_break_factor;
extern float universal_break_factor;
extern float barred_area_react_distance;
extern float oncoming_traffic_clearance;

extern int speed_limit;
extern bool overtaking_forbidden_zone;
extern int successful_parking_count;
extern float current_velocity;

namespace NODES {


    drive_ros_custom_behavior_trees::TrajectoryMessage trajectory_msg;
    TrackPropertyMessageHandler msg_handler;

    /* ---------- WaitForStart ---------- */
    WaitForStart::WaitForStart(std::string name) : BT::ActionNode(name) {

    }
    void WaitForStart::tick() {
        if(EnvModel::start_box_open()) {
            set_state(SUCCESS);
        }
        else {
            //Keep the wheels straight, don't drive
            trajectory_msg.control_metadata = STRAIGHT_FORWARD;
            trajectory_msg.max_speed = 0;
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- InitialDriving ---------- */
    InitialDriving::InitialDriving(std::string name) : BT::ActionNode(name) {

    }
    void InitialDriving::tick() {
        if(EnvModel::start_line_distance() < 0.2) {
            set_state(SUCCESS);
        }
        else {
            trajectory_msg.control_metadata = STRAIGHT_FORWARD;
            trajectory_msg.max_speed = general_max_speed_cautious; 
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- ParkingSpotSearch ---------- */
    ParkingSpotSearch::ParkingSpotSearch(std::string name) : BT::ActionNode(name) {

    }
    void ParkingSpotSearch::tick() {
        if(successful_parking_count >= 2) {
            set_state(FAILURE); //to break the parking process and start driving immediately
        }
        else if(true) { //Parking spot detected
            set_state(SUCCESS);
        }
        else {
            trajectory_msg.control_metadata = STANDARD;
            trajectory_msg.max_speed = parking_spot_search_speed;
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- ParkingBreaking ---------- */
    ParkingBreaking::ParkingBreaking(std::string name) : BT::ActionNode(name) {

    }
    void ParkingBreaking::tick() {
        if(current_velocity < 0.05) {
            set_state(SUCCESS);
        }
        else {
            trajectory_msg.control_metadata = STANDARD;
            trajectory_msg.max_speed = 0;
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- ParkingInProgress ---------- */
    ParkingInProgress::ParkingInProgress(std::string name) : BT::ActionNode(name) {

    }
    void ParkingInProgress::tick() { //TODO
        set_state(SUCCESS);
    }

    /* ---------- ParkingReverse ---------- */
    ParkingReverse::ParkingReverse(std::string name) : BT::ActionNode(name) {

    }
    void ParkingReverse::tick() {
        set_state(SUCCESS); //TODO
    }

    /* ---------- FreeDrive ---------- */
    FreeDrive::FreeDrive(std::string name) : BT::ActionNode(name) {

    }
    void FreeDrive::tick() {
        if(EnvModel::intersection_immediately_upfront()) {
            set_state(SUCCESS);
        }
        else if(EnvModel::start_line_distance() < 0.2) { //Start line/Parking sign detected
            set_state(FAILURE); //Break infinite drive loop to re-enter parking mode
        }
        else {
            trajectory_msg.control_metadata = STANDARD;
            trajectory_msg.max_speed = speed_limit < max_bridge_speed ? speed_limit : max_bridge_speed;
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- FreeDriveIntersection ---------- */
    FreeDriveIntersection::FreeDriveIntersection(std::string name) : BT::ActionNode(name) {

    }
    void FreeDriveIntersection::tick() {
        if(true) { //Back on normal road
            set_state(SUCCESS);
        }
        else { //Maybe implement turn left/right?
            trajectory_msg.control_metadata = STRAIGHT_FORWARD;
            trajectory_msg.max_speed = 1000; //Speed doesn't need to be limited 
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- SwitchToLeftLane ---------- */
    SwitchToLeftLane::SwitchToLeftLane(std::string name) : BT::ActionNode(name) {

    }
    void SwitchToLeftLane::tick() {
        if(EnvModel::get_current_lane() == 0) {
            set_state(SUCCESS);
        }
        else {
            drive_ros_custom_behavior_trees::TrajectoryMessage *msg = new drive_ros_custom_behavior_trees::TrajectoryMessage();
            msg->control_metadata = SWITCH_TO_LEFT_LANE;
            msg->max_speed = max_lane_switch_speed;
            msg_handler.addMessageSuggestion(msg);
        }
    }

    /* ---------- SwitchToRightLane ---------- */
    SwitchToRightLane::SwitchToRightLane(std::string name) : BT::ActionNode(name) {

    }
    void SwitchToRightLane::tick() {
        if(EnvModel::get_current_lane() == 1) {
            set_state(SUCCESS);
        }
        else {
            drive_ros_custom_behavior_trees::TrajectoryMessage *msg = new drive_ros_custom_behavior_trees::TrajectoryMessage();
            msg->control_metadata = SWITCH_TO_RIGHT_LANE;
            msg->max_speed = max_lane_switch_speed;
            msg_handler.addMessageSuggestion(msg);
        }
    }

    /* ---------- FollowingObject ---------- */
    FollowingObject::FollowingObject(std::string name) : BT::ActionNode(name) {
        last_speed = 0;
    }
    void FollowingObject::tick() {
        if(!EnvModel::object_on_current_lane()) {
            set_state(FAILURE);
        } else if(!(EnvModel::intersection_immediately_upfront() || overtaking_forbidden_zone)
            && EnvModel::upfront_object_distance() < overtake_distance) {
            set_state(SUCCESS);
        }
        else {
            if(last_speed == 0) last_speed = current_velocity;

            drive_ros_custom_behavior_trees::TrajectoryMessage *msg = new drive_ros_custom_behavior_trees::TrajectoryMessage();
            msg->control_metadata = SWITCH_TO_RIGHT_LANE;
            if(EnvModel::upfront_object_distance() < overtake_distance) { //Increase distance
                msg->max_speed = last_speed * object_following_break_factor;
                last_speed = msg->max_speed;
            }
            else msg->max_speed = last_speed * (1 / object_following_break_factor); //Reduce distance
            msg_handler.addMessageSuggestion(msg);
        }
    }
    
    /* ---------- LeftLaneDrive ---------- */
    LeftLaneDrive::LeftLaneDrive(std::string name) : BT::ActionNode(name) {

    }
    void LeftLaneDrive::tick() {
        if(false) { //When overtaking / passing barred area is finished.
            set_state(SUCCESS);
        }
        else {
            drive_ros_custom_behavior_trees::TrajectoryMessage *msg = new drive_ros_custom_behavior_trees::TrajectoryMessage();
            msg->control_metadata = STANDARD;
            msg->max_speed = general_max_speed;
            msg_handler.addMessageSuggestion(msg);
        }
    }

    /* ---------- BarredAreaAnticipate ---------- */
    BarredAreaAnticipate::BarredAreaAnticipate(std::string name) : BT::ActionNode(name) {

    }
    void BarredAreaAnticipate::tick() {
        if(EnvModel::barred_area_distance() < barred_area_react_distance
            && (EnvModel::upfront_object_distance() > oncoming_traffic_clearance)) {
            set_state(SUCCESS);
        }
        else {
            drive_ros_custom_behavior_trees::TrajectoryMessage *msg = new drive_ros_custom_behavior_trees::TrajectoryMessage();
            msg->control_metadata = STANDARD;
            msg->max_speed = 0;
            msg_handler.addMessageSuggestion(msg);
        }
    }

    /* ---------- CrosswalkBreak ---------- */
    CrosswalkBreak::CrosswalkBreak(std::string name) : BT::ActionNode(name) {

    }
    void CrosswalkBreak::tick() {
        if(EnvModel::crosswalk_distance() == -1) set_state(FAILURE);
        if(current_velocity == 0) {
            set_state(SUCCESS);
        }
        else {
            drive_ros_custom_behavior_trees::TrajectoryMessage *msg = new drive_ros_custom_behavior_trees::TrajectoryMessage();
            if(EnvModel::crosswalk_distance() < EnvModel::current_break_distance()) {
                msg->control_metadata = STANDARD;
                msg->max_speed = 0;
            }
            else {
                msg->control_metadata = STANDARD;
                msg->max_speed = general_max_speed_cautious;
                msg_handler.addMessageSuggestion(msg);
            }
        }
    }

    /* ---------- CrosswalkWait ---------- */
    CrosswalkWait::CrosswalkWait(std::string name) : BT::ActionNode(name) {

    }
    void CrosswalkWait::tick() {
        /*
            Track pedestrians
        */
        if(EnvModel::num_of_pedestrians() == 0 || (pedestrians_detected_on_track > 0 && EnvModel::crosswalk_clear())) {
            set_state(SUCCESS);
        }
        else {
            drive_ros_custom_behavior_trees::TrajectoryMessage *msg = new drive_ros_custom_behavior_trees::TrajectoryMessage();
            msg->control_metadata = STANDARD;
            msg->max_speed = 0;
            msg_handler.addMessageSuggestion(msg);
        }
    }

    void trackPropertyCallback(std::vector<BT::TreeNode *> *nodes) {
        msg_handler.evaluate_and_send();
        //Activate nodes when necessary
    }
    
    void TrackPropertyMessageHandler::addMessageSuggestion(drive_ros_custom_behavior_trees::TrajectoryMessage *msg) {
        suggestions.insert(msg);
    }
    void TrackPropertyMessageHandler::evaluate_and_send() {
        float speed = general_max_speed;
        int metadata = STRAIGHT_FORWARD;
        for(drive_ros_custom_behavior_trees::TrajectoryMessage *msg : suggestions) {
            if(msg->max_speed < speed) speed = msg->max_speed;
            if(metadata == STRAIGHT_FORWARD) metadata = msg->control_metadata;
        }
        suggestions.clear();
        drive_ros_custom_behavior_trees::TrajectoryMessage final_msg;
        final_msg.control_metadata = metadata;
        final_msg.max_speed = speed;
        publish_trajectory_metadata(final_msg);
    }

}