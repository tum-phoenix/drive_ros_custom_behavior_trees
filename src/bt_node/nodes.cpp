#include "bt_node/nodes.h"
#include "bt_node/environment_model.h"
#include "bt_lib/definitions.h"
#include "bt_lib/tree.h"
#include "bt_node/value_definitions.h"
#include "bt_node/node_reset.h"

#include <math.h>

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
extern float intersection_turn_speed;
extern float speed_zero_tolerance;

extern bool overtaking_forbidden_zone;
extern bool priority_road;
extern bool give_way;
extern int speed_limit;
extern int successful_parking_count;
extern int intersection_turn_indication;
extern float current_velocity;

extern std::string mode;
extern BT::Tree *tree;

namespace NODES {


    drive_ros_custom_behavior_trees::TrajectoryMessage trajectory_msg;
    TrackPropertyMessageHandler msg_handler;

    /* ---------- WaitForStart ---------- */
    WaitForStart::WaitForStart(std::string name) : BT::ActionNode(name) {}
    void WaitForStart::tick() {
        if(EnvModel::start_box_open()) {
            set_state(SUCCESS);
        }
        else {
            //Keep the wheels straight, don't drive
            trajectory_msg.control_metadata = DRIVE_CONTROL_STRAIGHT_FORWARD;
            trajectory_msg.max_speed = 0;
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- InitialDriving ---------- */
    InitialDriving::InitialDriving(std::string name) : BT::ActionNode(name) {}
    void InitialDriving::tick() {
        if(EnvModel::start_line_distance() < 0.2) {
            set_state(SUCCESS);
        }
        else {
            trajectory_msg.control_metadata = DRIVE_CONTROL_STRAIGHT_FORWARD;
            trajectory_msg.max_speed = fmin(general_max_speed_cautious, speed_limit); 
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- ParkingSpotSearch ---------- */
    ParkingSpotSearch::ParkingSpotSearch(std::string name) : BT::ActionNode(name) {}
    void ParkingSpotSearch::tick() {
        if(successful_parking_count >= 2) {
            set_state(FAILURE); //to break the parking process and start driving immediately
        }
        else if(true) { //Parking spot detected
            set_state(SUCCESS);
        }
        else {
            trajectory_msg.control_metadata = DRIVE_CONTROL_STANDARD;
            trajectory_msg.max_speed = fmin(parking_spot_search_speed, speed_limit);
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- ParkingBreaking ---------- */
    ParkingBreaking::ParkingBreaking(std::string name) : BT::ActionNode(name) {}
    void ParkingBreaking::tick() {
        if(current_velocity < speed_zero_tolerance) {
            set_state(SUCCESS);
        }
        else {
            trajectory_msg.control_metadata = DRIVE_CONTROL_STANDARD;
            trajectory_msg.max_speed = 0;
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- ParkingInProgress ---------- */
    ParkingInProgress::ParkingInProgress(std::string name) : BT::ActionNode(name) {}
    void ParkingInProgress::tick() { //TODO
        //TODO flash turn indicators
        set_state(SUCCESS);
    }

    /* ---------- ParkingReverse ---------- */
    ParkingReverse::ParkingReverse(std::string name) : BT::ActionNode(name) {}
    void ParkingReverse::tick() {
        if(EnvModel::get_current_lane() == LANE_RIGHT) {
          set_state(SUCCESS); //Car is back on track  
        }
        else if(EnvModel::get_current_lane() == LANE_UNDEFINED) { 
            //TODO
        }
        else { //Car is on left lane, error reset!
            reset_tree_state(tree);
        }
    }

    /* ---------- FreeDrive ---------- */
    FreeDrive::FreeDrive(std::string name) : BT::ActionNode(name) {}
    void FreeDrive::tick() {
        if(EnvModel::intersection_immediately_upfront()) {
            set_state(SUCCESS);
        }
        else if(EnvModel::start_line_distance() < 0.2 || EnvModel::parking_sign_distance() < 0.2) { //Start line/Parking sign detected
            set_state(FAILURE); //Break infinite drive loop to re-enter parking mode
        }
        else {
            trajectory_msg.control_metadata = DRIVE_CONTROL_STANDARD;
            trajectory_msg.max_speed = fmin(speed_limit, 
                EnvModel::in_sharp_turn() ? sharp_turn_speed : EnvModel::in_very_sharp_turn() ? very_sharp_turn_speed : general_max_speed);
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- FreeDriveIntersectionWait ---------- */
    FreeDriveIntersectionWait::FreeDriveIntersectionWait(std::string name) : BT::ActionNode(name) {
        start_waiting = 0;
    }
    void FreeDriveIntersectionWait::tick() {
        if(current_velocity < speed_zero_tolerance) start_waiting = 1;
        if(start_waiting == 1) {
            waiting_started = std::chrono::system_clock::now();
            start_waiting = 2;
        }
        if(start_waiting == 2 
            && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - waiting_started).count() > 3000) { //Waiting time is over or no waiting is needed
            set_state(SUCCESS); //The intersection crossing is done by IntersectionDrive.
        }
        else {
            trajectory_msg.control_metadata = DRIVE_CONTROL_STANDARD;
            trajectory_msg.max_speed = 0;
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- SwitchToLeftLane ---------- */
    SwitchToLeftLane::SwitchToLeftLane(std::string name) : BT::ActionNode(name) {}
    void SwitchToLeftLane::tick() {
        drive_ros_custom_behavior_trees::TrajectoryMessage *msg = new drive_ros_custom_behavior_trees::TrajectoryMessage();
        if(EnvModel::get_current_lane() == LANE_LEFT) {
            set_state(SUCCESS);
        }
        else if(EnvModel::get_current_lane() == LANE_RIGHT) {
            if(EnvModel::object_min_lane_distance(LANE_LEFT) > oncoming_traffic_clearance) {
                msg->max_speed = fmin(max_lane_switch_speed, speed_limit);
                msg->control_metadata = DRIVE_CONTROL_SWITCH_LEFT;
                msg_handler.addMessageSuggestion(msg);
            }
            else {
                msg->max_speed = 0;
                msg->control_metadata = DRIVE_CONTROL_STANDARD;
                msg_handler.addMessageSuggestion(msg);
            }
        }
        else { //Lane is undefined; in the middle of lane change
            if(!EnvModel::object_on_lane(LANE_LEFT)) {
                msg->control_metadata = DRIVE_CONTROL_SWITCH_LEFT;
            } else {
                msg->control_metadata = DRIVE_CONTROL_SWITCH_RIGHT;
            }
            msg->max_speed = fmin(max_lane_switch_speed, speed_limit);
            msg_handler.addMessageSuggestion(msg);
        }
    }

    /* ---------- SwitchToRightLane ---------- */
    SwitchToRightLane::SwitchToRightLane(std::string name) : BT::ActionNode(name) {}
    void SwitchToRightLane::tick() {
        if(EnvModel::get_current_lane() == LANE_RIGHT) {
            set_state(SUCCESS);
        }
        else { //No surprises to be expected here.
            drive_ros_custom_behavior_trees::TrajectoryMessage *msg = new drive_ros_custom_behavior_trees::TrajectoryMessage();
            msg->control_metadata = DRIVE_CONTROL_SWITCH_RIGHT;
            msg->max_speed = fmin(max_lane_switch_speed, speed_limit);
            msg_handler.addMessageSuggestion(msg);
        }
    }

    /* ---------- FollowingObject ---------- */
    FollowingObject::FollowingObject(std::string name) : BT::ActionNode(name) {
        last_speed = 0;
    }
    void FollowingObject::tick() {
        if(!EnvModel::object_on_lane(EnvModel::get_current_lane())) { //Cancel folowing; there's no object in the way any more.
            set_state(FAILURE);
        } else if(!(EnvModel::intersection_immediately_upfront() || overtaking_forbidden_zone)
            && EnvModel::object_min_lane_distance(LANE_RIGHT) < overtake_distance) {
            set_state(SUCCESS);
        }
        else {
            if(last_speed == 0) last_speed = current_velocity;

            drive_ros_custom_behavior_trees::TrajectoryMessage *msg = new drive_ros_custom_behavior_trees::TrajectoryMessage();
            msg->control_metadata = DRIVE_CONTROL_STANDARD;
            if(EnvModel::object_min_lane_distance(LANE_RIGHT) < overtake_distance - 0.3) 
                msg->max_speed = fmin(last_speed * object_following_break_factor, speed_limit); //Increase distance
            else 
                msg->max_speed = fmin(last_speed * (1 / object_following_break_factor), speed_limit); //Reduce distance
            last_speed = msg->max_speed;
            msg_handler.addMessageSuggestion(msg);
        }
    }
    
    /* ---------- LeftLaneDrive ---------- */
    LeftLaneDrive::LeftLaneDrive(std::string name) : BT::ActionNode(name) {}
    void LeftLaneDrive::tick() {
        if(!EnvModel::object_on_lane(LANE_RIGHT) 
            && (EnvModel::barred_area_distance() > 4 || EnvModel::barred_area_distance() == -1)) { //When overtaking / passing barred area is finished.
            set_state(SUCCESS);
        }
        else {
            if(EnvModel::object_min_lane_distance(LANE_LEFT) < oncoming_traffic_clearance 
                || EnvModel::barred_area_distance() < oncoming_traffic_clearance
                || EnvModel::pass_by_on_right_distance() < oncoming_traffic_clearance) { //Abort, there's no room to overtake.
                    set_state(SUCCESS); //Go to "switch to right lane" and maybe then go back to "switch to left lane" again to try once more.
            }
            else {
                drive_ros_custom_behavior_trees::TrajectoryMessage *msg = new drive_ros_custom_behavior_trees::TrajectoryMessage();
                msg->control_metadata = DRIVE_CONTROL_STANDARD;
                msg->max_speed = fmin(general_max_speed, speed_limit); //Spend as little tim
                msg_handler.addMessageSuggestion(msg);
            }
        }
    }

    /* ---------- BarredAreaAnticipate ---------- */
    BarredAreaAnticipate::BarredAreaAnticipate(std::string name) : BT::ActionNode(name) {}
    void BarredAreaAnticipate::tick() {
        if(EnvModel::barred_area_distance() < barred_area_react_distance
            && (EnvModel::object_min_lane_distance(LANE_LEFT) > oncoming_traffic_clearance)) {
            set_state(SUCCESS);
        }
        else {
            drive_ros_custom_behavior_trees::TrajectoryMessage *msg = new drive_ros_custom_behavior_trees::TrajectoryMessage();
            msg->control_metadata = DRIVE_CONTROL_STANDARD;
            msg->max_speed = 0;
            msg_handler.addMessageSuggestion(msg);
        }
    }

    /* ---------- CrosswalkBreak ---------- */
    CrosswalkBreak::CrosswalkBreak(std::string name) : BT::ActionNode(name) {}
    void CrosswalkBreak::tick() {
        if(EnvModel::crosswalk_distance() == -1) set_state(FAILURE);
        if(EnvModel::num_of_pedestrians() == 0 || current_velocity < speed_zero_tolerance) {
            set_state(SUCCESS);
        }
        else {
            drive_ros_custom_behavior_trees::TrajectoryMessage *msg = new drive_ros_custom_behavior_trees::TrajectoryMessage();
            if(EnvModel::crosswalk_distance() < EnvModel::current_break_distance()) {
                msg->control_metadata = DRIVE_CONTROL_STANDARD;
                msg->max_speed = 0;
            }
            else {
                msg->control_metadata = DRIVE_CONTROL_STANDARD;
                msg->max_speed = fmin(general_max_speed_cautious, speed_limit);
                msg_handler.addMessageSuggestion(msg);
            }
        }
    }

    /* ---------- CrosswalkWait ---------- */
    CrosswalkWait::CrosswalkWait(std::string name) : BT::ActionNode(name) {}
    void CrosswalkWait::tick() {
        if(EnvModel::num_of_pedestrians() == 0 || EnvModel::crosswalk_clear()) {
            set_state(SUCCESS);
        }
        else {
            drive_ros_custom_behavior_trees::TrajectoryMessage *msg = new drive_ros_custom_behavior_trees::TrajectoryMessage();
            msg->control_metadata = DRIVE_CONTROL_STANDARD;
            msg->max_speed = 0;
            msg_handler.addMessageSuggestion(msg);
        }
    }

    /* ---------- IntersectionWait ---------- */
    IntersectionWait::IntersectionWait(std::string name) : BT::ActionNode(name) {
        start_waiting = true;
    }
    void IntersectionWait::tick() {
        if(start_waiting) {
            waiting_started = std::chrono::system_clock::now();
            start_waiting = false;
        }

        if(priority_road 
            || (!priority_road 
                && !EnvModel::intersection_no_object_right() 
                && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - waiting_started).count() > 3000
            || (!priority_road && (
                (!give_way && EnvModel::intersection_no_object_right()) 
                || (give_way && EnvModel::intersection_no_object()))))) {
            set_state(SUCCESS);
        }
        else {
            drive_ros_custom_behavior_trees::TrajectoryMessage *msg = new drive_ros_custom_behavior_trees::TrajectoryMessage();
            msg->control_metadata = DRIVE_CONTROL_STANDARD;
            msg->max_speed = EnvModel::intersection_immediately_upfront() ? 0 : general_max_speed_cautious;
            msg_handler.addMessageSuggestion(msg);
        }
    }

    /* ---------- IntersectionDrive ---------- */
    IntersectionDrive::IntersectionDrive(std::string name) : BT::ActionNode(name) {}
    void IntersectionDrive::tick() {
        if(EnvModel::get_current_lane() != LANE_UNDEFINED) { //On normal track again
            set_state(SUCCESS);
        }
        else {
            drive_ros_custom_behavior_trees::TrajectoryMessage *msg = new drive_ros_custom_behavior_trees::TrajectoryMessage();
            if(!mode.compare("PARKING")) {
                msg->control_metadata = 0;
            } else {
                msg->control_metadata = intersection_turn_indication == 0 ? DRIVE_CONTROL_STRAIGHT_FORWARD : intersection_turn_indication;
            }
            msg->max_speed = intersection_turn_indication == 0 ? fmin(general_max_speed_cautious, speed_limit) : intersection_turn_speed;
            msg_handler.addMessageSuggestion(msg);
        }
    }

    void trackPropertyCallback(std::vector<BT::TreeNode *> *nodes) {
        msg_handler.evaluate_and_send();
        //Activate nodes when necessary
        //Nodes may only be activated when they are idling. Of course they can't be activated when already running, but they also shouldn't be when in SUCCESS or FAILURE state.
        if((*nodes)[0]->get_state() == IDLE 
            && EnvModel::object_min_lane_distance(LANE_RIGHT) < 2 * overtake_distance) {
            (*nodes)[0]->set_state(RUNNING);
        }
        if((*nodes)[1]->get_state() == IDLE 
            && EnvModel::barred_area_distance() < barred_area_react_distance) {
            (*nodes)[1]->set_state(RUNNING);
        }
        if((*nodes)[2]->get_state() == IDLE 
            && EnvModel::crosswalk_distance() < EnvModel::current_break_distance()
            && EnvModel::crosswalk_distance() > 0.1) { //This is a safety feature so the state won't reactivate when waiting at the crosswalk is already over
            (*nodes)[2]->set_state(RUNNING);
        }
        if((*nodes)[3]->get_state() == IDLE 
            && EnvModel::intersection_immediately_upfront()) {
            (*nodes)[3]->set_state(RUNNING);
        }
    }
    
    void TrackPropertyMessageHandler::addMessageSuggestion(drive_ros_custom_behavior_trees::TrajectoryMessage *msg) {
        suggestions.insert(msg);
    }
    void TrackPropertyMessageHandler::evaluate_and_send() {
        float speed = fmin(
            EnvModel::in_sharp_turn() ? sharp_turn_speed :
                EnvModel::in_very_sharp_turn() ? very_sharp_turn_speed : general_max_speed, speed_limit);
        int metadata = DRIVE_CONTROL_STRAIGHT_FORWARD;
        for(drive_ros_custom_behavior_trees::TrajectoryMessage *msg : suggestions) {
            if(msg->max_speed < speed) speed = msg->max_speed;
            if(metadata == DRIVE_CONTROL_STRAIGHT_FORWARD) metadata = msg->control_metadata;
        }
        suggestions.clear();
        drive_ros_custom_behavior_trees::TrajectoryMessage final_msg;
        final_msg.control_metadata = metadata;
        final_msg.max_speed = speed;
        publish_trajectory_metadata(final_msg);
    }

}