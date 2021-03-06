#include "bt_node/nodes.h"
#include "bt_node/environment_model.h"
#include "bt_lib/definitions.h"
#include "bt_lib/tree.h"
#include "bt_node/node_reset.h"

#include "drive_ros_msgs/TrajectoryMetaInput.h"
#include "drive_ros_msgs/Lane.h"
#include "drive_ros_msgs/value_definitions.h"

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
extern float intersection_turn_duration;

extern bool overtaking_forbidden_zone;
extern bool priority_road;
extern bool give_way;
extern bool on_bridge;
extern int speed_limit;
extern int successful_parking_count;
extern int intersection_turn_indication;
extern float current_velocity;

extern std::string mode;
extern BT::Tree *tree;

namespace NODES {


    drive_ros_msgs::TrajectoryMetaInput trajectory_msg;
    TrackPropertyMessageHandler msg_handler;

    /* ---------- class:WaitForStart ---------- */
    WaitForStart::WaitForStart(std::string name) : BT::ActionNode(name) {}
    void WaitForStart::tick() {
        if(EnvModel::start_box_open()) {
            set_state(SUCCESS);
        }
        else {
            //Keep the wheels straight, don't drive
            trajectory_msg.control_metadata = drive_ros_msgs::TrajectoryMetaInput::STRAIGHT_FORWARD;
            trajectory_msg.max_speed = 0;
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- class:InitialDriving ---------- */
    InitialDriving::InitialDriving(std::string name) : BT::ActionNode(name) {}
    void InitialDriving::tick() {
        //To improve robustness of the system, InitialDriving is completed when either the start line of the Parking Zone sign is detected.
        if((EnvModel::start_line_distance() != -1 && EnvModel::start_line_distance() < 0.2)
            || (EnvModel::parking_sign_distance() != -1 && EnvModel::parking_sign_distance() < 0.2)) {
            set_state(SUCCESS);
        }
        else {
            //To avoid confusion of line detection etc. because of the merging curve at the start, the car shall go straight forward.
            trajectory_msg.control_metadata = drive_ros_msgs::TrajectoryMetaInput::STRAIGHT_FORWARD;
            trajectory_msg.max_speed = fmin(general_max_speed_cautious, speed_limit); 
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- class:Parking ---------- */
    Parking::Parking(std::string name) : BT::ActionNode(name) {}
    void Parking::tick() {
        if(successful_parking_count >= 2) {
            //skip_failed_child is activated for parent node, thus parking will just be skipped
            set_state(FAILURE);
            return;
        }
        if(parking_finished()) {
            set_state(SUCCESS);
        }
        trajectory_msg.control_metadata = drive_ros_msgs::TrajectoryMetaInput::PARKING;
        trajectory_msg.max_speed = general_max_speed_cautious;
        publish_trajectory_metadata(trajectory_msg);
    }

    /* ---------- class:FreeDrive ---------- */
    FreeDrive::FreeDrive(std::string name) : BT::ActionNode(name) {}
    void FreeDrive::tick() {
        if(EnvModel::intersection_immediately_upfront()) {
            set_state(SUCCESS);
        }
        else if((EnvModel::start_line_distance() != -1 && EnvModel::start_line_distance() < 0.2)
            ||  (EnvModel::parking_sign_distance() != -1 && EnvModel::parking_sign_distance() < 0.2)) {
            set_state(FAILURE); //Break infinite drive loop to re-enter parking mode
        }
        else {
            if(EnvModel::get_current_lane() == drive_ros_msgs::Lane::RIGHT) {
                trajectory_msg.control_metadata = drive_ros_msgs::TrajectoryMetaInput::STANDARD;
                trajectory_msg.max_speed = fmin(speed_limit, 
                    EnvModel::in_sharp_turn() ? sharp_turn_speed : EnvModel::in_very_sharp_turn() ? very_sharp_turn_speed : general_max_speed);
            }
            else if(EnvModel::get_current_lane() == drive_ros_msgs::Lane::LEFT) {
                trajectory_msg.control_metadata = drive_ros_msgs::TrajectoryMetaInput::SWITCH_RIGHT;
                trajectory_msg.max_speed = fmin(speed_limit, 
                    EnvModel::in_sharp_turn() ? sharp_turn_speed : EnvModel::in_very_sharp_turn() ? very_sharp_turn_speed : general_max_speed);
            }
            else { //Car is out of track
                
            }
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- class:FreeDriveIntersectionWait ---------- */
    FreeDriveIntersectionWait::FreeDriveIntersectionWait(std::string name) : BT::ActionNode(name) {
        waiting_status = 0;
    }
    void FreeDriveIntersectionWait::tick() {
        if(waiting_status == 0 && (current_velocity < speed_zero_tolerance)) waiting_status = 1;
        if(waiting_status == 1) {
            waiting_started = ros::Time::now();
            waiting_status = 2;
        }
        if(waiting_status == 2 
            && (ros::Time::now().toSec() - waiting_started.toSec()) > 3.0) {
            set_state(SUCCESS); //The intersection crossing is done by IntersectionDrive.
        }
        else {
            trajectory_msg.control_metadata = drive_ros_msgs::TrajectoryMetaInput::STANDARD;
            trajectory_msg.max_speed = 0;
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- class:SwitchToLeftLane ---------- */
    SwitchToLeftLane::SwitchToLeftLane(std::string name) : BT::ActionNode(name) {}
    void SwitchToLeftLane::tick() {
        drive_ros_msgs::TrajectoryMetaInput *msg = new drive_ros_msgs::TrajectoryMetaInput();
        if(EnvModel::get_current_lane() == drive_ros_msgs::Lane::LEFT) {
            set_state(SUCCESS);
        }
        else if(EnvModel::get_current_lane() == drive_ros_msgs::Lane::RIGHT) {
            if(EnvModel::object_min_lane_distance(drive_ros_msgs::Lane::LEFT) > oncoming_traffic_clearance) {
                msg->max_speed = max_lane_switch_speed;
                msg->control_metadata = drive_ros_msgs::TrajectoryMetaInput::SWITCH_LEFT;
                msg_handler.addMessageSuggestion(msg);
            }
            else {
                msg->max_speed = 0;
                msg->control_metadata = drive_ros_msgs::TrajectoryMetaInput::STANDARD;
                msg_handler.addMessageSuggestion(msg);
            }
        }
        else { //Lane is undefined; in the middle of lane change
            if(!EnvModel::object_on_lane(drive_ros_msgs::Lane::LEFT)) {
                msg->control_metadata = drive_ros_msgs::TrajectoryMetaInput::SWITCH_LEFT;
                msg->max_speed = max_lane_switch_speed;
                msg_handler.addMessageSuggestion(msg);
            } else {
                set_state(FAILURE);
            }
        }
    }

    /* ---------- class:SwitchToRightLane ---------- */
    SwitchToRightLane::SwitchToRightLane(std::string name) : BT::ActionNode(name) {}
    void SwitchToRightLane::tick() {
        //Currently, we don't need to explicitly send SWITCH_RIGHT.
        set_state(SUCCESS);
        return;
        //Actual node code; currently unused ---------------------------------------------
        if(EnvModel::get_current_lane() == drive_ros_msgs::Lane::RIGHT) {
            set_state(SUCCESS);
        }
        else {
            drive_ros_msgs::TrajectoryMetaInput *msg = new drive_ros_msgs::TrajectoryMetaInput();
            //If there's at least some space on the right lane, switch.
            if((EnvModel::object_min_lane_distance(drive_ros_msgs::Lane::RIGHT) != -1 || EnvModel::object_min_lane_distance(drive_ros_msgs::Lane::RIGHT) > 0.5)
                && (EnvModel::barred_area_right_distance() != -1 || EnvModel::barred_area_right_distance() > 0.5)) {
                msg->control_metadata = drive_ros_msgs::TrajectoryMetaInput::SWITCH_RIGHT;
                msg->max_speed = max_lane_switch_speed;
                msg_handler.addMessageSuggestion(msg);
            }
            //Otherwise slowly go forward and switch to the right lane as soon as possible.
            else {
                msg->control_metadata = drive_ros_msgs::TrajectoryMetaInput::STANDARD;
                msg->max_speed = general_max_speed_cautious;
                msg_handler.addMessageSuggestion(msg);
            }
        }
    }

    /* ---------- class:FollowingObject ---------- */
    FollowingObject::FollowingObject(std::string name) : BT::ActionNode(name) {
        last_speed = 0;
    }
    void FollowingObject::tick() {
        if(!EnvModel::object_on_lane(drive_ros_msgs::Lane::RIGHT)) { //Cancel following; there's no object in the way any more.
            set_state(FAILURE);
        } else if(!(EnvModel::intersection_immediately_upfront() || overtaking_forbidden_zone)
            && EnvModel::object_min_lane_distance(drive_ros_msgs::Lane::RIGHT) < overtake_distance) { //According to the regulations, there's nothing on the left lane when an obstacle is in front of you.
            set_state(SUCCESS);
        }
        else { //Can't yet overtake, just follow the object.
            if(last_speed == 0) last_speed = current_velocity;

            drive_ros_msgs::TrajectoryMetaInput *msg = new drive_ros_msgs::TrajectoryMetaInput();
            msg->control_metadata = drive_ros_msgs::TrajectoryMetaInput::STANDARD;
            if(EnvModel::object_min_lane_distance(drive_ros_msgs::Lane::RIGHT) < overtake_distance - 0.3) 
                msg->max_speed = last_speed * object_following_break_factor; //Increase distance
            else 
                msg->max_speed = last_speed * (1 / object_following_break_factor); //Reduce distance
            last_speed = msg->max_speed;
            msg_handler.addMessageSuggestion(msg);
        }
    }
    
    /* ---------- class:LeftLaneDrive ---------- */
    LeftLaneDrive::LeftLaneDrive(std::string name) : BT::ActionNode(name) {
        start_waiting = true;
    }
    void LeftLaneDrive::tick() {
        if(start_waiting) {
            waiting_started = ros::Time::now();
            start_waiting = false;
        }
        if(!EnvModel::object_on_lane(drive_ros_msgs::Lane::RIGHT) 
            && (EnvModel::barred_area_right_distance() > oncoming_traffic_clearance 
                || EnvModel::barred_area_right_distance() == -1)
            && !start_waiting 
            && ((ros::Time::now().toSec() - waiting_started.toSec()) > 0.5)) { //When overtaking / passing barred area is finished.
            set_state(SUCCESS);
            start_waiting = true;
        }
        else {
            if(EnvModel::object_min_lane_distance(drive_ros_msgs::Lane::LEFT) < oncoming_traffic_clearance 
                || (EnvModel::barred_area_left_distance() == -1 ? 10000 : EnvModel::barred_area_left_distance()) < oncoming_traffic_clearance
                || (EnvModel::pass_by_on_right_distance() == -1 ? 10000 : EnvModel::pass_by_on_right_distance()) < oncoming_traffic_clearance) { //Abort, there's no room to overtake.
                    set_state(SUCCESS); //Go to "switch to right lane" and maybe then go back to "switch to left lane" again to try once more.
            }
            else {
                drive_ros_msgs::TrajectoryMetaInput *msg = new drive_ros_msgs::TrajectoryMetaInput();
                msg->control_metadata = drive_ros_msgs::TrajectoryMetaInput::SWITCH_LEFT;
                msg->max_speed = general_max_speed; //Spend as little time as possible on left lane
                msg_handler.addMessageSuggestion(msg);
            }
        }
    }

    /* ---------- class:BarredAreaAnticipate ---------- */
    BarredAreaAnticipate::BarredAreaAnticipate(std::string name) : BT::ActionNode(name) {}
    void BarredAreaAnticipate::tick() {
        if(EnvModel::barred_area_right_distance() != -1 && EnvModel::barred_area_right_distance() < barred_area_react_distance
            && (EnvModel::object_min_lane_distance(drive_ros_msgs::Lane::LEFT) > oncoming_traffic_clearance)) {
            set_state(SUCCESS);
        }
        else {
            drive_ros_msgs::TrajectoryMetaInput *msg = new drive_ros_msgs::TrajectoryMetaInput();
            msg->control_metadata = drive_ros_msgs::TrajectoryMetaInput::STANDARD;
            msg->max_speed = 0;
            msg_handler.addMessageSuggestion(msg);
        }
    }

    /* ---------- class:CrosswalkBreak ---------- */
    CrosswalkBreak::CrosswalkBreak(std::string name) : BT::ActionNode(name) {}
    void CrosswalkBreak::tick() {
        if(EnvModel::crosswalk_distance() == -1) {
            set_state(FAILURE);
        }
        else {
            drive_ros_msgs::TrajectoryMetaInput *msg = new drive_ros_msgs::TrajectoryMetaInput();
            if(EnvModel::num_of_pedestrians() == 0) {
                set_state(FAILURE);
            } else if (current_velocity < speed_zero_tolerance) {
                set_state(SUCCESS);
            }
            else {
                if(EnvModel::crosswalk_distance() < EnvModel::current_break_distance()) {
                    msg->control_metadata = drive_ros_msgs::TrajectoryMetaInput::STANDARD;
                    msg->max_speed = 0;
                }
                else {
                    msg->control_metadata = drive_ros_msgs::TrajectoryMetaInput::STANDARD;
                    msg->max_speed = general_max_speed_cautious;
                    msg_handler.addMessageSuggestion(msg);
                }
            }
        }
    }

    /* ---------- class:CrosswalkWait ---------- */
    CrosswalkWait::CrosswalkWait(std::string name) : BT::ActionNode(name) {}
    void CrosswalkWait::tick() {
        drive_ros_msgs::TrajectoryMetaInput *msg = new drive_ros_msgs::TrajectoryMetaInput();
        //There's nobody at the crosswalk, go on
        if(EnvModel::num_of_pedestrians() == 0) {
            already_waiting = false;
            set_state(SUCCESS);
        }
        //There was (!) a pedestrian on the track, but currently nobody is.
        else if(EnvModel::pedestrians_on_track() == 0 && EnvModel::was_pedestrian_on_track()) {
            set_state(SUCCESS);
        }
        //There's somebody to wait for.
        else {
            msg->control_metadata = msg->STANDARD;
            msg->max_speed = 0;
            msg_handler.addMessageSuggestion(msg);
        }
    }

    /* ---------- class:IntersectionWait ---------- */
    IntersectionWait::IntersectionWait(std::string name) : BT::ActionNode(name) {
        start_waiting = true;
    }
    bool IntersectionWait::waited3sec() {
        return !start_waiting && (ros::Time::now().toSec() - waiting_started.toSec()) > 3.0;
    }
    void IntersectionWait::tick() {
        //Only start waiting when the car has stopped
        if(current_velocity < speed_zero_tolerance && start_waiting) {
            waiting_started = ros::Time::now();
            start_waiting = false;
        }

        //Cases where waiting can be stopped
        if(priority_road
            || (waited3sec() && give_way && EnvModel::intersection_no_object()) 
            || (waited3sec() && !give_way && EnvModel::intersection_no_object_right())) {
            set_state(SUCCESS);
        }
        else {
            drive_ros_msgs::TrajectoryMetaInput *msg = new drive_ros_msgs::TrajectoryMetaInput();
            msg->control_metadata = drive_ros_msgs::TrajectoryMetaInput::STANDARD;
            msg->max_speed = EnvModel::intersection_immediately_upfront() ? 0 : general_max_speed_cautious;
            msg_handler.addMessageSuggestion(msg);
        }
    }

    /* ---------- class:IntersectionDrive ---------- */
    IntersectionDrive::IntersectionDrive(std::string name) : BT::ActionNode(name) {
        started_driving = false;
    }
    void IntersectionDrive::tick() {
        //Set start time at first tick
        if(!started_driving) {
            start_time = ros::Time::now();
            started_driving = true;
        }
        //If driving for long enough, assume the intersection has been crossed.
        if(started_driving && (ros::Time::now().toSec() - start_time.toSec()) > intersection_turn_duration) {
            intersection_turn_indication = 0;
            started_driving = false;
            set_state(SUCCESS);
        }
        else {
            drive_ros_msgs::TrajectoryMetaInput *msg = new drive_ros_msgs::TrajectoryMetaInput();
            //This node is used in both modes, with different requirements
            if(!mode.compare("PARKING")) {
                msg->control_metadata = drive_ros_msgs::TrajectoryMetaInput::STRAIGHT_FORWARD;
            } else {
                msg->control_metadata = intersection_turn_indication == 0 ? 
                    drive_ros_msgs::TrajectoryMetaInput::STRAIGHT_FORWARD : intersection_turn_indication;
            }
            msg->max_speed = intersection_turn_indication == 0 ? general_max_speed_cautious : intersection_turn_speed;
            msg_handler.addMessageSuggestion(msg);
        }
    }

    void trackPropertyCallback(std::vector<BT::TreeNode *> *nodes) {
        msg_handler.evaluate_and_send();
        //Activate nodes when necessary
        //Nodes may only be activated when they are idling. 
        //Of course they can't be activated when already running, but they also shouldn't be when in SUCCESS or FAILURE state.
        
        if((*nodes)[0]->get_state() == IDLE 
            && (EnvModel::object_min_lane_distance(drive_ros_msgs::Lane::RIGHT) < 2 * overtake_distance)) {
            (*nodes)[0]->set_state(RUNNING);
        }
        if((*nodes)[1]->get_state() == IDLE 
            && EnvModel::barred_area_right_distance() != -1 && EnvModel::barred_area_right_distance() < barred_area_react_distance) {
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
    
    void TrackPropertyMessageHandler::addMessageSuggestion(drive_ros_msgs::TrajectoryMetaInput *msg) {
        suggestions.insert(msg);
    }
    void TrackPropertyMessageHandler::evaluate_and_send() {
        //Initialize a minimum speed which already respects speed limits and some parameters (in case no special trackProperty is active)
        float speed = fmin(
            fmin(EnvModel::in_sharp_turn() ? sharp_turn_speed :
                EnvModel::in_very_sharp_turn() ? very_sharp_turn_speed : general_max_speed, speed_limit), on_bridge ? max_bridge_speed : general_max_speed);
        //If nothing else is defined, just follow the track
        int metadata = drive_ros_msgs::TrajectoryMetaInput::STANDARD;
        //Iterate through all suggested messages and update the initialized values if the new ones are more strict.
        for(drive_ros_msgs::TrajectoryMetaInput *msg : suggestions) {
            if(msg->max_speed < speed) speed = msg->max_speed;
            if(metadata == drive_ros_msgs::TrajectoryMetaInput::STANDARD) metadata = msg->control_metadata;
        }
        //Suggestions should only be considered once (if a trackProperty is active in the next cycle, it will send another one anyways).
        suggestions.clear();
        //Build the actual (and final) message
        drive_ros_msgs::TrajectoryMetaInput final_msg;
        final_msg.control_metadata = metadata;
        final_msg.max_speed = speed;
        //...then send it.
        publish_trajectory_metadata(final_msg);
    }

}
