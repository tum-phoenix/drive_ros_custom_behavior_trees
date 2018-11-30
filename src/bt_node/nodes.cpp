#include "bt_node/nodes.h"
#include "bt_node/environment_model.h"
#include "bt_node/ros_communication.h"
#include "bt_lib/definitions.h"

extern float max_bridge_speed;
extern float parking_spot_search_speed;
extern int speed_limit;

extern int successful_parking_count;
extern float current_velocity;

namespace NODES {


    drive_ros_custom_behavior_trees::TrajectoryMessage trajectory_msg;

    /* ---------- WaitForStart ---------- */
    WaitForStart::WaitForStart(std::string name) : BT::ActionNode(name) {

    }
    void WaitForStart::tick() {
        if(EnvModel::start_box_open()) {
            set_state(SUCCESS);
        }
        else {
            trajectory_msg.control_metadata = STRAIGHT_FORWARD;
            trajectory_msg.max_speed = 0;
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- InitialDriving ---------- */
    InitialDriving::InitialDriving(std::string name) : BT::ActionNode(name) {

    }
    void InitialDriving::tick() {
        if(true) { //Start line / parking sign detected
            set_state(SUCCESS);
        }
        else {
            trajectory_msg.control_metadata = STRAIGHT_FORWARD;
            trajectory_msg.max_speed = 1000; //Speed doesn't need to be limited 
            publish_trajectory_metadata(trajectory_msg);
        }
    }

    /* ---------- ParkingSpotSearch ---------- */
    ParkingSpotSearch::ParkingSpotSearch(std::string name) : BT::ActionNode(name) {

    }
    void ParkingSpotSearch::tick() {
        if(successful_parking_count >= 2) {
            set_state(FAILURE);
        }
        else if(true) { //Parking spot detected
            set_state(SUCCESS);
        }
        else {
            trajectory_msg.control_metadata = STANDARD;
            trajectory_msg.max_speed = parking_spot_search_speed;
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
        else if(true) { //Start line/Parking sign detected
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
        if(false) { //Back on normal road
            set_state(SUCCESS);
        }
        else { //Maybe implement turn left/right?
            trajectory_msg.control_metadata = STRAIGHT_FORWARD;
            trajectory_msg.max_speed = 1000; //Speed doesn't need to be limited 
            publish_trajectory_metadata(trajectory_msg);
        }
    }
    
    
}