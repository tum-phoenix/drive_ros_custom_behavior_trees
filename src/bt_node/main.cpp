#include "general.h"
#include <vector>

#include "bt_node/ros_communication.h"
#include "bt_node/environment_model.h"
#include "bt_lib/tree.h"
#include "bt_lib/sequence_node.h"
#include "bt_lib/action_node.h"

#include "bt_node/nodes.h"


/* ---------- GLOBAL DATA ---------- */
//Fixed parameters
std::string mode;
int tick_frequency;
float min_sign_react_distance;
float max_sign_react_distance;
float intersection_react_distance;
float max_bridge_speed;
float parking_spot_search_speed;

//Dynamic values
bool overtaking_forbidden_zone;
bool express_way;
bool priority_road;
bool on_bridge;
int speed_limit;
int successful_parking_count;
float current_velocity = 0;

void init_global_data(ros::NodeHandle *nh) {
    nh->getParam("behavior_tree/mode", mode);
    nh->getParam("behavior_tree/tick_frequency", tick_frequency);
    nh->getParam("behavior_tree/min_sign_react_distance", min_sign_react_distance);
    nh->getParam("behavior_tree/max_sign_react_distance", max_sign_react_distance);
    nh->getParam("behavior_tree/intersection_react_distance", intersection_react_distance);
    nh->getParam("behavior_tree/max_bridge_speed", max_bridge_speed);
    nh->getParam("behavior_tree/parking_spot_search_speed", parking_spot_search_speed);

    nh->getParam("behavior_tree/start_value__overtaking_forbidden_zone", overtaking_forbidden_zone);
    nh->getParam("behavior_tree/start_value__express_way", express_way);
    nh->getParam("behavior_tree/start_value__priority_road", priority_road);
    nh->getParam("behavior_tree/start_value__on_bridge", on_bridge);
    nh->getParam("behavior_tree/start_value__speed_limit", speed_limit);
    nh->getParam("behavior_tree/start_value__successful_parking_coung", successful_parking_count);

}
/* ------- END OF GLOBAL DATA ------ */


class Act : public BT::ActionNode {
public:
    int c;
    Act(std::string name) : ActionNode(name) {
        c = 1;
    }
    void tick() {
        drive_ros_custom_behavior_trees::TrajectoryMessage msg;
        msg.max_speed = speed_limit;
        msg.control_metadata = 2;
        publish_trajectory_metadata(msg);
        if(c % 5 == 0) set_state(SUCCESS);
        c++;
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "BehaviorTree");
    ros::NodeHandle nh;
    setup_ros_communication(&nh);
    init_global_data(&nh);

    BT::SequenceNode *head = new BT::SequenceNode("CaroloCup2019", false);
    if(!mode.compare("PARKING")) {
        NODES::WaitForStart *node_waitForStart = new NODES::WaitForStart("Waiting for gate");
        NODES::InitialDriving *node_initialDriving = new NODES::InitialDriving("Initial Driving");
        BT::SequenceNode *node_doCourse = new BT::SequenceNode("Course loop", true);

        BT::SequenceNode *node_parkingPending = new BT::SequenceNode("Parking", false);
        BT::SequenceNode *node_driving = new BT::SequenceNode("Driving", true);

        NODES::ParkingSpotSearch *node_parkingSpotSearch = new NODES::ParkingSpotSearch("Parking Spot Search");
        NODES::ParkingBreaking *node_parkingBreaking = new NODES::ParkingBreaking("Breaking (parking)");
        NODES::ParkingInProgress *node_parkingInProgress = new NODES::ParkingInProgress("Parking in progress");
        NODES::ParkingReverse *node_parkingReverse = new NODES::ParkingReverse("Parking reverse");

        NODES::FreeDrive *node_freeDrive = new NODES::FreeDrive("Free Drive");
        NODES::FreeDriveIntersection *node_freeDriveIntersection = new NODES::FreeDriveIntersection("Crossing intersection");

        head->addChild(node_waitForStart);
        head->addChild(node_initialDriving);
        head->addChild(node_doCourse);

        node_doCourse->addChild(node_parkingPending);
        node_doCourse->addChild(node_driving);

        node_parkingPending->addChild(node_parkingSpotSearch);
        node_parkingPending->addChild(node_parkingBreaking);
        node_parkingPending->addChild(node_parkingInProgress);
        node_parkingPending->addChild(node_parkingReverse);

        node_doCourse->addChild(node_freeDrive);
        node_doCourse->addChild(node_freeDriveIntersection);
    }
    else if(!mode.compare("OBSTACLES")) {
        NODES::WaitForStart *node_waitForStart = new NODES::WaitForStart("Waiting for gate");
        NODES::InitialDriving *node_initialDriving = new NODES::InitialDriving("Initial Driving");

        head->addChild(node_waitForStart);
        head->addChild(node_initialDriving);
    }
    else {
        ROS_ERROR("Driving mode not properly declared. Please check behaviorTree.launch");
        return -1;
    }

    BT::Tree *tree = new BT::Tree(head, tick_frequency);
    tree->execute();

    /*BT::SequenceNode h("Kopf", false);
    BT::SequenceNode sn("Sequence", true);
    Act a1("Act-node 1");
    Act a2("Act-node 2");
    Act a3("Act-node 3");
    Act a4("Act-node 4");

    h.addChild(&a1);
    h.addChild(&a2);
    h.addChild(&sn);
    sn.addChild(&a3);
    sn.addChild(&a4);

    BT::Tree t(&h, 100);

    std::set<std::string> s;
    s.insert("Act-node 2");
    t.reset_state(&s);

    t.execute();*/
}