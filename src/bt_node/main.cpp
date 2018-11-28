#include "general.h"
#include <vector>

#include "bt_node/ros_communication.h"
#include "bt_lib/tree.h"
#include "bt_lib/sequence_node.h"
#include "bt_lib/action_node.h"


/* ---------- GLOBAL DATA ---------- */
//Fixed parameters
float break_distance;
std::string mode;

//Dynamic values
bool overtaking_forbidden_zone;
bool express_way;
bool priority_road;
bool on_bridge;
int speed_limit;

void init_global_data(ros::NodeHandle *nh) {
    nh->getParam("behavior_tree/break_distance", break_distance);
    nh->getParam("behavior_tree/mode", mode);

    nh->getParam("behavior_tree/start_value__overtaking_forbidden_zone", overtaking_forbidden_zone);
    nh->getParam("behavior_tree/start_value__express_way", express_way);
    nh->getParam("behavior_tree/start_value__priority_road", priority_road);
    nh->getParam("behavior_tree/start_value__on_bridge", on_bridge);
    nh->getParam("behavior_tree/start_value__speed_limit", speed_limit);

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

    BT::SequenceNode h("Kopf", false);
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

    /*std::set<std::string> s;
    s.insert("Act-node 2");
    t.reset_state(&s);*/

    t.execute();
}