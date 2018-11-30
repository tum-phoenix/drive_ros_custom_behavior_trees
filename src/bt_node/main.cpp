#include "general.h"
#include <vector>

#include "bt_node/ros_communication.h"
#include "bt_lib/tree.h"
#include "bt_lib/sequence_node.h"
#include "bt_lib/action_node.h"


/* ---------- EXTERNAL DATA ---------- */
std::string mode;

void init_external_data(ros::NodeHandle *nh) {
    nh->getParam("behavior_tree/mode", mode);
}
/* ------- END OF EXTERNAL DATA ------ */


class Act : public BT::ActionNode {
public:
    int c;
    Act(std::string name) : ActionNode(name) {
        c = 1;
    }
    void tick() {
        if(c % 5 == 0) set_state(SUCCESS);
        c++;
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "BehaviorTree");
    ros::NodeHandle nh;
    RosInterface ros_interface(nh);
    init_external_data(&nh);


ROS_INFO_STREAM(mode);
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
