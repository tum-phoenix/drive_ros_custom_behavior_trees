#include "general.h"
#include <vector>

#include "bt_lib/tree.h"
#include "bt_lib/tree_node.h"
#include "bt_lib/sequence_node.h"
#include "bt_lib/action_node.h"

class Act : public BT::ActionNode {
public:
int c;
    Act(std::string name) : ActionNode(name) {
        c = 1;
    }
    void tick() {
        if(c % 5 == 0) set_state(BT::SUCCESS);
        c++;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "BehaviorTree");
    ros::NodeHandle nh;

    BT::SequenceNode h("Kopf", false);
    Act a1("Popf");
    Act a2("Bopfe");

    h.addChild(&a1);
    h.addChild(&a2);

    BT::Tree t(&h, 100);

    t.execute();
}