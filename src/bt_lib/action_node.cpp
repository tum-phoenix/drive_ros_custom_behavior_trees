#include "bt_lib/definitions.h"
#include "bt_lib/action_node.h"

#include <algorithm>

namespace BT {

    ActionNode::ActionNode(std::string name) : TreeNode(name) {
        
    }

    bool ActionNode::reset_state(std::set<std::string> *new_states) {
        if(new_states->find(get_name()) != new_states->end()) {
            new_states->erase(get_name());
            set_state(RUNNING);
            return true;
        }
        set_state(IDLE);
        return false;
    }

    void ActionNode::currently_running_nodes(std::set<TreeNode *> *nodes) {
        if(get_state() == RUNNING) {
            ROS_INFO_STREAM(get_name());
            nodes->insert(this);
        }
    }

}