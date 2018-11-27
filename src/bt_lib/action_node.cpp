#include "bt_lib/definitions.h"
#include "bt_lib/action_node.h"

#include <algorithm>

namespace BT {

    ActionNode::ActionNode(std::string name) : TreeNode(name) {
        
    }

    bool remove_if_contained(std::vector<std::string> *v, std::string name) {
        for(int i = 0; i < v->size(); i++) {
            if(!v->at(i).compare(name)) {
                v->erase(v->begin() + i);
                return true;
            }
        }
        return false;
    }

    bool ActionNode::reset_state(std::vector<std::string> *new_states) {
        if(remove_if_contained(new_states, get_name())) {
            set_state(RUNNING);
            return true;
        }
        return false;
    }

    std::vector<TreeNode *> ActionNode::currently_running_nodes() {
        std::vector<TreeNode *> v;
        if(get_state() == RUNNING) {
            v.push_back(this);
        }
        return v;
    }

}