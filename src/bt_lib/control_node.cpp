#include "bt_lib/definitions.h"
#include "bt_lib/control_node.h"

namespace BT {

    ControlNode::ControlNode(std::string name) : TreeNode(name) {
        
    }

    void ControlNode::addChild(TreeNode *child) {
        child->set_parent(this);
        children.push_back(child);
    }

    bool ControlNode::reset_state(std::set<std::string> *new_states) {
        bool flag = false;
        for(int i = 0; i < children.size(); i++) {
            if(children.at(i)->reset_state(new_states)) {
                flag = true;
                set_state(RUNNING);
            }
        }
        if(!flag) set_state(IDLE);
        return flag;
    }

    void ControlNode::currently_running_nodes(std::set<TreeNode *> *nodes) {
        for(int i = 0; i < children.size(); i++) {
            if(children.at(i)->get_state() == RUNNING) {
                children.at(i)->currently_running_nodes(nodes);
            }
        }
    }

}