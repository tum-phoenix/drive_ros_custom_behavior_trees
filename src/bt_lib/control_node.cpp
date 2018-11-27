#include "bt_lib/definitions.h"
#include "bt_lib/control_node.h"

namespace BT {

ControlNode::ControlNode(std::string name) : TreeNode(name) {
    
}

void ControlNode::addChild(TreeNode *child) {
    child->set_parent(this);
    children.push_back(child);
}

bool ControlNode::reset_state(std::vector<std::string> *new_states) {
    bool flag = false;
    for(int i = 0; new_states->size() > 0 && i < children.size(); i++) {
        if(children.at(i)->reset_state(new_states)) flag = true;
    }
    return flag;
}

std::vector<TreeNode *> ControlNode::currently_running_nodes() {
    std::vector<TreeNode *> v;
    for(int i = 0; i < children.size(); i++) {
        if(children.at(i)->get_state() == RUNNING) {
            std::vector<TreeNode *> res = children.at(i)->currently_running_nodes();
            v.insert(v.end(), res.begin(), res.end());
        }
    }
    return v;
}

}