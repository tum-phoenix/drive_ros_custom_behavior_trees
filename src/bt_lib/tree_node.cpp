#include "bt_lib/definitions.h"
#include "bt_lib/tree_node.h"

namespace BT {
TreeNode::TreeNode(std::string name) {
    this->name = name;
    this->state = IDLE;
}

void TreeNode::tick() {
    
}

int TreeNode::get_state() {
    return state;
}
void TreeNode::set_state(int newState) {
    this->state = newState;
}
TreeNode * TreeNode::get_parent() {
    return parent;
}
void TreeNode::set_parent(TreeNode *parent) {
    this->parent = parent;
}
std::string TreeNode::get_name() {
    return name;
}

}