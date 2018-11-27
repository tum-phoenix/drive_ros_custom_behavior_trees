#ifndef CONTROL_NODE_H
#define CONTROL_NODE_H

#include "general.h"

#include "bt_lib/tree_node.h"

namespace BT {

class ControlNode : public TreeNode {
public:
    ControlNode(std::string name);
    void addChild(TreeNode *child);

    virtual bool reset_state(std::vector<std::string> *new_states);
    virtual std::vector<TreeNode *> currently_running_nodes();
protected:
    std::vector<TreeNode *> children;
};

}

#endif