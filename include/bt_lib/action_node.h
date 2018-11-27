#ifndef ACTION_NODE_H
#define ACTION_NODE_H

#include "general.h"

#include "bt_lib/tree_node.h"

namespace BT {

class ActionNode : public TreeNode {
public:
    ActionNode(std::string name);
    virtual bool reset_state(std::vector<std::string> *new_states);
    virtual std::vector<TreeNode *> currently_running_nodes();
private:
    
};

}
#endif