#ifndef ACTION_NODE_H
#define ACTION_NODE_H

#include "general.h"

#include "bt_lib/tree_node.h"

namespace BT {

class ActionNode : public TreeNode {
public:
    ActionNode(std::string name);
    virtual bool reset_state(std::set<std::string> *new_states);
    virtual void currently_running_nodes(std::set<TreeNode *> *nodes);
private:
    
};

}
#endif