#ifndef CONTROL_NODE_H
#define CONTROL_NODE_H

#include "general.h"

#include "bt_lib/tree_node.h"

namespace BT {

    class ControlNode : public TreeNode {
    public:
        ControlNode(std::string name);
        void addChild(TreeNode *child);

        virtual bool reset_state(std::set<std::string> *new_states);

        virtual void print_tree(std::string *str, int indent);
        virtual void currently_running_nodes(std::set<TreeNode *> *nodes);
    protected:
        std::vector<TreeNode *> children;
    };

}

#endif