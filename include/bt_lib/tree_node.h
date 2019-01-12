#ifndef TREE_NODE_H
#define TREE_NODE_H

#include "general.h"
#include <vector>

namespace BT
{

    class TreeNode {
    public:
        TreeNode(std::string name);
        virtual void tick();
        virtual bool reset_state(std::set<std::string> *new_states) = 0;
        int get_state();
        void set_state(int newState);
        TreeNode *get_parent();
        void set_parent(TreeNode *parent);
        std::string get_name();

        virtual void print_tree(std::string *str, int indent) = 0;
        virtual void currently_running_nodes(std::set<TreeNode *> *nodes) = 0;
    private:
        int state;
        std::string name;
        TreeNode *parent;
    };

} // BT

#endif