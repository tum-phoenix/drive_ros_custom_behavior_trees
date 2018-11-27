#ifndef PARALLEL_NODE_H
#define PARALLEL_NODE_H

#include "general.h"

#include "bt_lib/tree_node.h"
#include "bt_lib/control_node.h"

namespace BT {

class ParallelNode : public ControlNode {
public:
    ParallelNode(std::string name, void (*activationFunction)(std::vector<TreeNode *> *), bool stayAlive);
    virtual void tick();
private:
    bool stayAlive;
    void (*activationFunction)(std::vector<TreeNode *> *);
};

}

#endif