#ifndef SEQUENCE_NODE_H
#define SEQUENCE_NODE_H

#include "general.h"

#include "bt_lib/control_node.h"

namespace BT  {

class SequenceNode : public ControlNode {
public:
    SequenceNode(std::string name, bool repeatOnSuccess);
    virtual void tick();
private:
    bool repeatOnSuccess;
    int currentChildIndex;
};

}

#endif