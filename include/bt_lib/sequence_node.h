#ifndef SEQUENCE_NODE_H
#define SEQUENCE_NODE_H

#include "general.h"

#include "bt_lib/control_node.h"

namespace BT  {

class SequenceNode : public ControlNode {
public:
    SequenceNode(std::string name, bool repeatOnSuccess);
    virtual void tick();
    virtual bool reset_state(std::set<std::string> *new_states);
private:
    bool repeatOnSuccess;
    int currentChildIndex;
};

}

#endif