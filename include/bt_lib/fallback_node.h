#ifndef FALLBACK_NODE_H
#define FALLBACK_NODE_H

#include "general.h"
#include "bt_lib/control_node.h"

namespace BT {
    class FallbackNode : public ControlNode {
    public:
        FallbackNode(std::string name);
        virtual void tick();
        virtual bool reset_state(std::set<std::string> *new_states);
    private:
        int currentChildIndex;
    };
}

#endif