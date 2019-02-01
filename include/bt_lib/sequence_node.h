#ifndef SEQUENCE_NODE_H
#define SEQUENCE_NODE_H

#include "general.h"

#include "bt_lib/control_node.h"

namespace BT  {

    class SequenceNode : public ControlNode {
    public:
        SequenceNode(std::string name, bool repeatOnSuccess = false, bool skipFailedChild = false);
        virtual void tick();
        virtual bool reset_state(std::set<std::string> *new_states);
    private:
        bool repeatOnSuccess;
        bool skipFailedChild;
        int currentChildIndex;
        void go_to_next_child(int *newState);
        void state_switch(int *newState);
    };

}

#endif