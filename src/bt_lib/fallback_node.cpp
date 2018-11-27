#include "bt_lib/definitions.h"
#include "bt_lib/fallback_node.h"

namespace BT {
    FallbackNode::FallbackNode(std::string name) : ControlNode(name) {
        currentChildIndex = 0;
    }
    void FallbackNode::tick() {
        int newState = IDLE;
        
        if(children.at(currentChildIndex)->get_state() == IDLE) {
            children.at(currentChildIndex)->set_state(RUNNING);
        }

        switch(children.at(currentChildIndex)->get_state()) {
            case FAILURE:
                children.at(currentChildIndex)->set_state(IDLE);
                if(currentChildIndex == children.size() - 1) {
                    newState = FAILURE;
                    currentChildIndex = 0;
                }
                else {
                    newState = RUNNING;
                    currentChildIndex++;
                }
                break;
            case SUCCESS:
                newState = SUCCESS;
                currentChildIndex = 0;
                break;
            case RUNNING:
                newState = RUNNING;
                break;
        }
        set_state(newState);
    }

    //This method needs to be overwritten because the fallback node is performance optimized. 
    //It uses a childIndex instead of iterating all children every tick, which has to be reset.
    bool FallbackNode::reset_state(std::set<std::string> *new_states) {
        bool flag = false;
        for(int i = 0; i < children.size(); i++) {
            if(children.at(i)->reset_state(new_states)) {
                flag = true;
                set_state(RUNNING);
                currentChildIndex = i;
            }
        }
        if(!flag) set_state(IDLE);
        return flag;
    }

}