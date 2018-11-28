#include "bt_lib/definitions.h"

#include "bt_lib/sequence_node.h"

namespace BT {

    SequenceNode::SequenceNode(std::string name, bool repeatOnSuccess = false) : ControlNode(name) {
        this->repeatOnSuccess = repeatOnSuccess;
        currentChildIndex = 0;
    }

    void SequenceNode::tick() {
        int newState = IDLE;
        
        if(children.at(currentChildIndex)->get_state() == IDLE) {
            children.at(currentChildIndex)->set_state(RUNNING);
        }
        
        switch(children.at(currentChildIndex)->get_state()) {
            case FAILURE:
                newState = FAILURE;
                children.at(currentChildIndex)->set_state(IDLE);
                currentChildIndex = 0;
                break;
            case SUCCESS:
                children.at(currentChildIndex)->set_state(IDLE);
                if(currentChildIndex < children.size() - 1) {
                    currentChildIndex++;
                    children.at(currentChildIndex)->set_state(RUNNING);
                    newState = RUNNING;
                }
                else if(repeatOnSuccess) {
                    currentChildIndex = 0;
                    children.at(currentChildIndex)->set_state(RUNNING);
                    newState = RUNNING;
                }
                else {
                    newState = SUCCESS;
                    currentChildIndex = 0;
                }
                break;
            case RUNNING:
                children.at(currentChildIndex)->tick();
                newState = RUNNING;
                break;
        }
        set_state(newState);
    }

    //This method needs to be overwritten because the sequence node is performance optimized. 
    //It uses a childIndex instead of iterating all children every tick, which has to be reset.
    bool SequenceNode::reset_state(std::set<std::string> *new_states) {
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