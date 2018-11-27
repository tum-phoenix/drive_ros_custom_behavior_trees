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
            newState = RUNNING;
        }
        else if(repeatOnSuccess) {
            currentChildIndex = 0;
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
    this->set_state(newState);
}

}