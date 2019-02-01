#include "bt_lib/definitions.h"

#include "bt_lib/sequence_node.h"

namespace BT {

    SequenceNode::SequenceNode(std::string name, bool repeatOnSuccess, bool skipFailedChild) : ControlNode(name) {
        this->repeatOnSuccess = repeatOnSuccess;
        this->skipFailedChild = skipFailedChild;
        currentChildIndex = 0;
    }


    void SequenceNode::tick() {
        int newState = IDLE;
        
        if(children.at(currentChildIndex)->get_state() == IDLE) {
            children.at(currentChildIndex)->set_state(RUNNING);
        }
        
        state_switch(&newState);
        
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

    void SequenceNode::state_switch(int *newState) {
        switch(children.at(currentChildIndex)->get_state()) {
            //Usually, a failed child leads to failure of the whole sequence node.
            case FAILURE:
                //In case this special property is active, the tree goes on anyways, just as if it would have succeeded.
                if(skipFailedChild) {
                    go_to_next_child(newState);
                } 
                //Otherwise, the node itself fails and resets the child counter.
                else {
                    *newState = FAILURE;
                    children.at(currentChildIndex)->set_state(IDLE);
                    currentChildIndex = 0;
                }
                break;
            //When a child succeeded it just transitions to the next child.
            case SUCCESS:
                go_to_next_child(newState);
                break;
            //Usually, running nodes are only ticked and the node remains RUNNING.
            case RUNNING:
                children.at(currentChildIndex)->tick();
                *newState = RUNNING;
                //But when the child succeeded or failed this time, immediate state-reevaluation is called.
                if(children.at(currentChildIndex)->get_state() == SUCCESS 
                    || children.at(currentChildIndex)->get_state() == FAILURE)
                    state_switch(newState);
                break;
        }
    }


    void SequenceNode::go_to_next_child(int *newState) {
        children.at(currentChildIndex)->set_state(IDLE);
        //If it wasn't the last child, just go to the next one.
        if(currentChildIndex < children.size() - 1) {
            currentChildIndex++;
            children.at(currentChildIndex)->set_state(RUNNING);
            *newState = RUNNING;
            state_switch(newState);
        }
        //If it was, but the node shall repeat, it activates the first child again.
        else if(repeatOnSuccess) {
            currentChildIndex = 0;
            children.at(currentChildIndex)->set_state(RUNNING);
            *newState = RUNNING;
            state_switch(newState);
        }
        //Last child succeeded and the node shall not repeat
        else {
            *newState = SUCCESS;
            currentChildIndex = 0;
        }
    }
}