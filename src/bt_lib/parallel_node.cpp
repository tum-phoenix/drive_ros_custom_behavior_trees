#include "bt_lib/definitions.h"
#include "bt_lib/parallel_node.h"

namespace BT {

    ParallelNode::ParallelNode(std::string name, void (*activationFunction)(std::vector<TreeNode *> *), bool stayAlive = true) : ControlNode (name) {
        this->stayAlive = stayAlive;
        this->activationFunction = activationFunction;
    }

    void ParallelNode::tick() {
        activationFunction(&children);

        int newState = RUNNING;
        for(int i = 0; i < children.size(); i++) {
            switch(children.at(i)->get_state()) {
            case FAILURE:
                children.at(i)->set_state(IDLE);
                if(!stayAlive) newState = FAILURE;
                break;
            case SUCCESS:
                if(stayAlive) children.at(i)->set_state(IDLE);
                break;
            case RUNNING:
                children.at(i)->tick();
                break;
            }
        }
        if(!stayAlive) {
            bool allSuccess = true;
            for(int i = 0; i < children.size(); i++) {
                allSuccess &= children.at(i)->get_state() == SUCCESS;
            }
            if(allSuccess) {
                for(int i = 0; i < children.size(); i++) {
                    children.at(i)->set_state(IDLE);
                }
                set_state(SUCCESS);
                return;
            }
        }
        set_state(newState);
    }

}