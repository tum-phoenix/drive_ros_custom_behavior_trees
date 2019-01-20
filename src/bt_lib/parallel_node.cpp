#include "bt_lib/definitions.h"
#include "bt_lib/parallel_node.h"

namespace BT {

    ParallelNode::ParallelNode(std::string name, void (*activationFunction)(std::vector<TreeNode *> *), bool stayAlive) : ControlNode (name) {
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

    void ParallelNode::currently_running_nodes(std::set<TreeNode *> *nodes) {
        int size_before = nodes->size();
        for(int i = 0; i < children.size(); i++) {
            if(children.at(i)->get_state() == RUNNING) {
                children.at(i)->currently_running_nodes(nodes);
            }
        }
        if(nodes->size() == size_before) nodes->insert(this);
    }

    bool ParallelNode::reset_state(std::set<std::string> *new_states) {
        bool flag = false;
        //A parallel node may as well set itself to running, even if no child will be.
        for(std::string s : *new_states) {
            if(!s.compare(get_name())) {
                set_state(RUNNING);
                flag = true;
            }
        }
        for(int i = 0; i < children.size(); i++) {
            if(children.at(i)->reset_state(new_states)) {
                flag = true;
                set_state(RUNNING);
            }
        }
        if(!flag) set_state(IDLE);
        return flag;
    } 
}