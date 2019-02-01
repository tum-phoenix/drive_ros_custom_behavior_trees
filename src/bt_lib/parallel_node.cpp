#include "bt_lib/definitions.h"
#include "bt_lib/parallel_node.h"

namespace BT {

    ParallelNode::ParallelNode(std::string name, void (*activationFunction)(std::vector<TreeNode *> *), bool stayAlive) : ControlNode (name) {
        this->stayAlive = stayAlive;
        this->activationFunction = activationFunction;
    }

    void ParallelNode::tick() {
        //First, the external activationFunction activates children if necessary (using child->set_state(..)).
        activationFunction(&children);

        int newState = RUNNING;
        for(int i = 0; i < children.size(); i++) {
            switch(children.at(i)->get_state()) {
            //If a child fails, the whole node fails - except for when it shall explicitly stay alive anyways.
            case FAILURE:
                children.at(i)->set_state(IDLE);
                if(!stayAlive) newState = FAILURE;
                break;
            //A succeeded child is only being reset if the node shall stay alive - otherwise nothing needs to be done.
            case SUCCESS:
                if(stayAlive) children.at(i)->set_state(IDLE);
                break;
            //Running nodes are ticked, that's it.
            case RUNNING:
                children.at(i)->tick();
                break;
            }
        }

        //Check if all children have already succeeded, then the node succeeds as well
        //IMPORTANT NOTE: stayAlive doesn't need to be checked, because if it's active, succeeded children have already been set to IDLE beforehand.
        bool allSuccess = true;
        for(int i = 0; i < children.size(); i++) {
            allSuccess &= children.at(i)->get_state() == SUCCESS;
        }
        if(allSuccess) {
            for(int i = 0; i < children.size(); i++) {
                children.at(i)->set_state(IDLE);
            }
            newState = RUNNING;
            return;
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
        //If no child is running, the parallel node returns its own name
        if(get_state() == RUNNING && nodes->size() == size_before) nodes->insert(this);
    }

    bool ParallelNode::reset_state(std::set<std::string> *new_states) {
        bool flag = false;
        //A parallel node may as well set itself to running, even if no child will be set to running.
        for(std::string s : *new_states) {
            if(!s.compare(get_name())) {
                set_state(RUNNING);
                flag = true;
            }
        }
        //Then all children are called.
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