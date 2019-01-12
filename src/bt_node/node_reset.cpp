#include "bt_node/node_reset.h"

extern std::string mode;

void reset_tree_state(BT::Tree *tree) {
    //Analyse the current state of the tree
    std::set<BT::TreeNode *> currently_running;
    tree->get_head()->currently_running_nodes(&currently_running);
    std::set<std::string> currently_running_names;
    for(BT::TreeNode *tn : currently_running) currently_running_names.insert(tn->get_name());

    std::set<std::string> new_states;
    if(!mode.compare("PARKING")) {
        //We can assume that when RC is disabled again. we are at most JUST IN FRONT OF an intersection, not on it.
        //Therefore, the FreeDrive node will immediately switch to intersection if required.
        for(std::string s : currently_running_names) {
            if(!s.compare("Waiting for gate")) { //Probably the car did not manage to start driving.
                new_states.insert("Initial Driving");
/*          } else if(!s.compare("Initial Driving")) { //Probably transition to normal driving did not work
                new_states.insert("Free Drive"); //Then skip the praking phase for safety reasons.
            } else if(!s.compare("Parking Spot Search")) { //The car may have missed all parking spots and tries to apply parking spot search to the whole track.
                new_states.insert("FreeDrive");
            } else if(!s.compare("Breaking (parking)")) { //Maybe the car won't realize it's already standing.
                new_states.insert("FreeDrive");
            } else if(!s.compare("Parking in progress")) { //The remote control won't leave the car in the middle of a parking process, but will guide the car on track again.
                new_states.insert("FreeDrive");
            }
*/
            } else { //As can be seen above, usually the FreeDrive node is the best choice in a reset situation.
                new_states.insert("Free Drive");
            }
        }
    }
    else if(!mode.compare("OBSTACLES")) {
        for(std::string s : currently_running_names) {
            if(!s.compare("Waiting for gate")) { //Probably the car did not manage to start driving.
                new_states.insert("Initial Driving");
            } else if(!s.compare("Initial Driving")) { //Most likely it didn't exit initial driving state early enough..
                new_states.insert("Track property");
            } else if(!s.compare("Track property")) { //No special track property was being applied
                new_states.insert("Track property"); //The TrackProperty node will evaluate the environment situation itself.
            }
            //Some special track properties were being applied; then check if they should still be active.

        }
    }
    else {
        ROS_ERROR("Driving mode not properly declared. Please check behaviorTree.launch");
        exit(-1);
    }
    tree->reset_state(&new_states);
}
