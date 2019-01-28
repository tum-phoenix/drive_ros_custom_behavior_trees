#include "bt_lib/tree.h"
#include "bt_lib/tree_node.h"
#include "bt_lib/tree_printer.h"

#include <vector>

namespace BT {

    Tree::Tree(TreeNode *head, int tick_frequency_ms, bool clean_output) {
        this->head = head;
        tick_freq_ms = std::chrono::milliseconds(tick_frequency_ms);
        printer.set_clean_output(clean_output);
        ROS_INFO("Tree built");
    }

    int Tree::execute() {
        ROS_INFO("Starting tree execution");
        std::chrono::system_clock::time_point tick_start = std::chrono::system_clock::now();
        
        while(ros::ok()) {
            tick_start = std::chrono::system_clock::now();

            if(head->get_state() == IDLE || head->get_state() == RUNNING) head->tick();

            print(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - tick_start).count());

            std::this_thread::sleep_until(tick_start + tick_freq_ms);
        }
        return 0;
    }

    void Tree::reset_state(std::set<std::string> *new_states) {
        head->reset_state(new_states);
    }

    TreeNode * Tree::get_head() {
        return head;
    }

    void Tree::print(int timeDif) {
        printer.printTree(head, timeDif);
    }

}