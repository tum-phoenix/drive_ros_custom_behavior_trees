#include "bt_lib/tree.h"
#include "bt_lib/tree_node.h"
#include "bt_lib/tree_printer.h"

#include <vector>

namespace BT {

    Tree::Tree(TreeNode *head, int tick_frequency_ms) {
        this->head = head;
        tick_freq_ms = std::chrono::milliseconds(tick_frequency_ms);
    }

    int Tree::execute() {
        std::chrono::system_clock::time_point tick_start = std::chrono::system_clock::now();

        
        while(ros::ok()) {
            tick_start = std::chrono::system_clock::now();

            if(head->get_state() == IDLE || head->get_state() == RUNNING) head->tick();

            print();

            std::this_thread::sleep_until(tick_start + tick_freq_ms);
        }
        return 0;
    }

    void Tree::reset_state(std::set<std::string> *new_states) {
        head->reset_state(new_states);
    }

    void Tree::print() {
        printer.printTree(head);
    }

}