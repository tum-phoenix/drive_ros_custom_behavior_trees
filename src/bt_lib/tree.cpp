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

        head->tick();

        print();

        std::this_thread::sleep_until(tick_start + tick_freq_ms);
    }
    return 0;
}

void Tree::print() {
    printer.printTree(head);
}

}