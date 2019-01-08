#ifndef TREE_H
#define TREE_H

#include "general.h"
#include <vector>
#include <chrono>
#include <thread>

#include "bt_lib/definitions.h"
#include "bt_lib/tree_node.h"
#include "bt_lib/tree_printer.h"

namespace BT {
    class Tree {
    public:
        Tree(TreeNode *head, int tick_frequency_ms, bool clean_output);
        int execute();
        void reset_state(std::set<std::string> *new_states);
        TreeNode *get_head();
        void print(int timeDif);
    private:
        std::chrono::milliseconds tick_freq_ms;
        TreeNode *head;
        TreePrinter printer;
    };
} // BT


#endif