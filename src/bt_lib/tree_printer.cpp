#include "bt_lib/definitions.h"

#include "bt_lib/tree_printer.h"
#include <algorithm>

namespace BT {

void erase_last_n_lines(int n) {
    for(int i = 0; i < n; i++) {
        std::cout << "\e[2K\r\e[1A";
    }
}

std::string clamp_string_to_length(std::string str, int length) {
    str = str.substr(0, length);
    while(str.size() < length) str += " ";
    return str;
}


void TreePrinter::printTree(TreeNode *tree) {
    std::string next_output;

    next_output = "Current node: ";
    std::set<TreeNode *> running_nodes;
    running_nodes.clear();
    tree->currently_running_nodes(&running_nodes);

    for(auto it : running_nodes) {
        next_output += it->get_name() + " ";
    }
    next_output += "\n";

    if(CLEAN_OUTPUT) erase_last_n_lines(std::count(last_output.begin(), last_output.end(), '\n'));

    std::cout << next_output;
    last_output = next_output;
}

}