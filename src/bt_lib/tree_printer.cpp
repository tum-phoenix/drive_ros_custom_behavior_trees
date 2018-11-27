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
    std::vector<TreeNode *> running_nodes = tree->currently_running_nodes();

    for(int i = 0; i < running_nodes.size(); i++) {
        next_output += running_nodes.at(i)->get_name() + " ";
    }
    next_output += "\n";

    erase_last_n_lines(std::count(last_output.begin(), last_output.end(), '\n'));

    std::cout << next_output;
    last_output = next_output;
}

}