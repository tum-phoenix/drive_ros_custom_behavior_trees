#ifndef TREE_PRINTER_H
#define TREE_PRINTER_H

#include "general.h"
#include "bt_lib/tree_node.h"

namespace BT {

    class TreePrinter {
    public:
        void printTree(TreeNode *headNode, int timeDif);
    private:
        std::string last_output;
    };
}

#endif