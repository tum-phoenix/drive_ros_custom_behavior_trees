#ifndef TREE_PRINTER_H
#define TREE_PRINTER_H

#include "general.h"
#include "bt_lib/tree_node.h"

namespace BT {

    class TreePrinter {
    public:
        void printTree(TreeNode *headNode, int timeDif);
        void set_clean_output(bool co);
    private:
        bool clean_output;
    };
}

#endif