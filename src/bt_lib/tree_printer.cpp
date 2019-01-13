#include "bt_lib/definitions.h"

#include "bt_lib/tree_printer.h"
#include <algorithm>

extern float break_distance;
extern std::string mode;

extern bool overtaking_forbidden_zone;
extern bool express_way;
extern bool priority_road;
extern bool on_bridge;
extern int speed_limit;

namespace BT {

    void erase_last_n_lines(std::string *s, int n) {
        for(int i = 0; i < n; i++) {
            *s += "\e[2K\r\e[1A";
        }
    }

    void set_color(std::string *s, int foreground, int background) {
        *s += "\e[" + std::to_string(foreground) + "m\e[" + std::to_string(background) + "m";
    }

    std::string change_string_to_length(std::string str, int length) {
        str = str.substr(0, length);
        while(str.size() < length) str += " ";
        return str;
    }

    int number_of_lines(std::string str) {
        int c = 0;
        for(int i = 0; i < str.size(); i++) {
            if(str.at(i) == '\n') c++;
        }
        return c;
    }

    int last_output_length = 0;

    void TreePrinter::printTree(TreeNode *tree, int timeDif) {
        int var_print_width = 32;
        std::string next_output = "";

        next_output += "Live-updated tree status (Computation time: " + std::to_string(timeDif) + " ns)\n";
        set_color(&next_output, 30, 47);
        next_output += change_string_to_length("Flags:", var_print_width);
        set_color(&next_output, 0, 0);
        next_output += "\n";
        set_color(&next_output, overtaking_forbidden_zone ? 32 : 31, 47);
        next_output += change_string_to_length("overtaking_forbidden_zone", var_print_width);
        set_color(&next_output, 0, 0);
        next_output += "\n";
        set_color(&next_output, express_way ? 32 : 31, 47);
        next_output += change_string_to_length("express_way", var_print_width);
        set_color(&next_output, 0, 0);
        next_output += "\n";
        set_color(&next_output, priority_road ? 32 : 31, 47);
        next_output += change_string_to_length("priority_road", var_print_width);
        set_color(&next_output, 0, 0);
        next_output += "\n";
        set_color(&next_output, on_bridge ? 32 : 31, 47);
        next_output += change_string_to_length("on_bridge", var_print_width);
        set_color(&next_output, 0, 0);
        next_output += "\n\n";

        set_color(&next_output, 30, 47);
        next_output += change_string_to_length("Other values:", var_print_width);
        set_color(&next_output, 0, 0);
        next_output += "\n";
        set_color(&next_output, 30, 47);
        next_output += change_string_to_length("speed_limit " + std::to_string(speed_limit), var_print_width);
        set_color(&next_output, 0, 0);
        next_output += "\n";

        set_color(&next_output, 0, 0);
        if(clean_output) erase_last_n_lines(&next_output, last_output_length);
        tree->print_tree(&next_output, 0);
        last_output_length = number_of_lines(next_output);

        std::cout << next_output;
        last_output = next_output;
    }

    void TreePrinter::set_clean_output(bool co) {
        clean_output = co;
    }

}