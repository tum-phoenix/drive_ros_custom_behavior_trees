#include "bt_lib/definitions.h"

#include "bt_lib/tree_printer.h"
#include <algorithm>

extern bool output_show_computation_time;
extern float break_distance;
extern std::string mode;

extern bool overtaking_forbidden_zone;
extern bool express_way;
extern bool priority_road;
extern bool force_stop;
extern bool on_bridge;
extern bool give_way;
extern int successful_parking_count;
extern int intersection_turn_indication;

extern float speed_limit;
extern float current_velocity;

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
    std::string last_output = "";

    void TreePrinter::printTree(TreeNode *tree, int timeDif) {
        int var_print_width = 32;
        std::string next_output = "";
        if(clean_output) erase_last_n_lines(&next_output, last_output_length);

        //Header
        next_output += "\n\n"; //A little distance to tree-generation related output
        next_output += "----------------------------------------------------------------\n";
        next_output += "Live-updated behavior tree status";
        if(output_show_computation_time) next_output += "(Computation time: " + std::to_string(timeDif) + " ns)";
        next_output += "\n";
        next_output += "----------------------------------------------------------------\n";
        set_color(&next_output, 30, 47);

        //VARIABLES, TWO IN EACH ROW
        //Boolean variables
        next_output += change_string_to_length("Global data (runtime variables):", 2 * var_print_width);
        set_color(&next_output, 0, 0);
        next_output += "\n";
        set_color(&next_output, overtaking_forbidden_zone ? 32 : 31, 47);
        next_output += change_string_to_length("overtaking_forbidden_zone", var_print_width);
        set_color(&next_output, express_way ? 32 : 31, 47);
        next_output += change_string_to_length("express_way", var_print_width);
        set_color(&next_output, 0, 0);
        next_output += "\n";
        set_color(&next_output, priority_road ? 32 : 31, 47);
        next_output += change_string_to_length("priority_road", var_print_width);
        set_color(&next_output, force_stop ? 32 : 31, 47);
        next_output += change_string_to_length("force_stop", var_print_width);
        set_color(&next_output, 0, 0);
        next_output += "\n";
        set_color(&next_output, give_way ? 32 : 31, 47);
        next_output += change_string_to_length("give_way", var_print_width);
        set_color(&next_output, on_bridge ? 32 : 31, 47);
        next_output += change_string_to_length("on_bridge", var_print_width);
        set_color(&next_output, 0, 0);
        next_output += "\n";

        //Number variables
        set_color(&next_output, 30, 47);
        next_output += change_string_to_length("successful_parking_count " + std::to_string(successful_parking_count), var_print_width);
        next_output += change_string_to_length("intersection_turn_indication " + std::to_string(intersection_turn_indication), var_print_width);
        set_color(&next_output, 0, 0);
        next_output += "\n";
        set_color(&next_output, 30, 47);
        next_output += change_string_to_length("speed_limit " + std::to_string(speed_limit), var_print_width);
        next_output += change_string_to_length("current_velocity " + std::to_string(current_velocity).substr(0, 3), var_print_width);
        set_color(&next_output, 0, 0);
        next_output += "\n\n";

        next_output += "----------------------------------------------------------------\n";
        next_output += "Behavior Tree Model:\n";
        next_output += "----------------------------------------------------------------\n\n";
        
        //The tree itself
        set_color(&next_output, 0, 0);
        tree->print_tree(&next_output, 0);

        //Print everything
        if(next_output.compare(last_output)) {
            std::cout << next_output;
            last_output = next_output;
            last_output_length = number_of_lines(next_output);
        }
    }

    void TreePrinter::set_clean_output(bool co) {
        clean_output = co;
    }

}