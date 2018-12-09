#ifndef ENVIRONMENT_MODEL_H
#define ENVIRONMENT_MODEL_H

#include "drive_ros_custom_behavior_trees/EnvModelMessage.h"

namespace EnvModel {
    bool intersection_immediately_upfront();
    bool start_box_open();
    bool object_on_lane(int lane);
    bool crosswalk_clear();
    int get_current_lane();
    int num_of_pedestrians();

    float upfront_object_distance();
    float barred_area_distance();
    float current_break_distance();
    float crosswalk_distance();
    float start_line_distance();
    
    void subscriber_callback(const drive_ros_custom_behavior_trees::EnvModelMessage &msg);
}

#endif