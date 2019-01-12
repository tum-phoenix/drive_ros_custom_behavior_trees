#ifndef ENVIRONMENT_MODEL_H
#define ENVIRONMENT_MODEL_H

#include "drive_ros_msgs/EnvironmentModel.h"

namespace EnvModel {

    bool was_pedestrian_on_track();
    bool intersection_immediately_upfront();
    bool start_box_open();
    bool object_on_lane(int lane);
    bool crosswalk_clear();
    bool in_sharp_turn();
    bool in_very_sharp_turn();
    bool intersection_no_object();
    bool intersection_no_object_right();
    int get_current_lane();
    int num_of_pedestrians();
    int pedestrians_on_track();

    float object_min_lane_distance(int lane);
    float barred_area_distance();
    float pass_by_on_right_distance();
    float current_break_distance();
    float crosswalk_distance();
    float start_line_distance();
    float parking_sign_distance();

    void clear_pedestrian_tracking();
    
    void subscriber_callback(const drive_ros_msgs::EnvironmentModel &msg);
}

#endif