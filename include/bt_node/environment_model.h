#ifndef ENVIRONMENT_MODEL_H
#define ENVIRONMENT_MODEL_H

#include "drive_ros_custom_behavior_trees/EnvModelMessage.h"

namespace EnvModel {
    bool intersection_immediately_upfront();
    bool start_box_open();

    float upfront_opject_distance();
    
    void subscriber_callback(const drive_ros_custom_behavior_trees::EnvModelMessage &msg);
}

#endif