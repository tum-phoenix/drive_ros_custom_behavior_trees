#include "bt_node/environment_model.h"

extern float min_sign_react_distance;
extern float max_sign_react_distance;
extern float intersection_react_distance;

extern float current_velocity;

/*
    TODO: Subscribe to vehicle and RC data, 
    find out if car is on track (again), e.g. after intersection or parking
*/
namespace EnvModel {
    //Only for internal use. NO external variable!!
    drive_ros_custom_behavior_trees::EnvModelMessage env_msg;

    float get_traffic_mark_distance(int id) {
        for(int i = 0; i < env_msg.traffic_marks_id.size(); i++) {
            if(env_msg.traffic_marks_id[i] == id) return env_msg.traffic_marks_track_distance[i];
        }
        return -1;
    }

    float upfront_object_distance() {
        return 0.1;
    }
    float barred_area_distance() {
        return 0;
    }
    float current_break_distance() {
        return current_velocity; //Very rough estimation: break_distance[m] = v[m/s]
    }
    float crosswalk_distance() {
        return get_traffic_mark_distance(102);
    }

    bool start_box_open() {
        return true;
    }
    bool object_on_current_lane() {
        return false;
    }
    bool crosswalk_clear() {
        return true;
    }
    int get_current_lane() {
        return 1;
    }
    int num_of_pedestrians() {
        return 1;
    }


    bool intersection_immediately_upfront() {
        //return get_traffic_mark_distance(0 /*INTERSECTION ID*/) < intersection_react_distance;
        return true;
    }
    void subscriber_callback(const drive_ros_custom_behavior_trees::EnvModelMessage &msg) {
        env_msg = msg;
        //Check for global sign flags to be set
        for(int i = 0; i < msg.traffic_marks_id.size(); i++) {
            if(msg.traffic_marks_track_distance[i] < max_sign_react_distance) {
                switch(msg.traffic_marks_id[i]) {
                //Depending of sign type also check for min_sign_react_distance
                }
            }
        }
    }
}
