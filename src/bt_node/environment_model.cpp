#include "bt_node/environment_model.h"
#include "bt_node/value_definitions.h"

extern float min_sign_react_distance;
extern float max_sign_react_distance;
extern float intersection_react_distance;
extern float max_start_box_distance;
extern float general_max_speed;

extern int intersection_turn_indication;
extern float speed_limit;
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

    bool f_upfront_object_distance = false;
    float v_upfront_object_distance;
    float upfront_object_distance() {
        if(f_upfront_object_distance) return v_upfront_object_distance;

        float shortest_distance = 100000;
        for(int i = 0; i < env_msg.obj_lane.size(); i++) {
            if(env_msg.obj_lane[i] == get_current_lane() 
                && env_msg.obj_track_distance[i] < shortest_distance
                && env_msg.obj_track_distance[i] > 0) 
                    shortest_distance = env_msg.obj_track_distance[i];
        }
        v_upfront_object_distance = shortest_distance;
        f_upfront_object_distance = true;
        return shortest_distance;
    }

    bool f_barred_area_distance = false;
    float v_barred_area_distance;
    float barred_area_distance() {
        if(f_barred_area_distance) return v_barred_area_distance;

        float d = get_traffic_mark_distance(MARKING_BARRED_AREA);
        v_barred_area_distance = d;
        f_barred_area_distance = true;
        return d;
    }

    float break_distance_to(float target_speed) {
        float d = 0.5 * (current_velocity - target_speed) * (current_velocity - target_speed) / 4; //Pretty accurate
        d *= 1.6; //Safety factor
        return d;
    }
    float current_break_distance() {
        return break_distance_to(0);
    }

    bool f_crosswalk_distance = false;
    float v_crosswalk_distance;
    float crosswalk_distance() {
        if(f_crosswalk_distance) return v_crosswalk_distance;

        float d =  get_traffic_mark_distance(MARKING_BARRED_AREA);
        v_crosswalk_distance = d;
        f_crosswalk_distance = true;
        return d;
    }

    bool f_start_line_distance = false;
    float v_start_line_distance;
    float start_line_distance() {
        if(f_start_line_distance) return v_start_line_distance;

        float d = get_traffic_mark_distance(MARKING_START_LINE);
        v_start_line_distance = d;
        f_start_line_distance = true;
        return d;
    }

    bool start_box_open() {
        return upfront_object_distance() > max_start_box_distance;
    }

    bool object_on_lane(int lane) {
        bool flag = false;
        for(int i = 0; i < env_msg.obj_lane.size(); i++) {
            if(env_msg.obj_lane[i] == lane) flag = true;
        }
        return flag;
    }

    bool f_crosswalk_clear = false;
    bool v_crosswalk_clear;
    bool crosswalk_clear() {
        if(f_crosswalk_clear) return v_crosswalk_clear;
        bool flag = true;
        for(int i = 0; i < env_msg.obj_lane.size(); i++) {
            if(env_msg.obj_lane[i] == LANE_LEFT || env_msg.obj_lane[i] == LANE_RIGHT)
                if(env_msg.obj_track_distance[i] > crosswalk_distance() - 0.1 && env_msg.obj_track_distance[i] < crosswalk_distance() + 0.6)
                    flag = false;
        }
        v_crosswalk_clear = flag;
        f_crosswalk_clear = true;
        return flag;
    }

    int get_current_lane() {
        return env_msg.current_lane;
    }

    bool f_num_of_pedestrians = false;
    int v_num_of_pedestrians;
    int num_of_pedestrians() {
        if(f_num_of_pedestrians) return v_num_of_pedestrians;
        int n = 0;
        for(int i = 0; i < env_msg.traffic_marks_id.size(); i++) {
            if(env_msg.traffic_marks_id[i] == PEDESTRIAN) n++;
        }
        v_num_of_pedestrians = n;
        f_num_of_pedestrians = true;
        return n;
    }

    bool f_intersection_immediately_upfront = false;
    bool v_intersection_immediately_upfront;
    bool intersection_immediately_upfront() {
        if(f_intersection_immediately_upfront) return v_intersection_immediately_upfront;

        bool b = get_traffic_mark_distance(MARKING_INTERSECTION) < intersection_react_distance;
        v_intersection_immediately_upfront = b;
        f_intersection_immediately_upfront = true;
        return b;
    }


    float to_real_speed(int s) {
        return (s / 10) / 3.6;
    }
    void subscriber_callback(const drive_ros_custom_behavior_trees::EnvModelMessage &msg) {
        env_msg = msg;
        //Check for global sign flags to be set
        for(int i = 0; i < msg.traffic_marks_id.size(); i++) {
            switch(msg.traffic_marks_id[i]) {
            case SIGN_SPEED_ZONE_10:
                if(msg.traffic_marks_track_distance[i] < break_distance_to(to_real_speed(10))) {
                    speed_limit = to_real_speed(10);
                }
                break;
            case SIGN_SPEED_ZONE_20:
                if(msg.traffic_marks_track_distance[i] < break_distance_to(to_real_speed(20))) {
                    speed_limit = to_real_speed(20);
                }
                break;
            case SIGN_SPEED_ZONE_30:
                if(msg.traffic_marks_track_distance[i] < break_distance_to(to_real_speed(30))) {
                    speed_limit = to_real_speed(30);
                }
                break;
            case SIGN_SPEED_ZONE_40:
                if(msg.traffic_marks_track_distance[i] < break_distance_to(to_real_speed(40))) {
                    speed_limit = to_real_speed(40);
                }
                break;
            case SIGN_SPEED_ZONE_50:
                if(msg.traffic_marks_track_distance[i] < break_distance_to(to_real_speed(50))) {
                    speed_limit = to_real_speed(50);
                }
                break;
            case SIGN_SPEED_ZONE_60:
                if(msg.traffic_marks_track_distance[i] < break_distance_to(to_real_speed(60))) {
                    speed_limit = to_real_speed(60);
                }
                break;
            case SIGN_SPEED_ZONE_70:
                if(msg.traffic_marks_track_distance[i] < break_distance_to(to_real_speed(70))) {
                    speed_limit = to_real_speed(70);
                }
                break;
            case SIGN_SPEED_ZONE_80:
                if(msg.traffic_marks_track_distance[i] < break_distance_to(to_real_speed(80))) {
                    speed_limit = to_real_speed(80);
                }
                break;
            case SIGN_SPEED_ZONE_90:
                if(msg.traffic_marks_track_distance[i] < break_distance_to(to_real_speed(90))) {
                    speed_limit = to_real_speed(90);
                }
                break;
            case SIGN_SPEED_ZONE_END:
                if(msg.traffic_marks_track_distance[i] < min_sign_react_distance) {
                    speed_limit = general_max_speed;
                }
                break;
            case SIGN_TURN_LEFT:
                intersection_turn_indication = DRIVE_CONTROL_TURN_LEFT;
                break;
            case SIGN_TURN_RIGHT:
                intersection_turn_indication = DRIVE_CONTROL_TURN_RIGHT;
                break;
            }
        }
        //Invalidate all previously computed data
        f_upfront_object_distance = false;
        f_barred_area_distance = false;
        f_crosswalk_distance = false;
        f_start_line_distance = false;
        f_crosswalk_clear = false;
        f_num_of_pedestrians = false;
        f_intersection_immediately_upfront = false;
    }
}
