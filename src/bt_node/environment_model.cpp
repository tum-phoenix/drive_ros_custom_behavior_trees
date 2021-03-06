#include "bt_node/environment_model.h"

#include "drive_ros_msgs/ObstacleEnvironment.h"
#include "drive_ros_msgs/PedestrianEnvironment.h"
#include "drive_ros_msgs/TrafficMarkEnvironment.h"
#include "drive_ros_msgs/Lane.h"
#include "drive_ros_msgs/TrajectoryMetaInput.h"
#include "drive_ros_msgs/value_definitions.h"

#include "general.h"

#include <math.h>
#include <map>

extern float min_sign_react_distance;
extern float max_sign_react_distance;
extern float max_start_box_distance;
extern float max_bridge_speed;
extern float general_max_speed;
extern float break_distance_safety_factor;
extern float intersection_max_obj_distance;

extern bool priority_road;
extern bool force_stop;
extern bool overtaking_forbidden_zone;
extern bool express_way;
extern bool on_bridge;
extern int intersection_turn_indication;
extern float speed_limit;
extern float current_velocity;

extern std::string mode;

namespace EnvModel {
    //Working copy of the most recently received message. Only to be used in the environment_model.cpp!!!
    drive_ros_msgs::EnvironmentModel env_msg;
    //true when currently in pedestrian tracking "mode" and there was a pedestrian on the track at some point
    //set to false when no crosswalk is in sight
    bool pedestrian_on_track = false;

    bool was_pedestrian_on_track() {
        return pedestrian_on_track;
    }

    float get_traffic_mark_distance(int id) {
        for(int i = 0; i < env_msg.traffic_marks.size(); i++) {
            if(env_msg.traffic_marks[i].id == id) return env_msg.traffic_marks[i].track_distance;
        }
        return -1;
    }

    float object_min_lane_distance(int lane) {
        float shortest_distance = 100000;
        for(int i = 0; i < env_msg.obstacles.size(); i++) {
            if(env_msg.obstacles[i].obj_lane.obj_lane == lane
                && env_msg.obstacles[i].obj_track_distance < shortest_distance
                && env_msg.obstacles[i].obj_track_distance > 0) 
                    shortest_distance = env_msg.obstacles[i].obj_track_distance;
        }
        return shortest_distance;
    }

    bool f_barred_area_left_distance = false;
    float v_barred_area_left_distance;
    float barred_area_left_distance() {
        if(f_barred_area_left_distance) return v_barred_area_left_distance;

        float d = -1;
        for(int i = 0; i < env_msg.traffic_marks.size(); i++) {
            if(env_msg.traffic_marks[i].id == MARKING_BARRED_AREA_LEFT
                && (d == -1 || env_msg.traffic_marks[i].track_distance < d)) 
                d = env_msg.traffic_marks[i].track_distance;
        }
        v_barred_area_left_distance = d;
        f_barred_area_left_distance = true;
        return d;
    }

    bool arrived_at_parking_spot() {
        //Collect all constraints
        std::vector<float> constraint_distances;
        for(int i = 0; i < env_msg.obstacles.size(); i++) {
            if(env_msg.obstacles[i].obj_lane.obj_lane == drive_ros_msgs::Lane::RIGHT_SIDE) 
                constraint_distances.push_back(env_msg.obstacles[i].obj_track_distance);
        }
        for(int i = 0; i < env_msg.traffic_marks.size(); i++) {
            if(env_msg.traffic_marks[i].id == MARKING_PARKING_SPOT_BLOCKED 
                && env_msg.traffic_marks[i].obj_lane.obj_lane == drive_ros_msgs::Lane::RIGHT_SIDE)
                constraint_distances.push_back(env_msg.traffic_marks[i].track_distance);
        }

        //Sort them in ascending distance order
        std::sort(constraint_distances.begin(), constraint_distances.end(), std::greater<float>());
        //Try to find a parking spot
        for(int i = 0; i < constraint_distances.size() - 1; i++) {
            if(constraint_distances[i + 1] - constraint_distances[i] > 0.85) {
                return constraint_distances[i + 1] - 1.0 < 0.2;
            }
        }
        return false;
    }

    bool f_barred_area_right_distance = false;
    float v_barred_area_right_distance;
    float barred_area_right_distance() {
        if(f_barred_area_right_distance) return v_barred_area_right_distance;

        float d = -1;
        for(int i = 0; i < env_msg.traffic_marks.size(); i++) {
            if(env_msg.traffic_marks[i].id == MARKING_BARRED_AREA_RIGHT
                && (d == -1 || env_msg.traffic_marks[i].track_distance < d)) 
                d = env_msg.traffic_marks[i].track_distance;
        }
        if(d == -1) d = get_traffic_mark_distance(SIGN_YIELD_ONCOMING_TRAFFIC);
        v_barred_area_right_distance = d;
        f_barred_area_right_distance = true;
        return d;
    }

    bool f_pass_by_on_right_distance = false;
    float v_pass_by_on_right_distance;
    float pass_by_on_right_distance() {
        if(f_pass_by_on_right_distance) return v_pass_by_on_right_distance;

        float d = get_traffic_mark_distance(SIGN_PASS_BY_ON_RIGHT);
        v_pass_by_on_right_distance = d;
        f_pass_by_on_right_distance = true;
        return d;
    }

    bool intersection_no_object() {
        for(int i = 0; i < env_msg.obstacles.size(); i++) {
            if(abs(env_msg.obstacles[i].obj_lateral_offset) < intersection_max_obj_distance) {
                return false;
            }
        }
        return true;
    }

    bool intersection_no_object_right() {
        for(int i = 0; i < env_msg.obstacles.size(); i++) {
            if(env_msg.obstacles[i].obj_lateral_offset > -0.4 
                && env_msg.obstacles[i].obj_lateral_offset < intersection_max_obj_distance) {
                return false;
            }
        }
        return true;
    }

    float break_distance_to(float target_speed) {
        if(target_speed < current_velocity) return break_distance_safety_factor * 0.5 * (current_velocity - target_speed) * (current_velocity - target_speed) / 4;
        else return 0.1;
    }
    float current_break_distance() {
        return break_distance_to(0) + 0.1;
    }

    bool f_crosswalk_distance = false;
    float v_crosswalk_distance;
    float crosswalk_distance() {
        if(f_crosswalk_distance) return v_crosswalk_distance;

        float cd =  get_traffic_mark_distance(MARKING_CROSSWALK);
        float pd = get_traffic_mark_distance(PEDESTRIAN);
        float sd = get_traffic_mark_distance(SIGN_CROSSWALK);
        v_crosswalk_distance = fmax(fmax(cd, pd), sd);
        f_crosswalk_distance = true;
        return v_crosswalk_distance;
    }

    bool f_start_line_distance = false;
    float v_start_line_distance;
    float start_line_distance() {
        if(f_start_line_distance) return v_start_line_distance;

        v_start_line_distance = get_traffic_mark_distance(MARKING_START_LINE);
        f_start_line_distance = true;
        return v_start_line_distance;
    }

    bool f_parking_sign_distance = false;
    float v_parking_sign_distance;
    float parking_sign_distance() {
        if(f_parking_sign_distance) return v_parking_sign_distance;

        v_parking_sign_distance = get_traffic_mark_distance(SIGN_PARKING);
        f_parking_sign_distance = true;
        return v_parking_sign_distance;
    }

    bool f_in_sharp_turn = false;
    bool v_in_sharp_turn;
    bool in_sharp_turn() {
        if(f_in_sharp_turn) return v_in_sharp_turn;
        for(int i = 0; i < env_msg.traffic_marks.size(); i++) {
            if(env_msg.traffic_marks[i].id == SIGN_SHARP_TURN_LEFT || env_msg.traffic_marks[i].id == SIGN_SHARP_TURN_RIGHT)
                if(env_msg.traffic_marks[i].track_distance < 0.5) {
                    v_in_sharp_turn = true;
                    f_in_sharp_turn = true;
                    return true;
                }
        }
        v_in_sharp_turn = false;
        f_in_sharp_turn = true;
        return false;
    }

    bool f_in_very_sharp_turn = false;
    bool v_in_very_sharp_turn;
    bool in_very_sharp_turn() {
        if(f_in_very_sharp_turn) return v_in_very_sharp_turn;
        for(int i = 0; i < env_msg.traffic_marks.size(); i++) {
            if(env_msg.traffic_marks[i].id == SIGN_VERY_SHARP_TURN_LEFT || env_msg.traffic_marks[i].id == SIGN_VERY_SHARP_TURN_RIGHT)
                if(env_msg.traffic_marks[i].track_distance < 0.5) {
                    v_in_very_sharp_turn = true;
                    f_in_very_sharp_turn = true;
                    return true;
                }
        }
        v_in_very_sharp_turn = false;
        f_in_very_sharp_turn = true;
        return false;
    }

    bool start_box_was_closed = false;
    bool start_box_open() {
        if(!start_box_was_closed 
            && ((env_msg.front_distance == 0 ? 10000 : env_msg.front_distance) < max_start_box_distance)) {
            start_box_was_closed = true;
            ROS_INFO("Start box detected");
        }
        return start_box_was_closed && (env_msg.front_distance > max_start_box_distance);
    }

    bool object_on_lane(int lane) {
        for(int i = 0; i < env_msg.obstacles.size(); i++) {
            if(env_msg.obstacles[i].obj_lane.obj_lane == lane) return true;
        }
        return false;
    }

    bool f_crosswalk_clear = false;
    bool v_crosswalk_clear;
    bool crosswalk_clear() {
        if(f_crosswalk_clear) return v_crosswalk_clear;
        bool flag = true;
        for(int i = 0; i < env_msg.obstacles.size(); i++) {
            if(env_msg.obstacles[i].obj_lane.obj_lane == drive_ros_msgs::Lane::LEFT || env_msg.obstacles[i].obj_lane.obj_lane == drive_ros_msgs::Lane::RIGHT)
                if(env_msg.obstacles[i].obj_track_distance > crosswalk_distance() - 0.1 && env_msg.obstacles[i].obj_track_distance < crosswalk_distance() + 0.6)
                    flag = false;
        }
        v_crosswalk_clear = flag;
        f_crosswalk_clear = true;
        return flag;
    }

    int get_current_lane() {
        return env_msg.current_lane;
    }

    bool f_pedestrians_on_track = false;
    int v_pedestrians_on_track;
    int pedestrians_on_track() {
        if(f_pedestrians_on_track) return v_pedestrians_on_track;
        int c = 0;
        for(int i = 0; i < env_msg.pedestrians.size(); i++) {
            if(env_msg.pedestrians[i].obj_lane.obj_lane < 2) c++;
        }
        v_pedestrians_on_track = c;
        f_pedestrians_on_track = true;
    }

    bool f_intersection_immediately_upfront = false;
    bool v_intersection_immediately_upfront;
    bool intersection_immediately_upfront() {
        if(f_intersection_immediately_upfront) return v_intersection_immediately_upfront;

        float yd = get_traffic_mark_distance(MARKING_CROSSING_YIELD);
        float sd = get_traffic_mark_distance(MARKING_CROSSING_STOP);
        float id = get_traffic_mark_distance(MARKING_INTERSECTION);
        float distance = fmin(fmin(yd == -1 ? 10000 : yd, sd == -1 ? 10000 : sd), id == -1 ? 10000 : id);
        float intersect_dist = distance == 10000 ? -1 : distance;
        bool b = (intersect_dist == -1) ? false : intersect_dist < current_break_distance();
        v_intersection_immediately_upfront = b;
        f_intersection_immediately_upfront = true;
        return b;
    }

    int num_of_pedestrians() {
        return env_msg.pedestrians.size();
    }


    float to_real_speed(int s) {
        return (s / 10) / 3.6;
    }
    void subscriber_callback(const drive_ros_msgs::EnvironmentModel &msg) {
        env_msg = msg;
        //Pedestrian tracking
        if(crosswalk_distance() == -1) {
            pedestrian_on_track = false;
        }
        else {
            if(!pedestrian_on_track) {
                for(int i = 0; i < msg.pedestrians.size(); i++) {
                    if(msg.pedestrians[i].obj_lane.obj_lane < 2) {
                        pedestrian_on_track = true;
                        break;
                    }
                }
            }
        }

        //Check for global sign flags to be set
        if(!mode.compare("OBSTACLES")) {
            for(int i = 0; i < msg.traffic_marks.size(); i++) {
                switch(msg.traffic_marks[i].id) {
                case SIGN_SPEED_ZONE_10:
                    if(msg.traffic_marks[i].track_distance < break_distance_to(to_real_speed(10))) {
                        speed_limit = to_real_speed(10);
                    }
                    break;
                case SIGN_SPEED_ZONE_20:
                    if(msg.traffic_marks[i].track_distance < break_distance_to(to_real_speed(20))) {
                        speed_limit = to_real_speed(20);
                    }
                    break;
                case SIGN_SPEED_ZONE_30:
                    if(msg.traffic_marks[i].track_distance < break_distance_to(to_real_speed(30))) {
                        speed_limit = to_real_speed(30);
                    }
                    break;
                case SIGN_SPEED_ZONE_40:
                    if(msg.traffic_marks[i].track_distance < break_distance_to(to_real_speed(40))) {
                        speed_limit = to_real_speed(40);
                    }
                    break;
                case SIGN_SPEED_ZONE_50:
                    if(msg.traffic_marks[i].track_distance < break_distance_to(to_real_speed(50))) {
                        speed_limit = to_real_speed(50);
                    }
                    break;
                case SIGN_SPEED_ZONE_60:
                    if(msg.traffic_marks[i].track_distance < break_distance_to(to_real_speed(60))) {
                        speed_limit = to_real_speed(60);
                    }
                    break;
                case SIGN_SPEED_ZONE_70:
                    if(msg.traffic_marks[i].track_distance < break_distance_to(to_real_speed(70))) {
                        speed_limit = to_real_speed(70);
                    }
                    break;
                case SIGN_SPEED_ZONE_80:
                    if(msg.traffic_marks[i].track_distance < break_distance_to(to_real_speed(80))) {
                        speed_limit = to_real_speed(80);
                    }
                    break;
                case SIGN_SPEED_ZONE_90:
                    if(msg.traffic_marks[i].track_distance < break_distance_to(to_real_speed(90))) {
                        speed_limit = to_real_speed(90);
                    }
                    break;
                case SIGN_SPEED_ZONE_END:
                    if(msg.traffic_marks[i].track_distance < min_sign_react_distance) {
                        speed_limit = 100;
                    }
                    break;
                case SIGN_TURN_LEFT:
                    intersection_turn_indication = drive_ros_msgs::TrajectoryMetaInput::TURN_LEFT;
                    break;
                case SIGN_TURN_RIGHT:
                    intersection_turn_indication = drive_ros_msgs::TrajectoryMetaInput::TURN_RIGHT;
                    break;
                case MARKING_ARROW_LEFT:
                    intersection_turn_indication = drive_ros_msgs::TrajectoryMetaInput::TURN_LEFT;
                    break;
                case MARKING_ARROW_RIGHT:
                    intersection_turn_indication = drive_ros_msgs::TrajectoryMetaInput::TURN_RIGHT;
                    break;
                case SIGN_PRIORITY_ROAD:
                    priority_road = true;
                    break;
                case SIGN_GIVE_WAY:
                    if(msg.traffic_marks[i].track_distance < min_sign_react_distance) {
                        priority_road = false;
                        express_way = false;
                    }
                    break;
                case MARKING_CROSSING_YIELD:
                    priority_road = false;
                    express_way = false;
                    break;
                case SIGN_STOP:
                    if(msg.traffic_marks[i].track_distance < min_sign_react_distance) {
                        force_stop = true;
                        priority_road = false;
                        //Maybe the express_way_end was not detected...
                        express_way = false;
                    }
                    break;
                case MARKING_CROSSING_STOP:
                    force_stop = true;
                    priority_road = false;
                    //Maybe the express_way_end was not detected...
                    express_way = false;
                    break;
                case SIGN_NO_PASSING_ZONE:
                    if(msg.traffic_marks[i].track_distance < min_sign_react_distance) {
                        overtaking_forbidden_zone = true;
                    }
                    break;
                case SIGN_NO_PASSING_ZONE_END:
                    if(msg.traffic_marks[i].track_distance < min_sign_react_distance) {
                        overtaking_forbidden_zone = false;
                    }
                    break;
                case SIGN_EXPRESSWAY_BEGIN:
                    if(msg.traffic_marks[i].track_distance < min_sign_react_distance) {
                        express_way = true;
                        //Should increase robustness, esp. at falsely detected intersections.
                        priority_road = true;
                    }
                    break;
                case SIGN_EXPRESSWAY_END:
                    if(msg.traffic_marks[i].track_distance < min_sign_react_distance) {
                        express_way = false;
                        priority_road = true;
                    }
                    break;
                case SIGN_STEEP_INCLINE:
                    if(msg.traffic_marks[i].track_distance < break_distance_to(max_bridge_speed)) {
                        on_bridge = true;
                    }   
                    break;
                case SIGN_STEEP_DECLINE:
                    //We can already drive faster again when we are starting to leave the bridge.
                    on_bridge = false;
                    break;
                default:
                    break; //All other signs are used differently, e.g. by asking for them directly. These were just passive states.
                }
            }
        }
        //Invalidate all previously computed data
        f_barred_area_left_distance = false;
        f_barred_area_right_distance = false;
        f_pass_by_on_right_distance = false;
        f_crosswalk_distance = false;
        f_start_line_distance = false;
        f_parking_sign_distance = false;
        f_in_sharp_turn = false;
        f_in_very_sharp_turn = false;
        f_crosswalk_clear = false;
        f_pedestrians_on_track = false;
        f_intersection_immediately_upfront = false;
    }
}
