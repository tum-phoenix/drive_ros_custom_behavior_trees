#ifndef NODES_H
#define NODES_H

#include "general.h"

#include "bt_lib/action_node.h"
#include "bt_node/ros_communication.h"

#include <chrono>

namespace NODES {
    class WaitForStart : public BT::ActionNode {
    public:
        WaitForStart(std::string name);
        void tick();
    };

    class InitialDriving : public BT::ActionNode {
    public:
        InitialDriving(std::string name);
        void tick();
    };

    class ParkingSpotSearch : public BT::ActionNode {
    public:
        ParkingSpotSearch(std::string name);
        void tick();
    };

    class ParkingBreaking : public BT::ActionNode {
    public:
        ParkingBreaking(std::string name);
        void tick();
    };

    class ParkingInProgress : public BT::ActionNode {
    public:
        ParkingInProgress(std::string name);
        void tick();
    };

    class ParkingReverse : public BT::ActionNode {
    public:
        ParkingReverse(std::string name);
        void tick();
    };

    class FreeDrive : public BT::ActionNode {
    public:
        FreeDrive(std::string name);
        void tick();
    };

    class FreeDriveIntersectionWait : public BT::ActionNode {
    public:
        FreeDriveIntersectionWait(std::string name);
        void tick();
    private:
        bool start_waiting;
        std::chrono::system_clock::time_point waiting_started;
    };

    class SwitchToLeftLane : public BT::ActionNode {
    public:
        SwitchToLeftLane(std::string name);
        void tick();
    };

    class SwitchToRightLane : public BT::ActionNode {
    public:
        SwitchToRightLane(std::string name);
        void tick();
    };

    class FollowingObject : public BT::ActionNode {
    public:
        FollowingObject(std::string name);
        void tick();
    private:
        float last_speed;
    };

    class LeftLaneDrive : public BT::ActionNode {
    public:
        LeftLaneDrive(std::string name);
        void tick();
    };

    class BarredAreaAnticipate : public BT::ActionNode {
    public:
        BarredAreaAnticipate(std::string name);
        void tick();
    };

    class CrosswalkBreak : public BT::ActionNode {
    public:
        CrosswalkBreak(std::string name);
        void tick();
    };

    class CrosswalkWait : public BT::ActionNode {
    public:
        CrosswalkWait(std::string name);
        void tick();
    private:
        int pedestrians_detected_on_track;
    };

    class IntersectionWait : public BT::ActionNode {
    public:
        IntersectionWait(std::string name);
        void tick();
    private:
        bool start_waiting;
        std::chrono::system_clock::time_point waiting_started;
    };

    class IntersectionDrive : public BT::ActionNode {
    public:
        IntersectionDrive(std::string name);
        void tick();
    };



    void trackPropertyCallback(std::vector<BT::TreeNode *> *nodes);

    class TrackPropertyMessageHandler {
    public:
        void addMessageSuggestion(drive_ros_custom_behavior_trees::TrajectoryMessage *msg);
        void evaluate_and_send();
    private:
        std::set<drive_ros_custom_behavior_trees::TrajectoryMessage *> suggestions;
    };
}

#endif