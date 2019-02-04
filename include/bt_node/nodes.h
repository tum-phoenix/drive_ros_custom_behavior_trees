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
    private:
        bool clock_started;
        std::chrono::system_clock::time_point driving_start;
    };

    /* "OLD" PARKING SUBTREE */

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
    private:
        bool start_waiting;
        std::chrono::system_clock::time_point waiting_started;
    };

    class ParkingReverse : public BT::ActionNode {
    public:
        ParkingReverse(std::string name);
        void tick();
    private:
        bool start_waiting;
        std::chrono::system_clock::time_point waiting_started;
    };

    /* "NEW" PARKING SUBTREE */

    class Parking : public BT::ActionNode {
    public:
        Parking(std::string name);
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
        int start_waiting;
        std::chrono::system_clock::time_point waiting_started;
    };

    class SwitchToLeftLane : public BT::ActionNode {
    public:
        SwitchToLeftLane(std::string name);
        void tick();
    private:
        bool start_waiting;
        std::chrono::system_clock::time_point waiting_started;
    };

    class SwitchToRightLane : public BT::ActionNode {
    public:
        SwitchToRightLane(std::string name);
        void tick();
    private:
        bool start_waiting;
        std::chrono::system_clock::time_point waiting_started;
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
    private:
        bool start_waiting;
        std::chrono::system_clock::time_point waiting_started;
    };

    class BarredAreaAnticipate : public BT::ActionNode {
    public:
        BarredAreaAnticipate(std::string name);
        void tick();
    };

    /* "OLD" CROSSWALK SUBTREE */
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
        bool already_waiting;
        std::chrono::system_clock::time_point waiting_started;
    };

    /* "NEW" CROSSWALK SUBTREE */
    class Crosswalk : public BT::ActionNode {
    public:
        Crosswalk(std::string name);
        void tick();
    private:
        bool already_waiting;
        std::chrono::system_clock::time_point waiting_started;
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
    private:
        bool started_driving;
        std::chrono::system_clock::time_point start_time;
    };



    void trackPropertyCallback(std::vector<BT::TreeNode *> *nodes);

    class TrackPropertyMessageHandler {
    public:
        void addMessageSuggestion(drive_ros_msgs::TrajectoryMetaInput *msg);
        void evaluate_and_send();
    private:
        std::set<drive_ros_msgs::TrajectoryMetaInput *> suggestions;
    };
}

#endif