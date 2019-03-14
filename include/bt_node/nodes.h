#ifndef NODES_H
#define NODES_H

#include "general.h"

#include "bt_lib/action_node.h"
#include "bt_node/ros_communication.h"


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
        int waiting_status;
        ros::Time waiting_started;
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
    private:
        bool start_waiting;
        ros::Time waiting_started;
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
        bool already_waiting;
        ros::Time waiting_started;
    };

    class IntersectionWait : public BT::ActionNode {
    public:
        IntersectionWait(std::string name);
        void tick();
    private:
        bool waited3sec();
        bool start_waiting;
        ros::Time waiting_started;
    };

    class IntersectionDrive : public BT::ActionNode {
    public:
        IntersectionDrive(std::string name);
        void tick();
    private:
        bool started_driving;
        ros::Time start_time;
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