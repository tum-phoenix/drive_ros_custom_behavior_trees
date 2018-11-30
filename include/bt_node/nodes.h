#ifndef NODES_H
#define NODES_H

#include "general.h"

#include "bt_lib/action_node.h"

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

    class FreeDriveIntersection : public BT::ActionNode {
    public:
        FreeDriveIntersection(std::string name);
        void tick();
    };
}

#endif