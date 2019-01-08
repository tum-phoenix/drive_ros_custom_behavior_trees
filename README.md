# Behavior Trees
Behavior Tree implementation of TUM Phoenix Autonomous Drive Team

## WTF are Behavior Trees?!
Behavior Trees can be used for anything from modeling and supervising the current status of the car to a complete control unit taking in processed sensor data and outputting motor commands.

Basically they are trees organizing the possible states of some machine. They use Control Nodes (Sequence nodes, Fallback nodes, Parallel nodes -> usually provided by the lib you use) which specify the way their children are executed, and Action Nodes (-> usually custom coded) which specify how an atomar task is performed. 

All nodes have a a state for themselves (idle, running, success, ...). Of course, a control node can have both control and action nodes as children, while action nodes never have children.

## bt_lib
This is a custom-coded lightweight BT library containing only basic functionality. But therefore, it is also easy to understand and efficient.

### Constructors:
`TreeNode(std::string name)`: Every node will need a string in its constructor. It is the description and identifier for the node, used e.g. for state resetting. *Not to be used when constructing a tree (only a bt_lib-internal constructor).*

`ControlNode(std::string name)`: Control nodes may have children. Their order of execution is defined in the specialized control nodes. Children can be added using the function AddChild(TreeNode \*child). Remember that the order in which children are added will be important for the order of their execution. It can not be changed later. *Not to be used when constructing a tree (only a bt_lib-internal constructor).*

`SequenceNode(std::string name, bool repeatOnSuccess)`: If repeatOnSuccess is false, it will set its state to SUCCESS when all children have succeeded, so the tree can continue. If repeatOnSuccess is true, it will start over again, beginning with the first child. The SequenceNode will only be exited if its state is set to FAILURE.

`ParallelNode(std::string name, bool stayAlive)`: If stayAlive is true, the node will not set its state to SUCCESS if all children have succeeded, otherwise it will.

`FallbackNode(std::string name)`: The FallbackNode sets its state to SUCCESS when one child succeeded. You might always say it executes its children in order until one did not fail.

## bt_node:
This contains the ROS node and the actual BT models. A quick overview:

### File structure
File | Content
:---: | :---:
main.cpp | Builds the tree, reads the launch-file and contains and initializes global data
nodes.cpp / nodes.h | Declarations and definitions of all custom coded nodes (for both models), plus the callback fuction for the parallel "trackProperty"-node and its message sending management.
node_reset.cpp / node_reset.h | Everything needed for state resetting (e.g. after deactivating the RC mode). It wraps everything up in one function which can then easily be called.
environment_model.cpp / environment_model.h | Evaluation of incoming environment data and wrapper functions which can then be called in the nodes' implementation bodies.
ros_communication.cpp / ros_communication.h | All the more "advanced" ROS stuff, like subscribing to and publishing topics, using dynamic_reconfigure etc. It provides simple setup- and usage-functions.

### Launch parameters
The are roughly sorted in categories. Speeds are in m/s ; distances in m

Parameter name | Possible values / variable type | Description
:---: | :---: | :---:
**Uncategorized parameters** | | 
mode | "PARKING", "OBSTACLES" | Specifies the (CaroloCup) driving mode
clean_output | bool | After every cycle some brief information about the state of the tree is printed. If clean_output is true, the first output will be updated, while false will make it output new lines every time.
tick_frequency | int | IMPORTANT: This specifies the desired frequency IN MILLISECONDS. So it's rather a "cycle duration" than a frequency. Example: tick_frequency = 100 will result in ~10 cycles per second.
object_following_break_factor | float | This value should only be changed when testing. Its internal use is to specify how quickly the car breaks when coming too close to the object it is following. The value should therefore be *somewhere* in the region of 1.
break_distance_safety_factor | float | The BT EnvModel is able to approximate the break distance pretty well. Nevertheless, the computed break distance is multiplied with this factor for safety reasons.
**Speeds** | | 
general_max_speed | float | What the car should be allowed to drive on the track (-> it will never go faster than this)
general_max_speed_cautious | float | A max speed for situations where very exact sensing or quick breaking is required (e.g. when approaching a crosswalk but there's no need to radically break yet).
max_bridge_speed | float | Self-explaining. This speed is limited by the level of safety of the car when on the edges of the slopes.
parking_spot_search_speed | float | Self-explaining.
max_lane_switch_speed | float | Self-explaining.
intersection_turn_speed | float | The max speed when taking a 90° turn at an intersection. Has no effect when crossing an intersection.
sharp_turn_speed | float | Possibility to adjust curve speed if necessary; especially if the car doesn't perform too well in curves. This applies to curved roads, no intersection turns.
very_sharp_turn_speed | float | See sharp_turn_speed. This is used in very sharp turns (no bullshit!)
speed_zero_tolerance | If a sensed speed is less than this value it is considered to be zero. This is important for accurate state switching.
**Distances** | | 
min_sign_react_distance | float | currently unused.
max_sign_react_distance | float | currently unused.
overtake_distance | float | Specifies how far the car should be away from an object when attempting to overtake it.
barred_area_react_distance | float | Specifies how far the car should be away from a barred area when attempting to pass it.
oncoming_traffic_clearance | float | Tolerated distance to oncoming traffic which is considered safe. Use: when on the left lane and the distance to the oncoming traffic is smaller than this parameter, the car will try to get back on the right lane again.
max_start_box_distance | float | Used to classify whether the start box is open or not (distance greater than this means it is open).
intersection_min_obj_distance | float | Used to classify whether an object at an intersection is actually on the other lane or in front of the car (further down the track) May be deleted in favor of a better intersection algorithm.
**Start values** | | 
start_value__overtaking_forbidden_zone | bool | Currently in an overtaking forbidden zone?
start_value__express_way | bool | Currently on an express way?
start_value__priority_road | bool | Currently on a priority road?
start_value__force_stop | bool | Used for stop signs. Brings the car to halt.
start_value__on_bridge | bool | Currently on a bridge?
start_value__speed_limit | int | the current speed limit. Note: If not used, set it to a high value (e.g. general_amx_speed) since the car will always adhere to the smallest speed limit given (speed_limit=0 will permanently stop the car)
start_value__successful_parking_count | int | Counts the successful (and therefore correct for the CaroloCup) parking attempts.
start_value__give_way | Set if the car has to give way at an intersection

### Node descriptions
Node | Used in mode (P/O) | Name
:---: | :---: | :---:
WaitForStart | P, O | Stops the car while the start box is closed
InitialDriving | P, O | Avoids confusion when crossing the unusual merge of the short track behind the start gate and the parking zone
IntersectionDrive | P, O | Crosses an intersection. When in Parking/Free Drive mode, it only goes straight forward, otherwise it adheres to the previous turn commands.
ParkingSpotSearch | P | Keeps the car in a cautious speed while looking for a parking spot
ParkingBreaking | P | When a parking spot is found, this node is activated and breaks the car to approx. 0 m/s
ParkingInProgress | P | Active while the car is driving into the found parking spot. When finished, it also registers the successful parking attempt.
ParkingReverse | P | Active while the car is getting back on track.
FreeDrive | P | The standard node for Parking (Free Drive) mode. Makes the car follow the track until an intersection or a start line is detected.
FreeDriveIntersectionWait | P | Stops the car and then waits for 3 seconds.
SwitchToLeftLane | O | Safely switches the car to the left lane. When an object is upfront, it waits.
SwitchToRightLane | O | Switches the car back to the right lane. NO SECURITY CHECKS (yet) IMPLEMENTED!
FollowingObject | O | Follows an object in the context and with the intention of trying to overtake it afterwards. It tries to stay in a 0.3m long interval somewhere behind the object to avoid over-regulating the speed control.
LeftLaneDrive | O | Drives safely on the left side of the road. It's successful when there's no obstruction on the right any more, it also terminates when there's oncoming traffic, a barred area or a traffic island ahead.
BarredAreaAnticipate | O | Stops the car in front of a barred area. Successful when the car is closer than barred_area_react_distance and potential oncoming traffic is far enough away.
CrosswalkBreak | O | Breaks in front of a crosswalk if there are any pedestrians
CrosswalkWait | O | Waits for pedestrians if there are any
IntersectionWait | O | Waits at an intersection. Which means: Not if the car is on a priority road, either for 3 seconds if there is no object or for the object if there is one. The "right of way" implementation MIGHT STILL BE BUGGY

### What is "addMessageSuggestion" / how does the parallel "trackProperty" node only send one message each cycle?
To somehow merge the different control requirements the nodes have, first each active node finds out how it would like to control the car in the normal "tick"-method. These "finished" trajectoryMessages are then added to a collection of messages. 

When all subnodes of the parallel node have entered their request, the method "evaluate_and_send" finds the most strict of all requests and forwards it to the Trajectory Planner. If there is no node running, it has some default values prepared. 

Example: 

The car is following an object on the left lane because there is a barred area on the right. 

The barred area says, the usual speed limit can be applied, but since the object is quite slow, the FollowingObject-node will request a slower speed. Of course, the slower one is then chosen. 

The same applies for control_metadata. When FollowingObject says, STANDARD is enough, but the IntersectionCrossing node requests a TURN_RIGHT, of course, the TURN_RIGHT is then chosen. 

I hope it is now clear.
