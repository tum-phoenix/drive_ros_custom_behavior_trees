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

File | Content
:---: | :---:
main.cpp | Builds the tree, reads the launch-file and contains and initializes global data
nodes.cpp / nodes.h | Declarations and definitions of all custom coded nodes (for both models), plus the callback fuction for the parallel "trackProperty"-node and its message sending management.
node_reset.cpp / node_reset.h | Everything needed for state resetting (e.g. after deactivating the RC mode). It wraps everything up in one function which can then easily be called.


environment_model.cpp / environment_model.h | Evaluation of incoming environment data and wrapper functions which can then be called in the nodes' implementation bodies.
ros_communication.cpp / ros_communication.h | All the more "advanced" ROS stuff, like subscribing to and publishing topics, using dynamic_reconfigure etc. It provides simple setup- and usage-functions.
