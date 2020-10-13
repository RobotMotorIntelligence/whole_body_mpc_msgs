Whole-body model predictive control ROS message
==============================================

## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Introduction

A whole-body MPC message describes the whole-body state and feedback gains. The whole-body state is broken in three pieces: centroidal, joint and contact state, and it is described in: [whole_body_state_msgs](https://github.com/loco-3d/whole_body_state_msgs).
The feedback gain is stored in data vector with a predefined stored dimensions, i.e. `nx` and `nu`.

With this basic message, we could connect with any whole-body controller. Additionally, it defines a whole-body control message which contains desired and actual states.

## :penguin: Building

The whole_body_mpc_msgs is a catkin project which can be built as:

	cd your_ros_ws/
	catkin build #catkin_make

## :copyright: Credits

### :writing_hand: Written by

- [Carlos Mastalli](https://cmastalli.github.io/), The University of Edinburgh :uk:
