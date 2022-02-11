# safe_robot

A low-level ROS package for the safe operation of robots. Easily setup
with a single launch file. The `safe_robot_node.py` acts as a
remapper. Target joint states are passed through several safety
checks, if safe then the command is sent to the robot, otherwise they
are prevented. Possible checks
* joint position limits
* joint velocity limits
* end-effector/link box limits
* self-collision check

# Node

## `safe_robot_node.py`

### Subscribed topics

* `joint_states/target` ([sensor_msgs/JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html))

    The target joint state.

### Published topics

* `joint_states/command` ([sensor_msgs/JointState](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html) or [std_msgs/Float64MultiArray](http://docs.ros.org/en/api/std_msgs/html/msg/Float64MultiArray.html))

    The commanded joint state. Note, optionally this can be sent as a `sensor_msgs/JointState` message or `std_msgs/Float64MultiArray`.

* `robot_safety/status` ([diagnostic_msgs/DiagnosticStatus](http://docs.ros.org/en/api/diagnostic_msgs/html/msg/DiagnosticStatus.html))

    Status of the joint state.

* (optional) `safe_robot/box_limit/marker` ([visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))

    Optional, marker that shows the box limit for the end-effectors/links. This is only published when debug-mode is turned on (see below).

### Parameters

* `~exotica_xml_filename` (string)

    An [EXOTica](https://github.com/ipab-slmc/exotica) XML filename. This should point to a URDF and SRDF file.

* `~link_xlim` (string)

    Two floats separated by a space that specifies the end-effector/link limits in the x-axis for the world frame.

* `~link_ylim` (string)

    Two floats separated by a space that specifies the end-effector/link limits in the x-axis for the world frame.

* `~link_zlim` (string)

    Two floats separated by a space that specifies the end-effector/link limits in the x-axis for the world frame.

* `~safe_links` (string)

    A list of links that should be bound with the box limits, each link name should appear in the URDF or EXOTica XML config file. Every name should be separated by a space.

* `~joint_position_limit_factor` (double, default: 0.95, min: 0.0, max: 1.0)

    The joint position limits (from URDF) are multipled by a factor to determine safe operation.

* `~joint_velocity_limit_factor` (double, default: 0.95, min: 0.0, max: 1.0)

    The joint velocity limits (from URDF) are multipled by a factor to determine safe operation.

* `~safe_distance` (double, min: 0.0)

    Safe distance used in self-collision check calculation.

* `~publish_as_joint_state` (bool, default: true)

    When true the commands are published as `sensor_msgs/JointState` messages, otherwise `std_msgs/Float64MultiArray` messages are used.

* `~check_joint_position_limits` (bool, default: true)

    When true the joint position limits are checked, false otherwise.

* `~check_joint_velocity_limits` (bool, default: true)

    When true the joint velocity limits are checked, false otherwise.


* `~check_link_limits` (bool, default: true)

    When true the end-effector/link limits listed in `~safe_links` (above) are checked, false otherwise.

* `~check_self_collision` (bool, default: true)

    When true the target state is checked for self-collisions, false otherwise. This utilizes the functionality of EXOTica.

* `~stop_after_first_fail` (bool, default: true)

    When true all future target states are prevented.

* `~debug` (bool, default: false)

    When true debug mode is turned on.

* `~joint_name_order` (string, default: ordering specified by EXOTica)

    This parameter can be used to specify a joint ordering that is different to that is required by a controller. For example, when a robot accepts `std_msgs/Float64MultiArray` messages, the ordering of the joint positions is critical. Some controllers accept these in a different order to the given URDF.
