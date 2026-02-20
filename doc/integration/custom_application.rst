Building Custom Applications
=============================

Overview
--------

The WMX ROS2 packages expose standard ROS2 interfaces (actions, services, and
topics) that any ROS2 node can interact with. You can build custom applications
in **Python** or **C++** that control the robot without modifying the WMX ROS2
source code.

The system is designed for **generic 6-DOF robot arms** with EtherCAT servo
drives. The ROS2 interface layer is independent of the specific robot model --
only the configuration files and WMX3 parameter files are robot-specific.

There are two approaches to controlling the robot:

1. **Trajectory-based** -- Send a full trajectory to the
   ``FollowJointTrajectory`` action server (used by MoveIt2 and custom
   planners)
2. **Direct axis control** -- Publish velocity or position commands to
   individual axes via topics (used for low-level or real-time control)

Prerequisites
-------------

Before building a custom application:

- WMX ROS2 packages are installed and built
  (see :doc:`../getting_started`)
- The WMX ROS2 nodes are running (either the manipulator launch or
  the general launch)
- Your workspace is sourced: ``source ~/wmx_ros2_ws/install/setup.bash``
- You have a basic understanding of ROS2 actions, services, and topics

Available Interfaces
--------------------

.. list-table:: Actions
   :header-rows: 1
   :widths: 40 35 25

   * - Name
     - Type
     - Node
   * - ``/movensys_manipulator_arm_controller/follow_joint_trajectory``
     - ``control_msgs/action/FollowJointTrajectory``
     - ``follow_joint_trajectory_server``

.. list-table:: Key Services
   :header-rows: 1
   :widths: 30 30 40

   * - Name
     - Type
     - Purpose
   * - ``/wmx/engine/get_status``
     - ``std_srvs/srv/Trigger``
     - Query engine state
   * - ``/wmx/set_gripper``
     - ``std_srvs/srv/SetBool``
     - Open/close gripper
   * - ``/wmx/axis/set_on``
     - ``wmx_ros2_message/srv/SetAxis``
     - Enable/disable servos

.. list-table:: Key Topics
   :header-rows: 1
   :widths: 30 30 15 25

   * - Name
     - Type
     - Rate
     - Purpose
   * - ``/joint_states``
     - ``sensor_msgs/msg/JointState``
     - 500 Hz
     - Current joint positions and velocities
   * - ``/wmx/axis/state``
     - ``wmx_ros2_message/msg/AxisState``
     - 100 Hz
     - Detailed axis status (alarms, limits, torques)
   * - ``/wmx/axis/velocity``
     - ``wmx_ros2_message/msg/AxisVelocity``
     - On demand
     - Velocity commands
   * - ``/wmx/axis/position``
     - ``wmx_ros2_message/msg/AxisPose``
     - On demand
     - Absolute position commands
   * - ``/wmx/axis/position/relative``
     - ``wmx_ros2_message/msg/AxisPose``
     - On demand
     - Relative position commands

For complete field-level documentation, see :doc:`../api_reference/ros2_actions`,
:doc:`../api_reference/ros2_services`, and :doc:`../api_reference/ros2_topics`.

Python Example: Send a Trajectory
-----------------------------------

This example creates a ROS2 Python node that sends a two-point trajectory to
the ``FollowJointTrajectory`` action server. The robot moves from the current
position to the specified joint targets.

.. code-block:: python

   #!/usr/bin/env python3
   """Send a joint trajectory to the WMX ROS2 follow_joint_trajectory_server."""

   import rclpy
   from rclpy.action import ActionClient
   from rclpy.node import Node
   from control_msgs.action import FollowJointTrajectory
   from trajectory_msgs.msg import JointTrajectoryPoint
   from builtin_interfaces.msg import Duration


   class TrajectoryClient(Node):
       def __init__(self):
           super().__init__('trajectory_client')
           self._client = ActionClient(
               self,
               FollowJointTrajectory,
               '/movensys_manipulator_arm_controller/follow_joint_trajectory'
           )

       def send_trajectory(self, positions, duration_sec=3):
           """Send a trajectory that moves to the given joint positions.

           Args:
               positions: List of 6 joint angles in radians.
               duration_sec: Time to reach the target in seconds.
           """
           self.get_logger().info('Waiting for action server...')
           self._client.wait_for_server()

           goal = FollowJointTrajectory.Goal()
           goal.trajectory.joint_names = [
               'joint1', 'joint2', 'joint3',
               'joint4', 'joint5', 'joint6'
           ]

           # Start point (current position -- use zeros or read from /joint_states)
           start_point = JointTrajectoryPoint()
           start_point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
           start_point.time_from_start = Duration(sec=0, nanosec=0)

           # Target point
           target_point = JointTrajectoryPoint()
           target_point.positions = positions
           target_point.time_from_start = Duration(sec=duration_sec, nanosec=0)

           goal.trajectory.points = [start_point, target_point]

           self.get_logger().info(
               f'Sending trajectory: {positions} over {duration_sec}s'
           )
           future = self._client.send_goal_async(
               goal, feedback_callback=self._feedback_cb
           )
           future.add_done_callback(self._goal_response_cb)

       def _goal_response_cb(self, future):
           goal_handle = future.result()
           if not goal_handle.accepted:
               self.get_logger().error('Goal rejected')
               return
           self.get_logger().info('Goal accepted, waiting for result...')
           result_future = goal_handle.get_result_async()
           result_future.add_done_callback(self._result_cb)

       def _result_cb(self, future):
           result = future.result().result
           if result.error_code == 0:
               self.get_logger().info('Trajectory executed successfully')
           else:
               self.get_logger().error(
                   f'Trajectory failed with error code: {result.error_code}'
               )
           rclpy.shutdown()

       def _feedback_cb(self, feedback_msg):
           # The WMX server does not publish feedback during execution
           pass


   def main():
       rclpy.init()
       client = TrajectoryClient()

       # Move to target: joint1=0.5rad, joint2=-0.3rad, others at 0
       client.send_trajectory([0.5, -0.3, 0.2, 0.0, 0.1, 0.0], duration_sec=3)

       rclpy.spin(client)


   if __name__ == '__main__':
       main()

**Run the example** (with the manipulator nodes already running):

.. code-block:: bash

   python3 trajectory_client.py

.. warning::

   This sends real motion commands. Ensure the robot workspace is clear
   before running.

**Key constraints** (from the action server source code):

- Maximum **1000 waypoints** per trajectory
- The first point's ``time_from_start`` is forced to zero by the server
- If the last point is within 1 ms of the previous point, it is dropped
- Only ``positions`` and ``time_from_start`` are used by the WMX3 engine
- The server blocks until motion completes (no intermediate feedback)

C++ Example: Send a Trajectory
-------------------------------

This example follows the same pattern used by ``wmx_ros2_general_example.cpp``
in the workspace. It creates an action client, sends a trajectory goal, and
waits for the result.

.. code-block:: cpp

   #include <memory>
   #include <chrono>

   #include "rclcpp/rclcpp.hpp"
   #include "rclcpp_action/rclcpp_action.hpp"
   #include "control_msgs/action/follow_joint_trajectory.hpp"
   #include "trajectory_msgs/msg/joint_trajectory_point.hpp"

   using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
   using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
   using namespace std::chrono_literals;

   int main(int argc, char **argv)
   {
     rclcpp::init(argc, argv);
     auto node = rclcpp::Node::make_shared("trajectory_client_cpp");

     auto client = rclcpp_action::create_client<FollowJointTrajectory>(
       node,
       "/movensys_manipulator_arm_controller/follow_joint_trajectory"
     );

     // Wait for action server
     if (!client->wait_for_action_server(10s)) {
       RCLCPP_ERROR(node->get_logger(), "Action server not available");
       return 1;
     }

     // Build goal
     auto goal = FollowJointTrajectory::Goal();
     goal.trajectory.joint_names = {
       "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
     };

     // Start point
     trajectory_msgs::msg::JointTrajectoryPoint start_pt;
     start_pt.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
     start_pt.time_from_start = rclcpp::Duration(0, 0);

     // Target point at 3 seconds
     trajectory_msgs::msg::JointTrajectoryPoint target_pt;
     target_pt.positions = {0.5, -0.3, 0.2, 0.0, 0.1, 0.0};
     target_pt.time_from_start = rclcpp::Duration(3, 0);

     goal.trajectory.points = {start_pt, target_pt};

     RCLCPP_INFO(node->get_logger(), "Sending trajectory goal...");

     auto send_goal_options =
       rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
     send_goal_options.result_callback =
       [&node](const GoalHandleFJT::WrappedResult &result) {
         if (result.result->error_code == 0) {
           RCLCPP_INFO(node->get_logger(), "Trajectory executed successfully");
         } else {
           RCLCPP_ERROR(node->get_logger(), "Trajectory failed: error_code=%d",
                        result.result->error_code);
         }
         rclcpp::shutdown();
       };

     client->async_send_goal(goal, send_goal_options);
     rclcpp::spin(node);
     return 0;
   }

To build this in your own package, add these dependencies to your
``package.xml``:

.. code-block:: xml

   <depend>rclcpp</depend>
   <depend>rclcpp_action</depend>
   <depend>control_msgs</depend>
   <depend>trajectory_msgs</depend>

And in ``CMakeLists.txt``:

.. code-block:: cmake

   find_package(rclcpp REQUIRED)
   find_package(rclcpp_action REQUIRED)
   find_package(control_msgs REQUIRED)
   find_package(trajectory_msgs REQUIRED)

   add_executable(trajectory_client src/trajectory_client.cpp)
   ament_target_dependencies(trajectory_client
     rclcpp rclcpp_action control_msgs trajectory_msgs)

Python Example: Read Robot State
---------------------------------

This example subscribes to ``/joint_states`` and prints the current joint
positions. The ``manipulator_state`` node publishes this topic at 500 Hz
with 8 values (6 joints + 2 gripper fingers).

.. code-block:: python

   #!/usr/bin/env python3
   """Subscribe to /joint_states and print joint positions."""

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import JointState


   class JointStateReader(Node):
       def __init__(self):
           super().__init__('joint_state_reader')
           self._sub = self.create_subscription(
               JointState, '/joint_states', self._callback, 10
           )

       def _callback(self, msg):
           # msg.name: 8 names (joint1-6 + picker_1_joint, picker_2_joint)
           # msg.position: 8 values in radians
           # msg.velocity: 8 values in rad/s (gripper velocities are always 0.0)
           joint_positions = dict(zip(msg.name, msg.position))
           self.get_logger().info(
               f'Positions: ' +
               ', '.join(f'{n}={p:.4f}' for n, p in joint_positions.items())
           )


   def main():
       rclpy.init()
       node = JointStateReader()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()

**Run the example:**

.. code-block:: bash

   python3 joint_state_reader.py

Expected output (values depend on current robot position):

.. code-block:: text

   [joint_state_reader] Positions: joint1=0.0012, joint2=-0.3021, joint3=0.1500, joint4=0.0003, joint5=0.0998, joint6=-0.0001, picker_1_joint=0.0000, picker_2_joint=0.0000

Python Example: Monitor Axis State and Control Gripper
-------------------------------------------------------

This example shows how to query the engine status, monitor detailed axis
diagnostics, and control the gripper -- combining services and topics.

.. code-block:: python

   #!/usr/bin/env python3
   """Query engine status and control gripper."""

   import rclpy
   from rclpy.node import Node
   from std_srvs.srv import Trigger, SetBool


   class RobotController(Node):
       def __init__(self):
           super().__init__('robot_controller')
           self._status_client = self.create_client(
               Trigger, '/wmx/engine/get_status'
           )
           self._gripper_client = self.create_client(
               SetBool, '/wmx/set_gripper'
           )

       def get_engine_status(self):
           """Query the WMX3 engine state."""
           self._status_client.wait_for_service()
           future = self._status_client.call_async(Trigger.Request())
           rclpy.spin_until_future_complete(self, future)
           result = future.result()
           self.get_logger().info(f'Engine status: {result.message}')
           return result.message

       def set_gripper(self, close: bool):
           """Open or close the gripper.

           Args:
               close: True to close, False to open.
           """
           self._gripper_client.wait_for_service()
           request = SetBool.Request()
           request.data = close
           future = self._gripper_client.call_async(request)
           rclpy.spin_until_future_complete(self, future)
           result = future.result()
           self.get_logger().info(
               f'Gripper: success={result.success}, {result.message}'
           )
           return result.success


   def main():
       rclpy.init()
       controller = RobotController()

       # Check engine is communicating
       status = controller.get_engine_status()
       if status != 'Communicating':
           controller.get_logger().error(
               f'Engine not ready (state: {status}). '
               'Launch the manipulator nodes first.'
           )
           rclpy.shutdown()
           return

       # Close gripper, wait, then open
       controller.set_gripper(close=True)
       import time; time.sleep(2.0)
       controller.set_gripper(close=False)

       controller.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()

Using with MoveIt2 Python API
-------------------------------

If MoveIt2 is installed and configured for your robot, you can use the
``MoveGroupInterface`` to plan and execute trajectories. MoveIt2 internally
sends goals to the same ``FollowJointTrajectory`` action server.

.. code-block:: python

   #!/usr/bin/env python3
   """MoveIt2 Python example using moveit_py."""

   import rclpy
   from rclpy.node import Node


   def main():
       rclpy.init()
       node = Node('moveit2_example')

       # MoveIt2 Python bindings (moveit_py) require the MoveIt configuration
       # package for your robot to be available.
       #
       # The typical workflow is:
       # 1. Launch the WMX ROS2 manipulator nodes (provides /joint_states)
       # 2. Launch MoveIt2 with your robot's config package
       # 3. Use MoveGroupInterface to plan and execute
       #
       # Example using the MoveIt2 Python API:
       #
       #   from moveit.planning import MoveItPy
       #   moveit = MoveItPy(node_name="moveit_py_node")
       #   arm = moveit.get_planning_component("manipulator_arm")
       #   arm.set_start_state_to_current_state()
       #   arm.set_goal_state(configuration_name="home")
       #   plan = arm.plan()
       #   if plan:
       #       arm.execute()

       node.get_logger().info(
           'MoveIt2 integration requires:\n'
           '  1. WMX ROS2 manipulator nodes running\n'
           '  2. MoveIt2 config package for your robot\n'
           '  3. MoveIt2 move_group node running\n'
           'See: ros2 launch <your_moveit_config> move_group.launch.py'
       )

       node.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()

MoveIt2 handles trajectory planning, collision avoidance, and sends the
result to the ``follow_joint_trajectory_server`` via the
``FollowJointTrajectory`` action. See
:doc:`moveit2_integration` for configuration details.

Direct Axis Control
--------------------

For applications that need real-time velocity or position control without
trajectory planning, publish directly to the motion topics. This requires
the ``wmx_ros2_general_node`` to be running.

.. code-block:: python

   #!/usr/bin/env python3
   """Direct axis velocity control via topic publishing."""

   import rclpy
   from rclpy.node import Node
   from wmx_ros2_message.msg import AxisVelocity, AxisPose


   class DirectAxisControl(Node):
       def __init__(self):
           super().__init__('direct_axis_control')
           self._vel_pub = self.create_publisher(
               AxisVelocity, '/wmx/axis/velocity', 1
           )
           self._pos_pub = self.create_publisher(
               AxisPose, '/wmx/axis/position', 1
           )
           self._rel_pub = self.create_publisher(
               AxisPose, '/wmx/axis/position/relative', 1
           )

       def send_velocity(self, axis_indices, velocities, acc=10.0, dec=10.0):
           """Command velocity motion on specified axes.

           The axes must be in velocity mode (set via /wmx/axis/set_mode
           with data=[1]).
           """
           msg = AxisVelocity()
           msg.index = axis_indices
           msg.profile = ''
           msg.velocity = velocities
           msg.acc = [acc] * len(axis_indices)
           msg.dec = [dec] * len(axis_indices)
           self._vel_pub.publish(msg)
           self.get_logger().info(
               f'Velocity command: axes={axis_indices}, vel={velocities}'
           )

       def send_position(self, axis_indices, targets, vel=5.0, acc=10.0, dec=10.0):
           """Command absolute position motion on specified axes."""
           msg = AxisPose()
           msg.index = axis_indices
           msg.target = targets
           msg.profile = ''
           msg.velocity = [vel] * len(axis_indices)
           msg.acc = [acc] * len(axis_indices)
           msg.dec = [dec] * len(axis_indices)
           self._pos_pub.publish(msg)
           self.get_logger().info(
               f'Position command: axes={axis_indices}, targets={targets}'
           )

       def send_relative_position(self, axis_indices, displacements,
                                   vel=5.0, acc=10.0, dec=10.0):
           """Command relative position motion on specified axes."""
           msg = AxisPose()
           msg.index = axis_indices
           msg.target = displacements
           msg.profile = ''
           msg.velocity = [vel] * len(axis_indices)
           msg.acc = [acc] * len(axis_indices)
           msg.dec = [dec] * len(axis_indices)
           self._rel_pub.publish(msg)
           self.get_logger().info(
               f'Relative move: axes={axis_indices}, disp={displacements}'
           )


   def main():
       rclpy.init()
       ctrl = DirectAxisControl()

       # Move axis 0 to 1.0 radian absolute
       ctrl.send_position([0], [1.0])

       import time; time.sleep(5.0)

       # Move axis 0 by +0.5 radian relative
       ctrl.send_relative_position([0], [0.5])

       ctrl.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()

.. important::

   Direct axis commands require the axes to be properly initialized first
   (servo enabled, mode set, homed). When using the manipulator launch file,
   this is handled automatically by ``manipulator_state``. When using the
   general launch, call the setup services first as described in the
   :doc:`../api_reference/ros2_services` workflow section.

.. warning::

   Publishing to motion topics causes **immediate physical motion**. These
   commands bypass MoveIt2 collision checking.

Adapting for Different 6-DOF Robots
-------------------------------------

The WMX ROS2 system is designed to be robot-agnostic. Supporting a new 6-DOF
manipulator with EtherCAT servo drives requires changes to configuration files
only -- no source code modifications are needed.

Configuration files to create or modify
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - File
     - Changes Required
   * - **YAML config** (e.g., ``new_robot_config.yaml``)
     - Set ``joint_number``, ``joint_name`` list, ``joint_feedback_rate``,
       gripper values, topic names, and ``wmx_param_file_path``
   * - **WMX3 XML parameters** (e.g., ``new_robot_wmx_parameters.xml``)
     - Define gear ratios, axis polarities, encoder modes, homing parameters,
       and limit switch settings for each servo axis
   * - **Launch file** (e.g., ``wmx_ros2_new_robot.launch.py``)
     - Point to the new YAML config and launch the 3 standard nodes
   * - **ENI files** (in ``eni/`` directory)
     - Add EtherCAT Network Information files for any new servo drive models
       not already supported

Steps to adapt
^^^^^^^^^^^^^^^

1. **Identify your servo drives** -- Determine the vendor and product IDs of
   each EtherCAT servo drive in your robot. Check if matching ENI files exist
   in the ``eni/`` directory.

2. **Create the WMX3 parameter file** -- Copy ``cr3a_wmx_parameters.xml`` and
   modify gear ratios, polarities, and encoder settings for your servo drives.
   The gear ratio maps encoder counts to radians:
   ``numerator = encoder_counts_per_revolution``,
   ``denominator = 2 * pi (6.28319)``.

3. **Create the YAML config** -- Copy ``intel_manipulator_config_cr3a.yaml``
   and update:

   .. code-block:: yaml

      manipulator_state:
        ros__parameters:
          joint_number: 6           # Must be 6 for the current system
          joint_feedback_rate: 500  # Hz
          joint_name: ["joint1", "joint2", "joint3", "joint4",
                        "joint5", "joint6",
                        "picker_1_joint", "picker_2_joint"]
          wmx_param_file_path: /path/to/new_robot_wmx_parameters.xml

4. **Create a launch file** -- Copy an existing launch file and reference your
   new YAML config.

5. **Update URDF/SRDF** (if using MoveIt2) -- Create a MoveIt2 configuration
   package for your robot with the correct kinematics, joint limits, and
   collision geometry.

What stays the same
^^^^^^^^^^^^^^^^^^^^

- All ROS2 node executables (``manipulator_state``,
  ``follow_joint_trajectory_server``, ``wmx_ros2_general_node``)
- All service and topic names
- The ``FollowJointTrajectory`` action interface
- The ``wmx_ros2_message`` custom message types
- The build process

See :doc:`../architecture/system_overview` for the full architecture and
:doc:`../packages/wmx_ros2_package` for node and parameter details.
