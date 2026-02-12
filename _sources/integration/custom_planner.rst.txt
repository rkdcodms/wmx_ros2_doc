Custom Planner Integration
===========================

Overview
--------

The WMX ROS2 application supports any motion planner that can send
trajectories via the standard ``FollowJointTrajectory`` action interface.
This allows integration of custom planners without modifying the WMX ROS2
nodes.

The architecture is:

.. code-block:: text

   ┌──────────────────────────┐
   │   Your Custom Planner    │
   │                          │
   │  1. Read /joint_states   │──── sensor_msgs/JointState (500 Hz)
   │  2. Compute trajectory   │
   │  3. Send action goal     │──── FollowJointTrajectory
   │  4. Wait for result      │
   └──────────────────────────┘
               │
               ▼
   ┌──────────────────────────────────────────────────────────┐
   │  follow_joint_trajectory_server                          │
   │                                                          │
   │  - Validates trajectory (max 1000 points)                │
   │  - Converts to WMX3 CSplinePos command                   │
   │  - Executes via AdvancedMotion API                       │
   │  - Blocks until complete, returns error_code             │
   └──────────────────────────────────────────────────────────┘
               │
               ▼
         EtherCAT → Servo Drives → Robot Motion

Replacing the Default Planner
------------------------------

MoveIt2 is the default planner, but you can replace it entirely. Any node
that implements the following two-step pattern can control the robot:

1. **Read current joint state** from ``/joint_states``
   (``sensor_msgs/msg/JointState``) -- published by ``manipulator_state``
   at 500 Hz

2. **Send trajectory goals** to the ``FollowJointTrajectory`` action server
   at ``/movensys_manipulator_arm_controller/follow_joint_trajectory``
   (``control_msgs/action/FollowJointTrajectory``)

No other setup is required. The manipulator launch already starts the
trajectory server, encoder publisher, and general control node.

Interface Requirements
----------------------

The trajectory goal must contain:

- ``trajectory.joint_names``: must match the robot configuration
  (``["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]``)
- ``trajectory.points[].positions``: target positions in radians for each
  waypoint (6 values per point)
- ``trajectory.points[].time_from_start``: timestamp for each waypoint

Constraints and Limitations
----------------------------

These constraints are enforced by the ``follow_joint_trajectory_server``
source code:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Constraint
     - Details
   * - Max waypoints
     - **1000** per trajectory (``MAX_TRAJ_POINTS`` in source). Goals with
       more points are aborted immediately.
   * - Min time interval
     - **1 ms** between the last two consecutive waypoints. If the final
       point is within 1 ms of the previous point, it is dropped.
   * - First point timing
     - The first point's ``time_from_start`` is forced to **zero** by the
       server, regardless of the value you set.
   * - Used fields
     - Only ``positions`` and ``time_from_start`` are passed to the WMX3
       engine. The ``velocities``, ``accelerations``, and ``effort`` fields
       are logged but not used for motion -- the cubic spline interpolation
       is computed internally by WMX3.
   * - Goal acceptance
     - All goals are accepted unconditionally (``ACCEPT_AND_EXECUTE``).
       No kinematic or collision validation is performed by the server.
   * - Cancellation
     - Cancel requests are accepted but **not acted upon** during motion.
       The server blocks on ``CoreMotion::Wait()`` and does not check
       cancellation status. (``TODO`` noted in source code.)
   * - Feedback
     - No intermediate feedback is published. The action completes only
       when the full trajectory has been executed or an error occurs.
   * - Concurrency
     - The execute callback runs in a detached thread, but only one
       trajectory can be executed at a time on the same set of axes.

See :doc:`../api_reference/ros2_actions` for the complete action interface
documentation.

Python Example: Custom Trajectory Generator
---------------------------------------------

This complete example reads the current joint positions, computes a simple
multi-point trajectory, sends it to the action server, and waits for the
result.

.. code-block:: python

   #!/usr/bin/env python3
   """Custom planner example: generate and execute a multi-point trajectory."""

   import math
   import rclpy
   from rclpy.action import ActionClient
   from rclpy.node import Node
   from sensor_msgs.msg import JointState
   from control_msgs.action import FollowJointTrajectory
   from trajectory_msgs.msg import JointTrajectoryPoint
   from builtin_interfaces.msg import Duration


   JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
   ACTION_NAME = '/movensys_manipulator_arm_controller/follow_joint_trajectory'


   class CustomPlanner(Node):
       def __init__(self):
           super().__init__('custom_planner')
           self._current_positions = None

           # Subscribe to joint states to know current position
           self._joint_sub = self.create_subscription(
               JointState, '/joint_states', self._joint_cb, 10
           )

           # Action client for trajectory execution
           self._action_client = ActionClient(
               self, FollowJointTrajectory, ACTION_NAME
           )

       def _joint_cb(self, msg):
           # Extract only the 6 arm joints (ignore gripper at indices 6, 7)
           if len(msg.position) >= 6:
               self._current_positions = list(msg.position[:6])

       def wait_for_joint_state(self, timeout_sec=5.0):
           """Wait until we receive at least one joint state message."""
           self.get_logger().info('Waiting for /joint_states...')
           start = self.get_clock().now()
           while self._current_positions is None:
               rclpy.spin_once(self, timeout_sec=0.1)
               elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
               if elapsed > timeout_sec:
                   self.get_logger().error('Timeout waiting for /joint_states')
                   return False
           self.get_logger().info(
               f'Current positions: {self._current_positions}'
           )
           return True

       def generate_trajectory(self, target_positions, num_points=10,
                                total_time_sec=5.0):
           """Generate a linear interpolation trajectory.

           Args:
               target_positions: Target joint angles (6 values, radians).
               num_points: Number of intermediate waypoints.
               total_time_sec: Total trajectory duration.

           Returns:
               List of JointTrajectoryPoint objects.
           """
           start = self._current_positions
           points = []
           for i in range(num_points + 1):
               t = i / num_points
               positions = [
                   s + t * (e - s) for s, e in zip(start, target_positions)
               ]
               time_sec = t * total_time_sec
               sec = int(time_sec)
               nanosec = int((time_sec - sec) * 1e9)

               pt = JointTrajectoryPoint()
               pt.positions = positions
               pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
               points.append(pt)

           return points

       def execute_trajectory(self, points):
           """Send trajectory to the action server and wait for result."""
           self.get_logger().info('Waiting for action server...')
           self._action_client.wait_for_server()

           goal = FollowJointTrajectory.Goal()
           goal.trajectory.joint_names = JOINT_NAMES
           goal.trajectory.points = points

           self.get_logger().info(
               f'Sending trajectory with {len(points)} points'
           )
           future = self._action_client.send_goal_async(goal)
           rclpy.spin_until_future_complete(self, future)

           goal_handle = future.result()
           if not goal_handle.accepted:
               self.get_logger().error('Goal rejected')
               return False

           self.get_logger().info('Goal accepted, executing...')
           result_future = goal_handle.get_result_async()
           rclpy.spin_until_future_complete(self, result_future)

           result = result_future.result().result
           if result.error_code == 0:
               self.get_logger().info('Trajectory executed successfully')
               return True
           else:
               self.get_logger().error(
                   f'Trajectory failed: error_code={result.error_code}'
               )
               return False


   def main():
       rclpy.init()
       planner = CustomPlanner()

       if not planner.wait_for_joint_state():
           rclpy.shutdown()
           return

       # Define target: move joint1 by +0.3 rad, joint2 by -0.2 rad
       current = planner._current_positions
       target = list(current)
       target[0] += 0.3
       target[1] -= 0.2

       # Generate 10-point trajectory over 5 seconds
       points = planner.generate_trajectory(target, num_points=10,
                                             total_time_sec=5.0)
       planner.execute_trajectory(points)

       planner.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()

.. warning::

   This example moves real motors. No collision checking is performed --
   ensure the robot workspace is clear before running.

Standalone Axis Control
-----------------------

For applications that don't use trajectory-based planning, the
``wmx_ros2_general_node`` provides direct axis control through:

- **Velocity commands** via ``/wmx/axis/velocity``
  (``wmx_ros2_message/msg/AxisVelocity``)
- **Absolute position commands** via ``/wmx/axis/position``
  (``wmx_ros2_message/msg/AxisPose``)
- **Relative position commands** via ``/wmx/axis/position/relative``
  (``wmx_ros2_message/msg/AxisPose``)

These topics bypass the action server and control motors directly using
``CoreMotion::StartVel()``, ``CoreMotion::StartPos()``, and
``CoreMotion::StartMov()`` respectively. All commands use trapezoidal
velocity profiles.

See :doc:`custom_application` for Python examples using direct axis control
and :doc:`../api_reference/ros2_topics` for message field details.

The full setup sequence for standalone axis control is documented in
:doc:`../api_reference/ros2_services` (Service Call Workflow section).

See Also
--------

- :doc:`../api_reference/ros2_actions` -- Complete action interface reference
- :doc:`../api_reference/ros2_topics` -- Topic message formats
- :doc:`../api_reference/ros2_services` -- Service call workflow
- :doc:`custom_application` -- Building custom applications with code examples
- :doc:`moveit2_integration` -- MoveIt2 configuration
- :doc:`../architecture/flowcharts` -- Trajectory execution flow diagram
