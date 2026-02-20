ROS2 Actions
=============

The WMX ROS2 application exposes one action server for trajectory execution,
compatible with MoveIt2 and any ``FollowJointTrajectory`` action client. No
custom ``.action`` files are defined -- the system uses the standard
``control_msgs`` action type.

.. list-table:: Action Summary
   :header-rows: 1
   :widths: 35 30 20 15

   * - Action Name
     - Type
     - Server Node
     - Status
   * - ``/movensys_manipulator_arm_controller/follow_joint_trajectory``
     - ``control_msgs/action/FollowJointTrajectory``
     - ``follow_joint_trajectory_server``
     - Active

FollowJointTrajectory
----------------------

.. list-table::
   :widths: 25 75

   * - **Action Name**
     - ``/movensys_manipulator_arm_controller/follow_joint_trajectory``
   * - **Action Type**
     - ``control_msgs/action/FollowJointTrajectory``
   * - **Server Node**
     - ``follow_joint_trajectory_server``
   * - **Configurable**
     - Name set via ``joint_trajectory_action`` parameter
   * - **Source File**
     - ``follow_joint_trajectory_server.cpp``

This action server receives a joint-space trajectory (a sequence of waypoints
with timestamps) and executes it on the physical robot using the WMX3
``AdvancedMotion::StartCSplinePos()`` cubic spline interpolation engine.

Goal
^^^^^

The goal uses the standard ``trajectory_msgs/msg/JointTrajectory`` message:

.. list-table::
   :header-rows: 1
   :widths: 30 20 50

   * - Field
     - Type
     - Description
   * - ``trajectory.joint_names``
     - ``string[]``
     - Joint names (``joint1`` through ``joint6``)
   * - ``trajectory.points[]``
     - ``JointTrajectoryPoint[]``
     - Ordered list of waypoints (maximum **1000 points**)
   * - ``trajectory.points[].positions``
     - ``float64[]``
     - Target joint positions in radians for each waypoint
   * - ``trajectory.points[].time_from_start``
     - ``duration``
     - Timestamp relative to trajectory start

.. note::

   Only the ``positions`` and ``time_from_start`` fields of each trajectory
   point are used. The ``velocities``, ``accelerations``, and ``effort``
   fields are logged for diagnostics but not passed to the WMX3 engine --
   the cubic spline interpolation is computed internally by WMX3.

Result
^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 20 50

   * - Field
     - Type
     - Description
   * - ``error_code``
     - ``int32``
     - ``0`` on success; WMX3 error code on failure
   * - ``error_string``
     - ``string``
     - Not set by the server (default empty)

Feedback
^^^^^^^^^

No intermediate feedback is published during execution. The action server
blocks on ``CoreMotion::Wait()`` until the spline motion completes or an error
occurs.

Execution Details
^^^^^^^^^^^^^^^^^^

The server processes the trajectory as follows:

1. **Goal acceptance** -- All incoming goals are accepted unconditionally
   (``ACCEPT_AND_EXECUTE``). No goal validation beyond point count is performed.

2. **Thread dispatch** -- The ``execute()`` callback runs in a **detached
   thread** spawned from ``handle_accepted()``, allowing the action server
   to remain responsive.

3. **Validation** -- Rejects goals with more than 1000 waypoints (aborts
   immediately with a warning). If the adjusted point count is 0, the server
   succeeds immediately without commanding motion.

4. **Timing adjustment** -- The first point's ``time_from_start`` is forced
   to zero. If the last point has a time interval less than 1 ms from the
   previous point, it is dropped to prevent interpolation errors.

5. **Spline construction** -- For each trajectory point, positions are packed
   into a ``CSplinePosData`` structure and timestamps are converted to
   milliseconds. The ``dimensionCount`` and ``axis[]`` array are set to the
   ``joint_number`` parameter (typically 6).

6. **Execution** -- ``AdvancedMotion::StartCSplinePos(0, ...)`` begins the
   interpolated motion on buffer index 0 across all joints simultaneously.

7. **Wait** -- The server blocks on ``CoreMotion::Wait()`` for all axes
   in the selection.

8. **Result** -- On success, ``error_code = 0`` and the goal succeeds.
   On failure, the WMX3 error code is returned and the goal is aborted.

.. note::

   Cancel requests are accepted by the ``handle_cancel()`` callback, but the
   blocking ``Wait()`` call does not currently check ``goal_handle->is_canceling()``.
   A ``TODO`` in the source code notes this limitation.

Example Usage
^^^^^^^^^^^^^^

**Send a simple two-point trajectory via the command line:**

.. code-block:: bash

   ros2 action send_goal \
     /movensys_manipulator_arm_controller/follow_joint_trajectory \
     control_msgs/action/FollowJointTrajectory \
     "{trajectory: {
       joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
       points: [
         {positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          time_from_start: {sec: 0, nanosec: 0}},
         {positions: [0.5, -0.3, 0.2, 0.0, 0.1, 0.0],
          time_from_start: {sec: 3, nanosec: 0}}
       ]
     }}"

**Check that the action server is available:**

.. code-block:: bash

   ros2 action list

Expected:

.. code-block:: text

   /movensys_manipulator_arm_controller/follow_joint_trajectory

**Inspect the action interface:**

.. code-block:: bash

   ros2 action info /movensys_manipulator_arm_controller/follow_joint_trajectory

.. warning::

   This action moves real motors. Ensure the robot workspace is clear and
   all safety precautions are in place before sending trajectory goals.

MoveIt2 Integration
^^^^^^^^^^^^^^^^^^^^

MoveIt2 connects to this action server as a trajectory execution controller.
The typical workflow is:

1. MoveIt2 reads the current robot state from ``/joint_states``
2. The planner computes a collision-free trajectory
3. MoveIt2 sends the trajectory as a ``FollowJointTrajectory`` goal
4. The ``follow_joint_trajectory_server`` executes it via WMX3 cubic spline
5. The ``manipulator_state`` node publishes real-time encoder feedback back
   to ``/joint_states`` at 500 Hz

The action server name must match the controller configuration in MoveIt2.
The default name ``/movensys_manipulator_arm_controller/follow_joint_trajectory``
is set via the ``joint_trajectory_action`` parameter in the config YAML.

See Also
^^^^^^^^

- :doc:`ros2_topics` -- ``/joint_states`` topic details
- :doc:`ros2_services` -- ``/wmx/set_gripper`` service on the same node
- :doc:`../architecture/flowcharts` -- Motion execution flow diagram
- :doc:`../integration/moveit2_integration` -- MoveIt2 setup
